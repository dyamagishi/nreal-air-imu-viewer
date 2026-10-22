"""ローカルHTTP + SSE。HIDは専用スレッドで読み、ブラウザに50Hzで配信。"""
import argparse
from collections import deque
import csv
from datetime import datetime
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import logging
from pathlib import Path
import signal
import struct
import threading
import time
from urllib.parse import urlparse

import hid

from .device import Air, decode_imu
from .pose import Pose

ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT/'web'
LOG = logging.getLogger('air')


class Sensors:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.recenter = threading.Event()
        self.translation_reset = threading.Event()
        self.zupt_enabled = False
        self.history = deque(maxlen=3000)  # 50Hzで60秒
        self.latest = None
        self.status = dict(state='connecting', message='Airを探しています', samples=0)
        self.sequence = 0

    def set_status(self, **values):
        with self.lock:
            self.status.update(values)
            self.sequence += 1

    def snapshot(self):
        with self.lock:
            return dict(sequence=self.sequence, status=self.status.copy(), sample=self.latest)

    def run(self):
        while not self.stop.is_set():
            air = None
            started = False
            try:
                self.set_status(state='connecting', message='本体情報・校正データを読み取り中', samples=0)
                if not hid.enumerate(0x3318,0x0424):
                    raise RuntimeError('Airが見つかりません。USB-C接続を確認してください')
                output = ROOT/'data/live'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
                output.mkdir(parents=True)
                air = Air(output, log_packets=False)
                air.command(3,0x19,b'\0')
                air.factory_config()
                factory = json.loads((output/'factory_calibration.json').read_text())
                metadata = {}
                for key,cmd in [('serial',0x15),('firmware',0x26)]:
                    reply = air.command(4,cmd)
                    metadata[key] = reply[1:].decode('ascii',errors='replace')
                pose = Pose(factory=factory['IMU']['device_1'])
                started = True
                air.command(3,0x19,b'\1')
                with self.lock:
                    self.history.clear(); self.latest = None
                self.set_status(state='waiting', message='IMUの応答待ち', **metadata)
                total = 0; previous_ns = None; gaps = 0
                last_sample = time.monotonic(); last_publish = 0
                rate_start = last_sample; rate_count = 0; rate = 0
                latest = None; other = 0; last_event = None
                while not self.stop.is_set():
                    for iface,device in air.handles.items():
                        for _ in range(128):
                            packet = bytes(device.read(128))
                            if not packet:
                                break
                            if iface == 3 and packet[:2] == b'\x01\x02':
                                sample = decode_imu(packet)
                                sample['host_ns'] = time.time_ns()
                                pose.zupt_enabled=self.zupt_enabled
                                if self.translation_reset.is_set():
                                    pose.reset_translation(); self.translation_reset.clear()
                                sample['pose'] = pose.update(sample)
                                if self.recenter.is_set():
                                    pose.center(); self.recenter.clear()
                                    sample['pose'] = pose.output()
                                if previous_ns is not None:
                                    delta = sample['device_ns']-previous_ns
                                    if delta <= 0 or delta > 2_000_000:
                                        gaps += 1
                                previous_ns = sample['device_ns']
                                total += 1; rate_count += 1
                                latest = sample
                                last_sample = time.monotonic()
                            else:
                                other += 1
                                if iface == 4 and len(packet)==64 and packet[0]==0xfd:
                                    cmd = struct.unpack_from('<H',packet,15)[0]
                                    last_event = f'0x{cmd:04X} / {packet[22:32].hex(" ")}'
                    now = time.monotonic()
                    if now-last_sample > 2:
                        raise TimeoutError('IMUの応答が途絶えました。接続を確認してください')
                    if now-rate_start >= 1:
                        rate = rate_count/(now-rate_start)
                        rate_count=0; rate_start=now
                    if latest and now-last_publish >= .02:
                        with self.lock:
                            self.latest = latest
                            self.history.append(latest)
                            self.status.update(state='live',message='実機から取得中',samples=total,
                                rate_hz=round(rate), gaps=gaps, other_packets=other,last_event=last_event)
                            self.sequence += 1
                        last_publish = now
                    self.stop.wait(.0005)
            except Exception as exc:
                LOG.exception('HIDセッション終了')
                with self.lock:
                    self.latest = None
                self.set_status(state='disconnected',message=str(exc),rate_hz=0)
            finally:
                if air:
                    try:
                        if started:
                            air.command(3,0x19,b'\0')
                    except Exception:
                        LOG.debug('切断済みデバイスの配信停止を省略',exc_info=True)
                    air.close()
            self.stop.wait(2)


class Handler(SimpleHTTPRequestHandler):
    sensors = None

    def __init__(self,*args,**kwargs):
        super().__init__(*args,directory=str(WEB),**kwargs)

    def log_message(self, fmt, *args):
        LOG.debug(fmt,*args)

    def json_response(self, data, status=200):
        encoded = json.dumps(data,allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(encoded)))
        self.end_headers(); self.wfile.write(encoded)

    def do_GET(self):
        path = urlparse(self.path).path
        if path=='/api/state':
            return self.json_response(self.sensors.snapshot())
        if path=='/events':
            self.send_response(200)
            self.send_header('Content-Type','text/event-stream')
            self.send_header('Cache-Control','no-cache')
            self.end_headers()
            sequence = -1
            try:
                while not self.sensors.stop.is_set():
                    state = self.sensors.snapshot()
                    if state['sequence'] != sequence:
                        self.wfile.write(('data: '+json.dumps(state,allow_nan=False)+'\n\n').encode())
                        self.wfile.flush(); sequence=state['sequence']
                    self.sensors.stop.wait(.02)
            except (BrokenPipeError,ConnectionResetError,TimeoutError):
                pass
            return
        if path=='/api/download.csv':
            with self.sensors.lock:
                rows=list(self.sensors.history)
            text=io.StringIO(); fields=['host_ns','device_ns','temperature_c']
            fields += [f'{g}_{a}' for g in ('gyro_dps','accel_m_s2','mag_scaled') for a in 'xyz']
            fields += [f'vqf_{mode}_{axis}' for mode in ('imu','mag') for axis in ('yaw','pitch','roll')]
            fields += [f'vqf_bias_{axis}_dps' for axis in 'xyz']
            fields += ['vqf_rest_detected','vqf_mag_disturbance','zupt_enabled','translation_generation']
            fields += [f'{mode}_{group}_{axis}' for mode in ('imu','mag')
                       for group in ('position_m','velocity_m_s','linear_acceleration_m_s2') for axis in 'xyz']
            writer=csv.DictWriter(text,fieldnames=fields); writer.writeheader()
            for s in rows:
                row={k:s[k] for k in fields[:3]}
                row.update({f'{g}_{a}':s[g][i] for g in ('gyro_dps','accel_m_s2','mag_scaled') for i,a in enumerate('xyz')})
                pose=s['pose']
                row.update({f'vqf_{mode}_{axis}':pose['variants'][mode][axis]
                            for mode in ('imu','mag') for axis in ('yaw','pitch','roll')})
                row.update({f'vqf_bias_{axis}_dps':pose['bias_dps'][i] for i,axis in enumerate('xyz')})
                row.update(vqf_rest_detected=int(pose['rest_detected']),
                           vqf_mag_disturbance=int(pose['magnetic_disturbance']))
                row.update(zupt_enabled=int(pose['zupt_enabled']),translation_generation=pose['translation_generation'])
                for mode in ('imu','mag'):
                    translation=pose['variants'][mode]['translation']
                    for column,key in [('position_m','position_world_m'),('velocity_m_s','velocity_world_m_s'),
                                       ('linear_acceleration_m_s2','linear_acceleration_world_m_s2')]:
                        row.update({f'{mode}_{column}_{axis}':translation[key][i] for i,axis in enumerate('xyz')})
                writer.writerow(row)
            encoded=text.getvalue().encode('utf-8-sig')
            self.send_response(200); self.send_header('Content-Type','text/csv; charset=utf-8')
            self.send_header('Content-Disposition','attachment; filename="nreal-air.csv"')
            self.send_header('Content-Length',str(len(encoded))); self.end_headers(); self.wfile.write(encoded)
            return
        return super().do_GET()

    def do_POST(self):
        # 他サイトからのフォーム送信やfetchでは操作できない。
        if self.headers.get('X-Air-Viewer')!='1':
            return self.json_response(dict(error='ヘッダーが必要です'),403)
        if urlparse(self.path).path=='/api/center':
            self.sensors.recenter.set()
            return self.json_response(dict(ok=True))
        if urlparse(self.path).path=='/api/translation/reset':
            self.sensors.translation_reset.set()
            return self.json_response(dict(ok=True))
        if urlparse(self.path).path=='/api/translation/zupt':
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=256:
                    raise ValueError('不正なサイズ')
                value=json.loads(self.rfile.read(size))['enabled']
                if type(value) is not bool:
                    raise ValueError('boolが必要です')
            except (ValueError,KeyError,TypeError):
                return self.json_response(dict(error='enabledにはtrueかfalseが必要です'),400)
            self.sensors.zupt_enabled=value
            return self.json_response(dict(ok=True,enabled=value))
        self.json_response(dict(error='Not found'),404)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    (ROOT/'data').mkdir(exist_ok=True)
    logging.basicConfig(level=logging.INFO,filename=ROOT/'data/viewer.log',
                        format='%(asctime)s %(levelname)s %(message)s')
    sensors=Sensors(); Handler.sensors=sensors
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    server.daemon_threads=True
    worker=threading.Thread(target=sensors.run,name='air-hid',daemon=True); worker.start()
    def stop(signum,frame):
        sensors.stop.set()
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
    print(f'Air Viewer: http://127.0.0.1:{args.port}',flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        sensors.stop.set(); worker.join(timeout=5); server.server_close()


if __name__=='__main__':
    main()
