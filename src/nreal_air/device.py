import json
import struct
import time
import zlib
import hid

def decode_imu(p):
    """センサー固有軸で復号。磁気値はoffset binary、倍率はbig endian。"""
    if len(p) != 64 or p[:2] != b'\x01\x02':
        raise ValueError('対応していないIMUレポート')
    def vector24(offset):
        return [int.from_bytes(p[i:i+3], 'little', signed=True)
                for i in range(offset, offset+9, 3)]
    gm, gd = struct.unpack_from('<hI', p, 12)
    am, ad = struct.unpack_from('<hI', p, 27)
    mm, md = struct.unpack_from('>hI', p, 42)
    if not all((gd, ad, md)):
        raise ValueError('センサー換算分母が0')
    gyro = vector24(18)
    accel = vector24(33)
    mag = [v - 32768 for v in struct.unpack_from('<3H', p, 48)]
    return dict(
        device_ns=struct.unpack_from('<Q', p, 4)[0],
        temperature_raw=struct.unpack_from('<h', p, 2)[0],
        temperature_c=struct.unpack_from('<h', p, 2)[0]/132.48+25,
        gyro_raw=gyro, accel_raw=accel, mag_raw=mag,
        gyro_dps=[v*gm/gd for v in gyro],
        accel_m_s2=[v*am/ad*9.80665 for v in accel],
        mag_scaled=[v*mm/md for v in mag],
        # 磁気単位は今回の実機では独立検証していない。
        mag_multiplier=mm, mag_divisor=md,
        tail_flag_raw=p[62], tail_hex=p[54:].hex(),
    )


class Air:
    def __init__(self, output, log_packets=True):
        self.output = output
        self.handles = {}
        self.log = (output/'transactions.jsonl').open('w') if log_packets else None
        devices = hid.enumerate(0x3318, 0x0424)
        (output/'devices.json').write_text(json.dumps(devices, indent=2,
            default=lambda x: x.decode() if isinstance(x, bytes) else str(x)))
        if len({d['serial_number'] for d in devices}) != 1:
            self.close()
            raise RuntimeError('接続する初代Airを1台にしてください')
        try:
            for info in devices:
                if info['interface_number'] not in (3, 4, 5, 6):
                    continue
                d = hid.device()
                d.open_path(info['path'])
                # timeout=0のhid_readをブロックさせない。
                d.set_nonblocking(1)
                self.handles[info['interface_number']] = d
            if not {3,4}.issubset(self.handles):
                raise RuntimeError('IMU/MCUインターフェースがありません')
        except BaseException:
            self.close()
            raise

    def record(self, direction, iface, p):
        host_ns = time.time_ns()
        if self.log:
            self.log.write(json.dumps(dict(host_ns=host_ns, direction=direction,
                interface=iface, hex=p.hex()))+'\n')
        return host_ns

    def command(self, iface, cmd, data=b''):
        if len(data) > (56 if iface == 3 else 42):
            raise ValueError('コマンドデータが長すぎます')
        if iface not in (3, 4):
            raise ValueError('コマンドはIMU/MCUのみ')
        p = bytearray(64)
        p[0] = 0xaa if iface == 3 else 0xfd
        n = len(data)+(3 if iface == 3 else 17)
        struct.pack_into('<H', p, 5, n)
        if iface == 3:
            p[7] = cmd
            p[8:8+len(data)] = data
        else:
            struct.pack_into('<I', p, 7, 0x1337)
            struct.pack_into('<H', p, 15, cmd)
            p[22:22+len(data)] = data
        struct.pack_into('<I', p, 1, zlib.crc32(p[5:5+n]))
        self.record('tx', iface, p)
        self.handles[iface].write(bytes(p))
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            r = bytes(self.handles[iface].read(128, 100))
            if not r:
                continue
            self.record('rx', iface, r)
            if len(r) < 22 or r[0] != p[0]:
                continue
            echo = r[7] if iface == 3 else struct.unpack_from('<H',r,15)[0]
            if echo != cmd:
                continue
            size = struct.unpack_from('<H',r,5)[0]
            if size < (3 if iface==3 else 17) or size+5 > len(r):
                raise ValueError('コマンド応答長が不正')
            if zlib.crc32(r[5:5+size]) != struct.unpack_from('<I',r,1)[0]:
                raise ValueError('コマンド応答CRC不一致')
            return r[8:5+size] if iface == 3 else r[22:5+size]
        raise TimeoutError(f'interface={iface}, command=0x{cmd:02x}')

    def factory_config(self):
        n = struct.unpack('<I', self.command(3, 0x14))[0]
        if not 0 < n < 1024*1024:
            raise ValueError(f'校正データ長が不正: {n}')
        config = bytearray()
        while len(config) < n:
            part = self.command(3, 0x15)
            if not part:
                raise ValueError('校正データの空応答')
            config.extend(part)
        if len(config) != n:
            raise ValueError('校正データ長が一致しません')
        (self.output/'factory_calibration.bin').write_bytes(config)
        parsed = json.loads(config.decode().rstrip('\0'))
        (self.output/'factory_calibration.json').write_text(json.dumps(parsed,indent=2))
        return n

    def close(self):
        for d in self.handles.values():
            d.close()
        if self.log:
            self.log.close()
