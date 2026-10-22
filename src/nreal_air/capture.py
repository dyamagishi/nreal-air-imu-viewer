"""IMUの有限時間記録。"""
import argparse, csv, json, math, statistics, time
from datetime import datetime
from pathlib import Path
from .device import Air, decode_imu

def summarize(samples, elapsed, events):
    if not samples:
        raise RuntimeError('IMUデータを取得できませんでした')
    deltas = [b['device_ns']-a['device_ns'] for a,b in zip(samples,samples[1:])]
    positive = [d for d in deltas if d > 0]
    def span(values):
        return dict(min=min(values), mean=statistics.mean(values), max=max(values))
    result = dict(samples=len(samples), wall_seconds=elapsed,
        host_rate_hz=len(samples)/elapsed,
        median_device_interval_ns=statistics.median(positive) if positive else None,
        device_rate_hz=1e9/statistics.median(positive) if positive else None,
        nonmonotonic_timestamps=sum(d<=0 for d in deltas),
        gaps_over_2ms=sum(d>2_000_000 for d in deltas),
        temperature_c=span([s['temperature_c'] for s in samples]),
        acceleration_norm_m_s2=span([math.sqrt(sum(v*v for v in s['accel_m_s2'])) for s in samples]),
        first_sample=samples[0], last_sample=samples[-1], other_packets=events)
    for group in ('gyro_dps','accel_m_s2','mag_scaled'):
        result[group] = {axis:span([s[group][i] for s in samples]) for i,axis in enumerate('xyz')}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--secondsには正数を指定してください')
    output = args.output or Path('data/captures')/datetime.now().strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    air = Air(output)
    samples, events = [], {}
    started = False
    try:
        air.command(3, 0x19, b'\0')
        config_size = air.factory_config()
        metadata = dict(factory_config_bytes=config_size)
        for name,cmd in [('serial',0x15),('mcu_firmware',0x26),('brightness',0x03),('display_mode',0x07)]:
            try:
                value = air.command(4,cmd)
                metadata[name] = dict(hex=value.hex(), status=value[0] if value else None,
                    text=value[1:].decode('ascii',errors='replace'))
            except (TimeoutError, ValueError) as exc:
                metadata[name] = dict(error=str(exc))
        (output/'metadata.json').write_text(json.dumps(metadata,indent=2))
        print(json.dumps(metadata,ensure_ascii=False),flush=True)
        started = True
        air.command(3, 0x19, b'\1')
        start = time.monotonic()
        last_print = start
        with (output/'imu.jsonl').open('w') as stream:
            while time.monotonic()-start < args.seconds:
                for iface,d in air.handles.items():
                    # 各回最大128パケット。IMUの洪水で他のHIDを飢餓させない。
                    for _ in range(128):
                        p = bytes(d.read(128))
                        if not p:
                            break
                        host_ns = air.record('rx',iface,p)
                        if iface == 3 and p[:2] == b'\x01\x02':
                            s = dict(host_ns=host_ns, **decode_imu(p))
                            samples.append(s)
                            stream.write(json.dumps(s)+'\n')
                        else:
                            key = (f'{iface}:0x{struct.unpack_from("<H",p,15)[0]:04x}'
                                   if iface == 4 and len(p) == 64 and p[0] == 0xfd
                                   else f'{iface}:{len(p)}bytes')
                            events[key] = events.get(key,0)+1
                now = time.monotonic()
                if samples and now-last_print >= 1:
                    s = samples[-1]
                    print(f"n={len(samples)} gyro[dps]={s['gyro_dps']} accel[m/s²]={s['accel_m_s2']} mag={s['mag_scaled']} T={s['temperature_c']:.2f}°C",flush=True)
                    last_print = now
                time.sleep(0.0005)
        elapsed = time.monotonic()-start
        summary = summarize(samples,elapsed,events)
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        fields = ['host_ns','device_ns','temperature_c']+[f'{g}_{a}' for g in ('gyro_dps','accel_m_s2','mag_scaled') for a in 'xyz']
        with (output/'imu.csv').open('w') as f:
            writer = csv.DictWriter(f,fieldnames=fields); writer.writeheader()
            for s in samples:
                row = {k:s[k] for k in fields[:3]}
                row.update({f'{g}_{a}':s[g][i] for g in ('gyro_dps','accel_m_s2','mag_scaled') for i,a in enumerate('xyz')})
                writer.writerow(row)
        print(f"保存: {output} / {len(samples)} samples / {summary['device_rate_hz']:.1f} Hz",flush=True)
    finally:
        try:
            if started:
                air.command(3, 0x19, b'\0')
        finally:
            air.close()


if __name__ == '__main__':
    main()
