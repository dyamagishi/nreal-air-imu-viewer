"""起動中ビューアの実機データ経路を検証。uv run python tools/verify_live.py"""
import csv
import io
import json
import math
from pathlib import Path
import time
import urllib.request

BASE='http://127.0.0.1:8765'
def get(path):
    with urllib.request.urlopen(BASE+path,timeout=5) as response:
        return response.read()

a=json.loads(get('/api/state')); time.sleep(1); b=json.loads(get('/api/state'))
assert a['status']['state']==b['status']['state']=='live'
assert b['status']['samples']>a['status']['samples']
assert b['sample']['device_ns']>a['sample']['device_ns']
assert 850<b['status']['rate_hz']<1150
pose=b['sample']['pose']
assert pose['algorithm']=='VQF'
assert set(pose['variants'])=={'imu','mag'}
for variant in pose['variants'].values():
    for key in ('position_world_m','velocity_world_m_s','linear_acceleration_world_m_s2','position_view_m'):
        assert len(variant['translation'][key])==3
        assert all(math.isfinite(v) for v in variant['translation'][key])
assert pose['translation']['elapsed_seconds']>0
for variant in pose['variants'].values():
    assert abs(sum(x*x for x in variant['quaternion_wxyz'])-1)<1e-9
    assert all(math.isfinite(variant[k]) for k in ('yaw','pitch','roll'))
assert 0<pose['magnetic_updates']<b['status']['samples']
with urllib.request.urlopen(BASE+'/events',timeout=5) as response:
    messages=[]
    while len(messages)<2:
        line=response.readline().decode()
        if line.startswith('data: '): messages.append(json.loads(line[6:]))
assert messages[1]['sequence']>messages[0]['sequence']
rows=list(csv.DictReader(io.StringIO(get('/api/download.csv').decode('utf-8-sig'))))
assert 10<len(rows)<=3000
assert {'vqf_imu_yaw','vqf_mag_yaw','vqf_bias_y_dps','vqf_mag_disturbance',
        'imu_position_m_x','imu_velocity_m_s_y','mag_linear_acceleration_m_s2_z','zupt_enabled'}<=set(rows[0])
assert all(math.isfinite(float(value)) for row in rows for value in row.values())
assert all(int(b['device_ns'])>int(a['device_ns']) for a,b in zip(rows,rows[1:]))
report=dict(checked_at=time.strftime('%Y-%m-%d %H:%M:%S %z'),state=b['status'],
    pose=pose,csv_rows=len(rows),sse_sequence=[m['sequence'] for m in messages],
    checks=['実機データの連続更新','取得レート','SSE配信','CSV列・有限数値・単調時刻','VQF両方式の有限姿勢・正規化','磁気更新の重複除外','並進3軸・速度・重力除去加速度・CSV'])
path=Path(__file__).resolve().parents[1]/'data/reports/live-verification.json'
path.parent.mkdir(parents=True,exist_ok=True)
path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
