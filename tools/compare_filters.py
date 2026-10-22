"""保存済みの実測を再生して処理速度を確認し、既知バイアス入力で補正を比較。"""
import importlib.util
import json
import math
from pathlib import Path
import time

from nreal_air.pose import Pose

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('old_pose',ROOT/'tools/reference/pose_before_vqf.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
samples=[json.loads(line) for line in (ROOT/'data/captures/20261021-205855/imu.jsonl').read_text().splitlines()]
factory=json.loads((ROOT/'data/captures/20261021-205855/factory_calibration.json').read_text())['IMU']['device_1']
report=dict(real_capture={},synthetic_stationary_bias={})
for label,pose in [('previous',old.Pose(factory['gyro_bias'])),('vqf',Pose(factory=factory))]:
    start=time.perf_counter()
    for s in samples:
        result=pose.update(s)
        assert all(math.isfinite(float(x)) for x in result['quaternion_wxyz'])
        assert abs(sum(x*x for x in result['quaternion_wxyz'])-1)<1e-9
    elapsed=time.perf_counter()-start
    report['real_capture'][label]=dict(samples=len(samples),processing_seconds=elapsed,
        processing_samples_per_second=len(samples)/elapsed,last_pose=result)
for label,pose in [('previous',old.Pose()),('vqf',Pose())]:
    for i in range(10001):
        result=pose.update(dict(device_ns=i*1_000_000,gyro_dps=[0,0,.8],accel_m_s2=[0,0,9.80665]))
    report['synthetic_stationary_bias'][label]=dict(seconds=10,injected_bias_dps=.8,
        final_yaw_deg=result['yaw'],estimated_bias_dps=result.get('bias_dps'))
report['note']='実測データには真値がないため姿勢精度の優劣は判定しない。合成試験のみ真値=静止。'
output=ROOT/'data/reports/vqf-comparison.json'
output.parent.mkdir(parents=True,exist_ok=True)
output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
