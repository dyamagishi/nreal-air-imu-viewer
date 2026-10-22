"""姿勢推定の基準運動と、実機レポートの復号互換性を検証。"""
import json
import math
from pathlib import Path
import unittest

from nreal_air.device import decode_imu
from nreal_air.pose import Calibration, Pose, conjugate, rotate


def sample(t, gyro=(0,0,0), accel=(0,0,9.80665)):
    return dict(device_ns=t,gyro_dps=list(gyro),accel_m_s2=list(accel))


class PoseTests(unittest.TestCase):
    def test_one_second_yaw_90_degrees(self):
        pose=Pose(); pose.update(sample(0))
        for i in range(1,1001):
            result=pose.update(sample(i*1_000_000,gyro=(0,0,90)))
        self.assertAlmostEqual(result['yaw'],90,places=3)
        self.assertAlmostEqual(result['pitch'],0,delta=.001)
        pose.center()
        result=pose.update(sample(1_001_000_000))
        self.assertAlmostEqual(result['yaw'],0,places=5)

    def test_tilted_stationary_gravity_is_stable(self):
        pose=Pose(); angle=math.radians(35)
        s=sample(0,accel=(0,math.sin(angle)*9.80665,math.cos(angle)*9.80665))
        pose.update(s); original=pose.q[:]
        for i in range(1,2001):
            pose.update(dict(s,device_ns=i*1_000_000))
        # VQFの数値近似を含む回転角で評価する。実測した誤差は約0.00029度。
        dot=min(1,abs(sum(a*b for a,b in zip(original,pose.q))))
        self.assertLess(2*math.degrees(math.acos(dot)),.001)
        world=rotate(pose.q,[0,math.cos(angle),math.sin(angle)])
        self.assertLess(math.degrees(math.acos(min(1,world[1]))),.001)

    def test_large_time_gap_does_not_integrate(self):
        pose=Pose();pose.update(sample(0))
        result=pose.update(sample(10_000_000_000,gyro=(0,0,1000)))
        self.assertEqual(result['quaternion_wxyz'],[1,0,0,0])

    def test_stationary_gyro_bias_is_estimated(self):
        pose=Pose()
        for i in range(10001):
            result=pose.update(sample(i*1_000_000,gyro=(0,0,.8)))
        self.assertTrue(result['rest_detected'])
        self.assertAlmostEqual(result['bias_dps'][1],.8,delta=.02)
        # 補正なしなら8度。VQFでは静止検出後にドリフトが抑制される。
        self.assertLess(abs(result['yaw']),2)

    def test_magnetic_reports_are_not_counted_twice(self):
        pose=Pose()
        for i in range(100):
            s=sample(i*1_000_000)
            s.update(mag_raw=[100,0,100],mag_scaled=[.1,0,.1],tail_flag_raw=int(i%6==0))
            result=pose.update(s)
        self.assertEqual(result['magnetic_updates'],17)
        self.assertEqual(set(result['variants']),{'imu','mag'})
        pose.center(); result=pose.output()
        for variant in result['variants'].values():
            for angle in ('yaw','pitch','roll'):
                self.assertAlmostEqual(variant[angle],0,places=6)

    def test_sensor_alignment_matches_monado_path(self):
        # Monadoと同じxyzwクォータニオン演算で磁気XをgyroのYへ回す。
        calibration=Calibration(dict(gyro_q_mag=[0,0,math.sqrt(.5),math.sqrt(.5)]))
        s=sample(0);s['mag_scaled']=[1,0,0]
        gyro,acc,mag=calibration.vectors(s)
        for actual,expected in zip(mag,[0,0,1]):
            self.assertAlmostEqual(actual,expected,places=6)
        self.assertAlmostEqual(acc[1],9.80665)

    def test_magnetic_disturbance_is_detected_without_heading_jump(self):
        pose=Pose()
        for i in range(11001):
            theta=math.radians(30*i*.001)
            q=[math.cos(theta/2),0,math.sin(theta/2),0]
            mag=rotate(conjugate(q),[.3,.4,0])
            scale=3 if i>10000 else 1
            s=sample(i*1_000_000,gyro=(0,0,30))
            s.update(mag_scaled=[-mag[0]*scale,mag[2]*scale,mag[1]*scale],
                     tail_flag_raw=int(i%6==0))
            result=pose.update(s)
            if i==10000:
                self.assertTrue(result['magnetic_reference_ready'])
                self.assertFalse(result['magnetic_disturbance'])
                correction=result['heading_correction_deg']
        self.assertTrue(result['magnetic_disturbance'])
        self.assertAlmostEqual(result['heading_correction_deg'],correction,delta=.01)
        self.assertAlmostEqual(result['variants']['mag']['yaw'],-30,delta=.01)

    def test_duplicate_timestamp_does_not_change_pose(self):
        pose=Pose();pose.update(sample(0))
        before=pose.update(sample(1_000_000,gyro=(0,0,10)))
        after=pose.update(sample(1_000_000,gyro=(0,0,1000)))
        self.assertEqual(before,after)

    def test_measured_capture_still_decodes_identically(self):
        path=Path(__file__).parent/'fixtures/imu_report.json'
        fixture=json.loads(path.read_text())
        actual=decode_imu(bytes.fromhex(fixture['hex']))
        for key,expected in fixture['expected'].items():
            self.assertEqual(actual[key],expected)



if __name__=='__main__': unittest.main()
