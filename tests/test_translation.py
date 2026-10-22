"""並進の積分を既知運動・重力・ゼロ速度補正で検証する。"""
import math
import unittest

from nreal_air.pose import Pose
from nreal_air.translation import GRAVITY, Translation


class TranslationTests(unittest.TestCase):
    def test_constant_acceleration_matches_kinematics(self):
        t=Translation();t.update([1,GRAVITY,0],None)
        for _ in range(2000):t.update([1,GRAVITY,0],.001)
        self.assertAlmostEqual(t.velocity[0],2,places=9)
        self.assertAlmostEqual(t.position[0],2,places=9)
        self.assertAlmostEqual(t.position[1],0,places=9)

    def test_constant_velocity_is_not_removed_when_zupt_off(self):
        t=Translation();t.velocity=[2,0,0];t.update([0,GRAVITY,0],None)
        for _ in range(1000):t.update([0,GRAVITY,0],.001,rest=True)
        self.assertAlmostEqual(t.position[0],2,places=9)
        self.assertEqual(t.velocity,[2,0,0])

    def test_zupt_only_when_enabled_and_rest_detected(self):
        t=Translation();t.velocity=[2,0,0];t.update([0,GRAVITY,0],None)
        t.update([0,GRAVITY,0],.001,rest=False,zupt=True)
        self.assertEqual(t.velocity,[2,0,0])
        t.update([0,GRAVITY,0],.001,rest=True,zupt=True)
        self.assertEqual(t.velocity,[0,0,0]);self.assertTrue(t.zupt_active)

    def test_full_pose_path_integrates_vertical_motion(self):
        pose=Pose()
        for i in range(2001):
            result=pose.update(dict(device_ns=i*1_000_000,gyro_dps=[0,0,0],accel_m_s2=[0,0,GRAVITY+1]))
        t=result['translation']
        self.assertAlmostEqual(t['velocity_world_m_s'][1],2,delta=.001)
        self.assertAlmostEqual(t['position_world_m'][1],2,delta=.001)
        self.assertFalse(result['zupt_enabled'])
        pose.center()
        self.assertEqual(pose.output()['translation']['position_world_m'],t['position_world_m'])
        pose.reset_translation()
        self.assertEqual(pose.output()['translation']['position_world_m'],[0,0,0])
        self.assertEqual(pose.output()['translation']['velocity_world_m_s'],[0,0,0])

    def test_tilted_gravity_does_not_become_translation(self):
        pose=Pose();angle=math.radians(35)
        for i in range(2001):
            r=pose.update(dict(device_ns=i*1_000_000,gyro_dps=[0,0,0],
                accel_m_s2=[0,math.sin(angle)*GRAVITY,math.cos(angle)*GRAVITY]))
        self.assertLess(max(abs(x) for x in r['translation']['position_world_m']),.001)
        self.assertLess(max(abs(x) for x in r['translation']['linear_acceleration_world_m_s2']),.001)

    def test_time_gap_resets_velocity_and_position(self):
        pose=Pose()
        pose.update(dict(device_ns=0,gyro_dps=[0,0,0],accel_m_s2=[0,0,GRAVITY+1]))
        pose.update(dict(device_ns=1_000_000,gyro_dps=[0,0,0],accel_m_s2=[0,0,GRAVITY+1]))
        r=pose.update(dict(device_ns=1_000_000_000,gyro_dps=[0,0,0],accel_m_s2=[0,0,GRAVITY]))
        self.assertEqual(r['translation']['position_world_m'],[0,0,0])
        self.assertEqual(r['translation']['velocity_world_m_s'],[0,0,0])
        self.assertEqual(r['translation_generation'],1)


if __name__=='__main__':unittest.main()
