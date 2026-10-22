"""VQF公式C++実装による回転推定。磁気なし/ありを同時に更新する。"""
import math

import numpy as np
from vqf import VQF

from .translation import Translation


def multiply(a, b):
    w,x,y,z = a; v,i,j,k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]


def normalize(q):
    length = math.sqrt(sum(x*x for x in q))
    return [x/length for x in q] if length > 1e-12 else [1,0,0,0]


def rotate(q, v):
    return multiply(multiply(q, [0,*v]), conjugate(q))[1:]


def factory_quat(values):
    """Monadoのread_json_quatと同じxyzw→wxyz。正規化して使用する。"""
    x,y,z,w = values
    return normalize([w,x,y,z])


class Calibration:
    """MonadoのAir校正経路を再現し、全センサーを表示用の共通body軸にそろえる。

    工場JSONのJPLという注記から独自に反転せず、実ドライバーと同じ演算を用いる。
    参照: docs/device-analysis.md のMonado校正経路。
    """
    def __init__(self, factory=None):
        self.factory = factory or {}
        self.accel_q_gyro = factory_quat(self.factory.get('accel_q_gyro',[0,0,0,1]))
        self.accel_q_mag = multiply(self.accel_q_gyro,
            factory_quat(self.factory.get('gyro_q_mag',[0,0,0,1])))

    def correct(self, vector, sensor):
        x,y,z = vector
        # 校正値はpre-biased座標系。バイアスを引き、スケールを適用してから表示軸へ。
        pre = [-x,-z,-y]
        bias = self.factory.get(sensor+'_bias',[0,0,0])
        scale = self.factory.get('scale_'+sensor,[1,1,1])
        v = [(a-b)*c for a,b,c in zip(pre,bias,scale)]
        return np.array([v[0],-v[1],-v[2]],dtype=np.float64)

    def vectors(self, sample):
        gyro = rotate(self.accel_q_gyro,[math.radians(v) for v in sample['gyro_dps']])
        mag = rotate(self.accel_q_mag,sample.get('mag_scaled',[0,0,0]))
        return (self.correct(gyro,'gyro'), self.correct(sample['accel_m_s2'],'accel'),
                self.correct(mag,'mag'))


# VQFはearth Z-up。ビューアのearth Y-upへ、左から-90度のX回転を掛ける。
EARTH_TO_VIEW = [math.sqrt(.5),-math.sqrt(.5),0,0]


def angles(q):
    w,x,y,z = q
    pitch = math.asin(max(-1,min(1,2*(w*x-y*z))))
    yaw = math.atan2(2*(w*y+x*z),1-2*(x*x+y*y))
    roll = math.atan2(2*(w*z+x*y),1-2*(x*x+z*z))
    return dict(quaternion_wxyz=q,pitch=math.degrees(pitch),
                yaw=math.degrees(yaw),roll=math.degrees(roll))


class Pose:
    def __init__(self, bias=None, *, factory=None, sample_period=.001, mag_period=.0061):
        # bias引数は以前の呼び出しと互換。factory指定時は工場校正一式を使用する。
        if factory is None and bias is not None:
            factory = dict(gyro_bias=bias)
        self.calibration = Calibration(factory)
        self.sample_period = sample_period
        self.mag_period = mag_period  # 初回実測の更新間隔の平均は約6.07ms
        self.filter = VQF(sample_period,magTs=mag_period)
        self.translation = {mode:Translation() for mode in ('imu','mag')}
        self.translation_generation = 0
        self.reference_generation = 0
        self.zupt_enabled = False
        self.previous = None
        self.reference = {'imu':[1,0,0,0],'mag':[1,0,0,0]}
        self.absolute = self.reference.copy()
        self.last_mag = None
        self.mag_updates = 0
        self.resets = 0
        self.q = [1,0,0,0]  # 表示earth座標での磁気なし絶対姿勢

    def reset_translation(self):
        for integrator in self.translation.values():
            integrator.reset()
        self.translation_generation += 1

    def center(self):
        self.reference_generation += 1
        self.reference = {key:q[:] for key,q in self.absolute.items()}

    def update(self, sample):
        gyr,acc,mag = self.calibration.vectors(sample)
        if not all(np.isfinite(v).all() for v in (gyr,acc,mag)):
            raise ValueError('姿勢推定への入力が有限数値ではありません')
        timestamp = sample['device_ns']
        dt = (timestamp-self.previous)/1e9 if self.previous is not None else None
        if dt is not None and dt <= 0:
            # 重複・逆順パケットを積分しない。基準時刻も巻き戻さない。
            return self.output()
        first = dt is None
        if dt is not None and dt > .1:
            self.filter.resetState(); self.last_mag=None; self.mag_updates=0
            self.resets += 1; first=True
            self.reset_translation()
        self.previous = timestamp
        # ジャイロの実測経過時間を保ち、短い欠落はnominal periodで分割する。
        # 最初のサンプルは過去の区間がないため積分しない。
        if not first:
            steps = max(1,round(dt/self.sample_period))
            for _ in range(steps):
                self.filter.updateGyr(gyr*(dt/(steps*self.sample_period)))
                self.filter.updateAcc(acc)
        else:
            self.filter.updateAcc(acc)
        raw = tuple(sample.get('mag_raw',sample.get('mag_scaled',[0,0,0])))
        flag = sample.get('tail_flag_raw',sample.get('mag_freshness_raw'))
        # 実機で値の変化と一致を確認したフラグ。古い記録では変化検出にfallback。
        fresh = flag==1 if flag is not None else raw!=self.last_mag
        first_mag = self.mag_updates==0 and fresh and np.linalg.norm(mag)>1e-12
        if fresh and np.linalg.norm(mag)>1e-12:
            self.filter.updateMag(mag)
            self.last_mag=raw; self.mag_updates += 1
        for key,getter in [('imu',self.filter.getQuat6D),('mag',self.filter.getQuat9D)]:
            self.absolute[key] = normalize(multiply(EARTH_TO_VIEW,getter()))
        self.q = self.absolute['imu']
        if first_mag:
            self.reference['mag']=self.absolute['mag'][:]
            self.translation['mag'].reset()
        if first:
            self.center()
        for key,integrator in self.translation.items():
            integrator.update(rotate(self.absolute[key],acc),None if first else dt,
                rest=self.filter.getRestDetected(),zupt=self.zupt_enabled)
        return self.output()

    def output(self):
        variants = {key:angles(normalize(multiply(conjugate(self.reference[key]),q)))
                    for key,q in self.absolute.items()}
        for key,integrator in self.translation.items():
            translation=integrator.output()
            translation['position_view_m']=rotate(conjugate(self.reference[key]),integrator.position)
            variants[key]['translation']=translation
        bias,sigma = self.filter.getBiasEstimate()
        return dict(**variants['imu'], variants=variants, algorithm='VQF',
            translation_generation=self.translation_generation,
            reference_generation=self.reference_generation,zupt_enabled=self.zupt_enabled,
            rest_detected=bool(self.filter.getRestDetected()),
            magnetic_disturbance=bool(self.filter.getMagDistDetected()),
            magnetic_reference_ready=self.filter.getMagRefNorm()>0,
            magnetic_updates=self.mag_updates,
            bias_dps=np.degrees(bias).tolist(),bias_sigma_dps=math.degrees(sigma),
            heading_correction_deg=math.degrees(self.filter.getDelta()),resets=self.resets)
