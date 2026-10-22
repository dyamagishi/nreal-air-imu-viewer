"""ジャイロを積分し、加速度の重力方向で傾きを補正する相対3DoF推定。"""
import math


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


class Pose:
    def __init__(self, bias=None):
        self.bias = bias or [0,0,0]
        self.q = [1,0,0,0]
        self.reference = [1,0,0,0]
        self.previous = None

    def center(self):
        self.reference = self.q[:]

    def update(self, s):
        # ar-drivers-rsと同じ表示座標系、Yが上。バイアスの符号も同実装に従う。
        gx,gy,gz = [math.radians(v) for v in s['gyro_dps']]
        gyro = [-gx-self.bias[0], gz+self.bias[1], gy+self.bias[2]]
        ax,ay,az = s['accel_m_s2']
        accel = [-ax,az,ay]
        n = math.sqrt(sum(v*v for v in accel))
        timestamp = s['device_ns']
        dt = (timestamp-self.previous)/1e9 if self.previous is not None else None
        self.previous = timestamp
        if dt is None or dt <= 0 or dt > .1:
            if n > 1:
                a = [v/n for v in accel]
                # 測定重力ベクトルaをワールド上方向[0,1,0]へ回す。
                self.q = normalize([1+a[1], -a[2], 0, a[0]]) if a[1] > -.9999 else [0,1,0,0]
            self.center()
        else:
            # 並進加速度が大きい場合は重力補正を弱める。
            if 7.5 < n < 12:
                measured = [v/n for v in accel]
                predicted = rotate(conjugate(self.q), [0,1,0])
                e = [measured[1]*predicted[2]-measured[2]*predicted[1],
                     measured[2]*predicted[0]-measured[0]*predicted[2],
                     measured[0]*predicted[1]-measured[1]*predicted[0]]
                gyro = [v+1.5*correction for v,correction in zip(gyro,e)]
            derivative = multiply(self.q,[0,*gyro])
            self.q = normalize([q+.5*d*dt for q,d in zip(self.q,derivative)])
        relative = normalize(multiply(conjugate(self.reference),self.q))
        # Y-upのyaw(Y), pitch(X), roll(Z)。YXZ順で分解。
        w,x,y,z = relative
        pitch = math.asin(max(-1,min(1,2*(w*x-y*z))))
        yaw = math.atan2(2*(w*y+x*z),1-2*(x*x+y*y))
        roll = math.atan2(2*(w*z+x*y),1-2*(x*x+z*z))
        return dict(quaternion_wxyz=relative, pitch=math.degrees(pitch),
                    yaw=math.degrees(yaw), roll=math.degrees(roll))
