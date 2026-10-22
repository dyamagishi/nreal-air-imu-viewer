"""ワールド座標の線形加速度を積分する。位置・速度の初期値は0。"""
import math

GRAVITY = 9.80665


class Translation:
    def __init__(self):
        self.reset()

    def reset(self):
        self.position = [0.,0.,0.]
        self.velocity = [0.,0.,0.]
        self.acceleration = [0.,0.,0.]
        self.previous_acceleration = None
        self.elapsed = 0.
        self.zupt_active = False

    def update(self, world_acceleration, dt, *, rest=False, zupt=False):
        # earth Y-up。比力に含まれる重力成分を、姿勢変換後に取り除く。
        a = [world_acceleration[0],world_acceleration[1]-GRAVITY,world_acceleration[2]]
        if not all(math.isfinite(v) for v in a):
            raise ValueError('並進積分への入力が有限数値ではありません')
        self.acceleration = a
        self.zupt_active = bool(rest and zupt)
        if dt is not None and 0 < dt <= .1 and self.previous_acceleration is not None:
            old_velocity = self.velocity[:]
            if self.zupt_active:
                # 明示的に有効にした場合のみ、静止判定をゼロ速度観測として使う。
                self.velocity = [0.,0.,0.]
            else:
                self.velocity = [v+.5*(previous+current)*dt
                    for v,previous,current in zip(old_velocity,self.previous_acceleration,a)]
                self.position = [p+.5*(old+new)*dt
                    for p,old,new in zip(self.position,old_velocity,self.velocity)]
            self.elapsed += dt
        elif self.zupt_active:
            self.velocity = [0.,0.,0.]
        self.previous_acceleration = a

    def output(self):
        return dict(position_world_m=self.position[:],velocity_world_m_s=self.velocity[:],
            linear_acceleration_world_m_s2=self.acceleration[:],
            elapsed_seconds=self.elapsed,zupt_active=self.zupt_active)
