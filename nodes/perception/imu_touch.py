"""imu_touch — IMU 로 "톡톡 건드림" 감지 (순수 계산). executive 가 SLEEP·DOZE 에서 깨우기 자극으로 씀 (11-14).

  기준: docs/calibration/imu-touch-2026-11-06.md — 정지 잡음 ~0.005 g, 손가락 톡 0.04 g (8배), 기우뚱 gx ~6 °/s.
  규칙
    1. 기준값 = 조용할 때의 평균 (EMA, 천천히 따라감). 브리지 IMU 는 x·y 자이로 치우침을 안 빼므로 자이로도 기준값 빼기
    2. 수평 가속 변화 √(dx²+dy²) > ACC_G (0.02 g) 또는 |gx|·|gy| 변화 > GYRO_DPS (4 °/s) → 톡
    3. **자기가 만든 흔들림은 뺌 (efference copy)**: quiet=False (고개 명령 뒤 1 s·바퀴 움직임 중) 이면 안 셈, 기준값도 안 고침
    4. 한 번 톡 뒤 REFRACT_S 동안은 다시 안 셈
  방향: 밀린 쪽의 반대가 맞은 쪽 → dir_deg = atan2(−dy, −dx) (보드 x = 앞, y = 왼쪽 가정. 부호는 실물 톡톡으로 확인 → SIGN 으로 뒤집기)
"""
import math

G = 9.81
ACC_G = 0.02
GYRO_DPS = 4.0
REFRACT_S = 1.0
EMA = 0.05
SIGN = 1.0        # 실물에서 방향이 반대로 나오면 −1


class Touch:
    def __init__(self):
        self.base = None          # [ax, ay, gx, gy]
        self.last_t = -1e9
        self.n = 0
        self.last = None          # (세기 g, 방향°) 로그용

    def step(self, now, ax, ay, gx, gy, quiet=True):
        """ax, ay m/s², gx, gy rad/s → None 또는 (dir_deg, size_g)"""
        v = [ax, ay, gx, gy]
        if self.base is None:
            self.base = v
            return None
        if not quiet:
            return None
        dx, dy = (ax - self.base[0]) / G, (ay - self.base[1]) / G
        dgx, dgy = math.degrees(gx - self.base[2]), math.degrees(gy - self.base[3])
        acc = math.hypot(dx, dy)
        hit = acc > ACC_G or abs(dgx) > GYRO_DPS or abs(dgy) > GYRO_DPS
        if not hit:
            self.base = [b + EMA * (x - b) for b, x in zip(self.base, v)]
            self.n = min(self.n + 1, 1000)
            return None
        if self.n < 20 or now - self.last_t < REFRACT_S:          # 기준값이 아직 없거나 방금 셌음
            return None
        self.last_t = now
        d = math.degrees(math.atan2(-dy * SIGN, -dx * SIGN)) if acc > ACC_G / 2 else None
        self.last = (round(acc, 3), None if d is None else round(d))
        return (d, acc)
