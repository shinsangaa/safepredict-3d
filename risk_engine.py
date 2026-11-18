import math
from typing import Dict, Tuple

COLLISION_RADIUS_M = 1.0 # 충돌 판정 거리
SAFE_RADIUS_BASE_M = 3.0 # 정지한 지게차의 안전 버블 반경
SAFE_RADIUS_SPEED_FACTOR_S = 0.5 # 지게차 속력 1m/s당 버블 증가량
TTC_SCORE_START_S = 6.0 # TTC 점수가 0에서 오르기 시작하는 시간
TTC_SCORE_FULL_S = 1.0 # TTC 점수가 1이 되는 시간
WARNING_THRESHOLD = 0.50
CRITICAL_THRESHOLD = 0.85
EPSILON = 1e-6 # 사실상 0을 판단하는 허용 오차

def classify_level(cri: float) -> str:
    if cri >= CRITICAL_THRESHOLD:
        return "CRITICAL"
    elif cri >= WARNING_THRESHOLD:
        return "WARNING"
    else:
        return "SAFE"

class CollisionRiskEngine:
    """
    두 객체(지게차, 작업자)의 위치, 속도, 방향을 입력받아
    현재 거리, DCPA(최근접 거리), TTC(예상 충돌 시간), CRI(충돌 위험 지수)를 계산하는 엔진
    """

    def calculate_risk(self, obj1: Dict, obj2: Dict) -> Dict:
        """
        두 객체의 상태 딕셔너리를 받아 위험 지표를 계산하여 반환
        """
        for name, obj in (("forklift", obj1), ("worker", obj2)):
            for key in ("x", "y", "speed", "heading"):
                value = obj.get(key)
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"{name}의 '{key}' 값이 올바르지 않습니다: {value}")
            if obj["speed"] < 0:
                raise ValueError(f"{name}의 속력이 음수입니다: {obj['speed']}")
        # 1. 상대 위치 벡터 (dP = P2 - P1)
        dx = obj2["x"] - obj1["x"]
        dy = obj2["y"] - obj1["y"]
        current_dist = math.hypot(dx, dy)  # 현재 두 객체 사이의 직선 거리 (m)

        # 2. 각 객체의 속도 벡터 (vx, vy)
        v1x = obj1["speed"] * math.cos(obj1["heading"])
        v1y = obj1["speed"] * math.sin(obj1["heading"])
        v2x = obj2["speed"] * math.cos(obj2["heading"])
        v2y = obj2["speed"] * math.sin(obj2["heading"])

        # 3. 상대 속도 벡터 (dV = V2 - V1)
        dvx = v2x - v1x
        dvy = v2y - v1y
        rel_speed_sq = dvx**2 + dvy**2  # 상대 속력의 제곱

        relative_speed = math.sqrt(rel_speed_sq)
        if current_dist <= EPSILON:
            closing_speed = 0.0 # 중심이 겹치면 아래 나눗셈의 분모가 0이 됨
        else:
            closing_speed = -(dx*dvx + dy*dvy) / current_dist

        # 4. TCPA (최근접 도달 시간) 및 DCPA (최근접 거리) 계산
        if relative_speed <= EPSILON:
            # 두 객체가 같은 속도와 방향으로 나란히 이동 중인 경우
            tcpa = 0.0
            dcpa = current_dist
        else:
            # 내적(dot product)을 이용해 가장 가까워지는 시간(TCPA) 산출
            # 음수면 이미 서로 멀어지고 있는 상태
            tcpa = -(dx * dvx + dy * dvy) / rel_speed_sq
            
            if tcpa < 0:
                # 이미 스쳐 지나갔거나 멀어지는 중이면 현재 거리가 최소 거리
                tcpa = 0.0
                dcpa = current_dist
            else:
                # tcpa 초 후의 예상 위치 차이 계산 -> DCPA
                future_dx = dx + dvx * tcpa
                future_dy = dy + dvy * tcpa
                dcpa = math.hypot(future_dx, future_dy)

        # 5. TTC (Time-To-Collision, 예상 충돌 시간) 정의
        # |r + vt|=1m 가 되는 t를 구하는 2차 방정식 a*t² + b*t + c = 0
        a = rel_speed_sq
        b = 2 * (dx * dvx + dy * dvy)
        c = current_dist**2 - COLLISION_RADIUS_M**2
        D = b**2 - 4*a*c

        if c <= 0:
            ttc = 0.0 # 이미 1m 구역 안 (이 검사가 반드시 첫 번째)
        elif relative_speed <= EPSILON: # 구역 밖인데 상대 이동이 없음
            ttc = None
        elif D < 0:
            ttc = None # 1m 원과 만나지 않고 지나감
        else:
            t1 = (-b - math.sqrt(D)) / (2 * a) # 원에 들어가는 시각
            t2 = (-b + math.sqrt(D)) / (2 * a) # 원에서 나오는 시각
            future_roots = [t for t in (t1, t2) if t >= 0]
            ttc = min(future_roots) if future_roots else None


        # 6. CRI (Collision Risk Index, 0.0 ~ 1.0) 종합 점수 산출
        safe_radius = SAFE_RADIUS_BASE_M + SAFE_RADIUS_SPEED_FACTOR_S * obj1["speed"] # obj1=지게차
        cri = self._compute_cri(current_dist, ttc, safe_radius)

        # 7. 기획서 기준 위험 단계 판정
        status = classify_level(cri)

        return {
            "distance": current_dist,
            "relative_speed": relative_speed,
            "closing_speed": closing_speed,
            "t_cpa": tcpa,
            "dcpa": dcpa,
            "ttc": ttc,         # 충돌 예상 없으면 None
            "collision_predicted": ttc is not None,
            "safe_radius": safe_radius,
            "cri": cri,
            "status": status
        }

    def _compute_cri(self, dist: float, ttc, safe_radius: float) -> float:
        # TTC 점수: 6초 -> 0점, 1초 -> 1점
        if ttc is None:
            s_ttc = 0.0
        else:
            s_ttc = max(0.0, min(1.0, (TTC_SCORE_START_S - ttc) / (TTC_SCORE_START_S - TTC_SCORE_FULL_S)))

        # 거리 점수: 버블 경계 -> 0점, 1m -> 1점
        s_distance = max(0.0, min(1.0, (safe_radius - dist) / (safe_radius - COLLISION_RADIUS_M)))

        # 둘 중 하나라도 위험하면 위험 -> 평균이 아니라 큰 값
        return max(s_ttc, s_distance)

# 1단계 시뮬레이터와 2단계 위험 엔진 연동 테스트
if __name__ == "__main__":
    from simulator import WarehouseSimulator

    print("=== [STEP 2] 시뮬레이터 + 위험 계산 엔진 통합 테스트 ===")
    sim = WarehouseSimulator(scenario="crossing")
    engine = CollisionRiskEngine()

    for _ in range(60):
        frame = sim.step()
        risk_result = engine.calculate_risk(frame["forklift"], frame["worker"])

        # 0.5초마다 위험도 판정 결과 출력
        if int(frame["timestamp"] * 10) % 5 == 0:
            ttc_str = f"{risk_result['ttc']:4.1f}s" if risk_result['ttc'] is not None else " 없음 "
            print(f"[{frame['timestamp']:4.1f}s] "
                  f"거리: {risk_result['distance']:5.2f}m | "
                  f"DCPA: {risk_result['dcpa']:4.2f}m | "
                  f"TTC: {ttc_str} | "
                  f"CRI: {risk_result['cri']:.3f} -> [{risk_result['status']}]")