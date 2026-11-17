import math
from typing import Dict, Tuple

class CollisionRiskEngine:
    """
    두 객체(지게차, 작업자)의 위치, 속도, 방향을 입력받아
    현재 거리, DCPA(최근접 거리), TTC(예상 충돌 시간), CRI(충돌 위험 지수)를 계산하는 엔진
    """
    def __init__(self, safe_distance_threshold: float = 3.0, safe_time_threshold: float = 4.0):
        """
        :param safe_distance_threshold: 위험으로 간주하는 최소 근접 거리 기준 (미터)
        :param safe_time_threshold: 위험으로 간주하는 최소 여유 시간 기준 (초)
        """
        self.d_thresh = safe_distance_threshold
        self.t_thresh = safe_time_threshold

    def calculate_risk(self, obj1: Dict, obj2: Dict) -> Dict:
        """
        두 객체의 상태 딕셔너리를 받아 위험 지표를 계산하여 반환
        """
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

        # 4. TCPA (최근접 도달 시간) 및 DCPA (최근접 거리) 계산
        if rel_speed_sq < 1e-6:
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
        # 최근접 거리(DCPA)가 안전 반경(1.5m) 이내로 들어올 때만 유효한 충돌 시간으로 인정
        collision_radius = 1.5
        if tcpa > 0 and dcpa <= collision_radius:
            ttc = tcpa
        else:
            ttc = float('inf')  # 충돌 경로가 아님 (무한대)

        # 6. CRI (Collision Risk Index, 0.0 ~ 1.0) 종합 점수 산출
        cri = self._compute_cri(current_dist, dcpa, ttc)

        # 7. 기획서 기준 위험 단계 판정
        if cri >= 0.85:
            status = "CRITICAL"  # 위험
        elif cri >= 0.50:
            status = "WARNING"   # 주의
        else:
            status = "SAFE"      # 정상

        return {
            "distance": round(current_dist, 2),
            "dcpa": round(dcpa, 2),
            "ttc": round(ttc, 2) if ttc != float('inf') else -1.0,  # -1.0은 충돌 없음 표시
            "cri": round(cri, 3),
            "status": status
        }

    def _compute_cri(self, dist: float, dcpa: float, ttc: float) -> float:
        """
        거리, DCPA, TTC를 조합하여 0.0 ~ 1.0 사이의 위험 지수(CRI) 계산
        """
        # 이미 매우 가까운 경우 즉각 최고 위험 부여
        if dist < 1.5:
            return 1.0

        # 거리 기반 위험도 (가까울수록 1에 수렴)
        dist_risk = max(0.0, min(1.0, 1.0 - (dist - 1.5) / 10.0))

        # DCPA 기반 위험도 (예상 최근접 거리가 좁을수록 위험)
        dcpa_risk = max(0.0, min(1.0, 1.0 - (dcpa / self.d_thresh)))

        # TTC 기반 위험도 (충돌까지 남은 시간이 짧을수록 위험)
        if ttc == float('inf') or ttc > self.t_thresh:
            ttc_risk = 0.0
        else:
            ttc_risk = max(0.0, min(1.0, 1.0 - (ttc / self.t_thresh)))

        # 가중 평균으로 최종 CRI 도출 (미래 예측 지표인 TTC와 DCPA에 높은 가중치 부여)
        if ttc != float('inf'):
            cri = 0.2 * dist_risk + 0.3 * dcpa_risk + 0.5 * ttc_risk
        else:
            # 충돌 경로가 아니더라도 물리적으로 너무 가까우면 주의 경보 발생
            cri = 0.6 * dist_risk + 0.4 * dcpa_risk

        return min(1.0, cri)

# 1단계 시뮬레이터와 2단계 위험 엔진 연동 테스트
if __name__ == "__main__":
    from simulator import WarehouseSimulator

    print("=== [STEP 2] 시뮬레이터 + 위험 계산 엔진 통합 테스트 ===")
    sim = WarehouseSimulator(scenario="collision")
    engine = CollisionRiskEngine()

    for _ in range(60):
        frame = sim.step()
        risk_result = engine.calculate_risk(frame["forklift"], frame["worker"])

        # 0.5초마다 위험도 판정 결과 출력
        if int(frame["timestamp"] * 10) % 5 == 0:
            ttc_str = f"{risk_result['ttc']:4.1f}s" if risk_result['ttc'] >= 0 else " 없음 "
            print(f"[{frame['timestamp']:4.1f}s] "
                  f"거리: {risk_result['distance']:5.2f}m | "
                  f"DCPA: {risk_result['dcpa']:4.2f}m | "
                  f"TTC: {ttc_str} | "
                  f"CRI: {risk_result['cri']:.3f} -> [{risk_result['status']}]")