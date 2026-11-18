import math
import random
from typing import List, Dict, Tuple
import torch
import torch.nn as nn
import torch.optim as optim

TRAIN_EPOCHS = 300 # 학습 반복 횟수 (app.py, eval_prediction.py가 함께 사용)

class DynamicsWorldModel(nn.Module):
    """
    [산업현장 특화 경량 월드 모델]
    현재 상태 s_t = [x, y, vx, vy] 와 행동/의도 a_t = [accel, yaw_rate] 를 입력받아
    다음 스텝(dt=0.1s)의 상태 변화량(delta_s)을 예측하는 상태 전이 신경망
    """
    def __init__(self, state_dim: int = 4, action_dim: int = 2, hidden_dim: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, state_dim)  # [dx, dy, dvx, dvy] 잔차(Residual) 예측
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """
        state: (batch, 4) -> [x, y, vx, vy]
        action: (batch, 2) -> [accel (가속도), yaw_rate (회전각속도)]
        return: next_state (batch, 4)
        """
        x = torch.cat([state, action], dim=-1)
        delta_state = self.net(x)
        return state + delta_state  # Residual Connection으로 물리적 연속성 보장


class IndustrialWorldEngine:
    """
    월드 모델 학습, 상상 롤아웃(Imagination Rollout) 및 미래 충돌 위험 선제 예측기
    """
    def __init__(self, dt: float = 0.1):
        self.dt = dt
        self.device = torch.device("cpu")  # Streamlit Cloud 등 경량 배포 호환을 위해 CPU 사용
        torch.manual_seed(42) # 신경망 초기 가중치를 매번 같게 고정 (모델 생성 전에 호출해야 함)
        self.model = DynamicsWorldModel().to(self.device)
        self.is_trained = False

    def train_quick_dynamics(self, epochs: int = TRAIN_EPOCHS):
        """
        물리적 운동 방정식(직진, 가감속, 코너 선회) 기반 가상 전이 데이터를 생성하여
        월드 모델이 공간 동역학(State Transition)을 빠르게 습득하도록 학습
        """
        torch.manual_seed(42) # 랜덤 값을 매번 같게 고정
        random.seed(42)

        num_samples = 4000
        states = []
        actions = []
        next_states = []

        for _ in range(num_samples):
            x = random.uniform(-5.0, 30.0)
            y = random.uniform(-5.0, 30.0)
            speed = random.uniform(0.0, 4.5)
            heading = random.uniform(-math.pi, math.pi)
            
            vx = speed * math.cos(heading)
            vy = speed * math.sin(heading)

            # 행동(Action): 가속도(-2~2 m/s^2), 회전각속도(-1.5~1.5 rad/s)
            accel = random.uniform(-1.5, 1.5)
            yaw_rate = random.uniform(-1.2, 1.2)

            # 실제 물리 법칙에 따른 다음 상태(0.1초 후) 계산
            next_speed = max(0.0, min(5.0, speed + accel * self.dt))
            next_heading = heading + yaw_rate * self.dt
            next_vx = next_speed * math.cos(next_heading)
            next_vy = next_speed * math.sin(next_heading)
            next_x = x + ((vx + next_vx) / 2.0) * self.dt
            next_y = y + ((vy + next_vy) / 2.0) * self.dt

            states.append([x, y, vx, vy])
            actions.append([accel, yaw_rate])
            next_states.append([next_x, next_y, next_vx, next_vy])

        s_tensor = torch.tensor(states, dtype=torch.float32, device=self.device)
        a_tensor = torch.tensor(actions, dtype=torch.float32, device=self.device)
        ns_tensor = torch.tensor(next_states, dtype=torch.float32, device=self.device)

        optimizer = optim.Adam(self.model.parameters(), lr=0.005)
        criterion = nn.MSELoss()

        self.model.train()
        for _ in range(epochs):
            optimizer.zero_grad()
            pred_ns = self.model(s_tensor, a_tensor) # 4,000개 전부 예측
            loss = criterion(pred_ns, ns_tensor) # 정답과 얼마나 다른지 계산
            loss.backward()
            optimizer.step() # 덜 틀리도록 가중치를 조금 고침

        self.model.eval()
        self.is_trained = True

    def estimate_action(self, prev_obj: Dict, curr_obj: Dict) -> Tuple[float, float]:
        """
        직전 프레임과 현재 프레임을 비교해 객체의 현재 제어 의도(가속도, 조향각속도) 추정
        """
        if prev_obj is None:
            return 0.0, 0.0
        accel = (curr_obj["speed"] - prev_obj["speed"]) / self.dt
        # 각도 차이 정규화 (-pi ~ pi)
        d_heading = (curr_obj["heading"] - prev_obj["heading"] + math.pi) % (2 * math.pi) - math.pi
        yaw_rate = d_heading / self.dt
        return accel, yaw_rate

    @torch.no_grad()
    def imagine_rollout(self, obj: Dict, action: Tuple[float, float], horizon_steps: int = 25) -> List[Dict]:
        """
        [상상 롤아웃 (Imagination Rollout)]
        현재 상태(s_0)와 추정 행동(a_0)을 바탕으로 미래 N스텝(기본 25스텝 = 2.5초)의 궤적을 자가 회귀 예측
        """
        if not self.is_trained:
            self.train_quick_dynamics()

        vx = obj["speed"] * math.cos(obj["heading"])
        vy = obj["speed"] * math.sin(obj["heading"])
        curr_s = torch.tensor([[obj["x"], obj["y"], vx, vy]], dtype=torch.float32, device=self.device)
        
        # 회전 동작은 일정 시간 후 감쇠한다고 가정(자연스러운 코너 탈출 반영)
        accel, yaw_rate = action
        trajectory = []

        for step in range(1, horizon_steps + 1):
            step_yaw = yaw_rate * max(0.0, 1.0 - (step / 20.0))
            curr_a = torch.tensor([[accel * 0.5, step_yaw]], dtype=torch.float32, device=self.device)
            curr_s = self.model(curr_s, curr_a)
            
            px, py, pvx, pvy = curr_s[0].tolist()
            p_speed = math.hypot(pvx, pvy)
            p_heading = math.atan2(pvy, pvx)
            
            trajectory.append({
                "step": step,
                "time_offset": round(step * self.dt, 2),
                "x": round(px, 3),
                "y": round(py, 3),
                "speed": round(p_speed, 3),
                "heading": round(p_heading, 3)
            })
        return trajectory

    def evaluate_future_risk(self, fl_traj: List[Dict], wk_traj: List[Dict], risk_engine) -> Dict:
        """
        월드 모델이 예측한 두 객체의 미래 궤적을 시간대별로 비교하여
        가장 위험한 미래 시점의 CRI(Predicted CRI) 산출
        """
        max_pred_cri = 0.0
        critical_offset = -1.0
        min_future_dist = float('inf')

        for fl_future, wk_future in zip(fl_traj, wk_traj):
            res = risk_engine.calculate_risk(fl_future, wk_future)
            if res["distance"] < min_future_dist:
                min_future_dist = res["distance"]
            if res["cri"] > max_pred_cri:
                max_pred_cri = res["cri"]
                critical_offset = fl_future["time_offset"]

        return {
            "max_pred_cri": round(max_pred_cri, 3),
            "critical_offset": critical_offset,
            "min_future_dist": round(min_future_dist, 2)
        }


# STEP 3 단독 및 통합 테스트 실행
if __name__ == "__main__":
    from simulator import WarehouseSimulator
    from risk_engine import CollisionRiskEngine

    print("=== [STEP 3] 월드 모델 학습 및 코너 급회전 사각지대 선제 예측 테스트 ===")
    world_engine = IndustrialWorldEngine()
    print("1. 월드 모델 상태 전이 신경망(Dynamics Model) 학습 중...")
    world_engine.train_quick_dynamics()
    print("   -> 학습 완료!\n")

    sim = WarehouseSimulator(scenario="corner_turn")
    risk_engine = CollisionRiskEngine()

    prev_fl, prev_wk = None, None

    for _ in range(45):
        frame = sim.step()
        fl = frame["forklift"]
        wk = frame["worker"]

        # 현재 시점 물리 기반 CRI
        curr_risk = risk_engine.calculate_risk(fl, wk)

        # 행동(조향/가속) 추정 및 월드 모델 미래 2.5초 롤아웃
        fl_action = world_engine.estimate_action(prev_fl, fl)
        wk_action = world_engine.estimate_action(prev_wk, wk)

        fl_future_traj = world_engine.imagine_rollout(fl, fl_action, horizon_steps=25)
        wk_future_traj = world_engine.imagine_rollout(wk, wk_action, horizon_steps=25)

        # 미래 궤적 기반 위험도 평가
        future_risk = world_engine.evaluate_future_risk(fl_future_traj, wk_future_traj, risk_engine)

        # 통합 CRI 계산: 현재 위험도와 월드 모델 예측 위험도 결합
        total_cri = max(curr_risk["cri"], 0.9 * future_risk["max_pred_cri"])
        status = "CRITICAL" if total_cri >= 0.85 else ("WARNING" if total_cri >= 0.50 else "SAFE")

        if int(frame["timestamp"] * 10) % 5 == 0:
            print(f"[{frame['timestamp']:4.1f}s] "
                  f"현재거리: {curr_risk['distance']:5.2f}m | "
                  f"현재CRI: {curr_risk['cri']:.3f} | "
                  f"월드모델 예측CRI(2.5초내): {future_risk['max_pred_cri']:.3f} "
                  f"-> 통합판정: [{status}]")

        prev_fl, prev_wk = fl.copy(), wk.copy()