# 등속도와 월드 모델 평가 스크립트
import math
from simulator import WarehouseSimulator
from world_model import IndustrialWorldEngine

# ── 평가 설정 ─────────────────────────────────────────────
SCENARIO = "corner_turn" # S06
TOTAL_STEPS = 60 # 6.0초 (S06 최대 실행시간)
HORIZON = 25 # 예측 구간 25스텝 = 2.5초
DT = 0.1
TURN_START, TURN_END = 1.8, 3.3 # 지게차 회전 구간(초)
EPSILON = 1e-6

# 정답 궤적 만들기
def collect_frames(scenario: str, total_steps: int) -> list:
    """시뮬레이터를 끝까지 돌려 정답 궤적(프레임 목록)을 만든다."""
    sim = WarehouseSimulator(scenario=scenario)
    return [sim.step() for _ in range(total_steps)]

# 등속도(CV) 예측
def predict_cv(obj: dict, horizon: int, dt: float) -> list:
    """등속도(CV) 예측: 현재 속력·방향을 그대로 유지한다고 가정."""
    vx = obj["speed"] * math.cos(obj["heading"])
    vy = obj["speed"] * math.sin(obj["heading"])
    return [{"x": obj["x"] + vx*dt*k, "y": obj["y"] + vy*dt*k}
            for k in range(1, horizon + 1)]

# ADE · FDE 계산
def ade_fde(pred: list, truth: list) -> tuple:
    """예측 궤적과 정답 궤적의 스텝별 위치 오차로 ADE, FDE를 계산한다."""
    errors = [math.hypot(p["x"] - t["x"], p["y"] - t["y"])
              for p, t in zip(pred, truth)]
    return sum(errors) / len(errors), errors[-1]

# 시점마다 두 예측 비교
def evaluate(engine, frames: list, entity: str = "forklift") -> list:
    """예측 시작 시점마다 CV와 월드 모델의 ADE·FDE를 구한다."""
    rows = []
    # i-1 프레임(행동 추정용)과 i+HORIZON 프레임(정답)이 모두 있어야 한다.
    for i in range(1, len(frames) - HORIZON):
        prev_obj = frames[i - 1][entity]
        curr_obj = frames[i][entity]
        truth = [frames[i + k][entity] for k in range(1, HORIZON + 1)]

        cv_traj = predict_cv(curr_obj, HORIZON, DT)

        action = engine.estimate_action(prev_obj, curr_obj)
        wm_traj = engine.imagine_rollout(curr_obj, action, horizon_steps=HORIZON)

        cv_ade, cv_fde = ade_fde(cv_traj, truth)
        wm_ade, wm_fde = ade_fde(wm_traj, truth)

        t = frames[i]["timestamp"]
        rows.append({
            "t": t,
            "in_turn": TURN_START - EPSILON <= t <= TURN_END + EPSILON,
            "cv_ade": cv_ade, "cv_fde": cv_fde,
            "wm_ade": wm_ade, "wm_fde": wm_fde,
        })
    return rows

# 구간 평균 · 개선율
def summarize(rows: list) -> dict:
    """구간 평균과 개선율(%)을 계산한다."""
    n = len(rows)
    mean = {k: sum(r[k] for r in rows) / n
            for k in ("cv_ade", "cv_fde", "wm_ade", "wm_fde")}
    mean["n"] = n
    mean["ade_gain"] = (mean["cv_ade"] - mean["wm_ade"]) / mean["cv_ade"] * 100
    mean["fde_gain"] = (mean["cv_fde"] - mean["wm_fde"]) / mean["cv_fde"] * 100
    return mean

# 실행부
if __name__ == "__main__":
    engine = IndustrialWorldEngine()
    engine.train_quick_dynamics()

    frames = collect_frames(SCENARIO, TOTAL_STEPS)
    rows = evaluate(engine, frames, entity="forklift")

    sections = {
        "전체": rows,
        "회전 구간": [r for r in rows if r["in_turn"]],
        "비회전 구간": [r for r in rows if not r["in_turn"]],
    }

    print("| 구간 | 표본 수 | CV ADE | WM ADE | ADE 개선율 | CV FDE | WM FDE | FDE 개선율 |")
    print("|---|---|---|---|---|---|---|---|")
    for name, sec in sections.items():
        s = summarize(sec)
        print(f"| {name} | {s['n']} | {s['cv_ade']:.3f} | {s['wm_ade']:.3f} | {s['ade_gain']:+.1f}% "
              f"| {s['cv_fde']:.3f} | {s['wm_fde']:.3f} | {s['fde_gain']:+.1f}% |")