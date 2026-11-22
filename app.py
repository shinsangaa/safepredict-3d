# 실행 명령어 : python -m streamlit run app.py
import os
import subprocess
import sys
import time
import math
from datetime import datetime, timezone, timedelta
import json
import pandas as pd
import streamlit as st
import streamlit.components.v1 as components

from simulator import WarehouseSimulator, SCENARIO_MAX_TIME, SCENARIO_STOP_DISTANCE
from risk_engine import CollisionRiskEngine, EPSILON, classify_level
from world_model import IndustrialWorldEngine
from stream_reader import StreamReader

# 1. 페이지 기본 설정 (넓은 관제 화면 레이아웃)
st.set_page_config(
    page_title="지게차·작업자 충돌 위험 관제",
    layout="wide"
)

# 2. 월드 모델 초기화 및 캐싱 (최초 1회만 학습하여 빠른 반응속도 유지)
@st.cache_resource
def load_world_model():
    engine = IndustrialWorldEngine(dt=0.1)
    engine.train_quick_dynamics()
    return engine

world_engine = load_world_model()
risk_engine = CollisionRiskEngine()
RULE_VERSION = "v0.3" # risk_rules.md의 규칙 버전
SCENARIO_IDS = {
    "parallel": "S01", "stop_approach": "S02", "head_on": "S03",
    "crossing": "S04", "retreat": "S05", "corner_turn": "S06",
    "multi_worker": "S07",
}

# 시나리오별 랙 배치 (화면 표시용, 충돌 계산에는 반영하지 않음)
# 각 항목: [중심 x, 중심 y, x방향 길이, y방향 길이] (m, 시뮬레이션 좌표)
SCENARIO_RACKS = {
    "parallel":      [[10.0, 4.75, 16.0, 2.5], [10.0, -3.5, 16.0, 3.0]],
    "stop_approach": [[5.0, 5.0, 12.0, 3.0], [5.0, -3.5, 12.0, 3.0]],
    "head_on":       [[14.0, 5.0, 30.0, 3.0], [14.0, -3.5, 30.0, 3.0]],
    "crossing":      [[9.75, 6.25, 17.5, 5.5], [24.5, 6.25, 6.0, 5.5]],
    "retreat":       [[5.0, 5.0, 12.0, 3.0], [5.0, -3.5, 12.0, 3.0]],
    "corner_turn":   [[9.0, 8.5, 4.5, 4.5]],
    "multi_worker":  [[10.5, 4.5, 17.0, 3.0], [27.0, 4.5, 6.0, 3.0], [13.0, -3.5, 26.0, 3.0]],
}

# 시나리오별 화면 중심과 카메라 거리 (화면 표시용)
# 각 항목: [중심 x, 중심 y, 카메라 수평 거리] (m, 시뮬레이션 좌표)
SCENARIO_VIEW = {
    "parallel":      [10.0, 3.0, 16.0],
    "stop_approach": [5.0, 0.0, 12.0],
    "head_on":       [14.0, 0.0, 16.0],
    "crossing":      [13.0, 4.0, 18.0],
    "retreat":       [5.0, 0.0, 12.0],
    "corner_turn":   [10.0, 8.0, 18.0],
    "multi_worker":  [13.0, 2.0, 20.0],
}

# 위험 단계별 권장 조치 (화면 표시용)
RECOMMENDED_ACTIONS = {
    "WARNING": "지게차 서행, 작업자에게 접근 알림",
    "CRITICAL": "지게차 즉시 정지, 작업자 대피 알림",
}

# 3. 세션 상태(Session State) 초기화
if "sim" not in st.session_state:
    st.session_state.scenario = "parallel"
    st.session_state.sim = WarehouseSimulator(scenario="parallel")
    st.session_state.prev_fl = None
    st.session_state.prev_wks = {} # 작업자별 직전 상태 (작업자 ID -> 상태)
    st.session_state.event_logs = []
    st.session_state.is_running = False
    st.session_state.is_finished = False # 시나리오 종료 여부
    st.session_state.prev_levels = {} # 작업자별 직전 위험 단계 (없으면 아직 판정 전)
    st.session_state.event_seq = 0 # 이번 실행에서 발급한 이벤트 번호
    st.session_state.run_seq = 1 # 실행 번호 (run_001, run_002, ...)
    st.session_state.run_start = datetime.now(timezone.utc) # 실행 시작 UTC 시각
    st.session_state.risk_results = [] # 틱별 계산 결과 (data_spec.md 4절)
    st.session_state.input_mode = "내장 시뮬레이터" # 데이터 입력 방식
    st.session_state.reader = None # 실시간 수신용 읽기 도구
    st.session_state.rx_run_id = None # 지금 받고 있는 송신의 ID
    st.session_state.rx_info = None # 마지막으로 받은 메시지
    st.session_state.rx_delay_ms = 0.0 # 마지막 메시지의 수신 지연(ms)
    st.session_state.rx_last_time = 0.0 # 마지막으로 받은 시각

def reset_simulation(new_scenario: str):
    st.session_state.scenario = new_scenario
    st.session_state.sim = WarehouseSimulator(scenario=new_scenario)
    st.session_state.prev_fl = None
    st.session_state.prev_wks = {}
    st.session_state.event_logs = []
    st.session_state.is_finished = False # 종료 상태 해제
    st.session_state.pop("latest_data", None) # 이전 화면 데이터 삭제
    st.session_state.prev_levels = {}
    st.session_state.event_seq = 0
    st.session_state.run_seq += 1
    st.session_state.run_start = datetime.now(timezone.utc)
    st.session_state.risk_results = []

def record_event(frame, worker_id, curr_risk, future_risk, total_cri, status):
    """위험 단계가 바뀐 틱에만 이벤트 1건 기록 (risk_rules.md 8절). 작업자마다 따로 판단한다"""
    prev_level = st.session_state.prev_levels.get(worker_id) # 이 작업자의 직전 단계 (없으면 None)
    if status == prev_level:
        return # 같은 단계 유지 -> 기록하지 않음

    st.session_state.event_seq += 1
    sim_time = frame["timestamp"]
    cri_pred = future_risk["max_pred_cri"]
    event_time = st.session_state.run_start + timedelta(seconds=sim_time)

    st.session_state.event_logs.append({
        "event_id": f"evt_{st.session_state.event_seq:04d}",
        "run_id": f"run_{st.session_state.run_seq:03d}",
        "scenario_id": SCENARIO_IDS[st.session_state.scenario],
        "worker_id": worker_id,
        "tick": round(sim_time * 10),
        "simulation_time_s": sim_time,
        "timestamp": event_time.isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        "event_type": "INIT" if prev_level is None else "LEVEL_CHANGE",
        "previous_level": prev_level,
        "current_level": status,
        "distance_m": round(curr_risk["distance"], 3),
        "ttc_s": None if curr_risk["ttc"] is None else round(curr_risk["ttc"], 3),
        "dcpa_m": round(curr_risk["dcpa"], 3),
        "cri": round(curr_risk["cri"], 3),
        "rule_version": RULE_VERSION,
        "cri_pred": cri_pred,
        "cri_total": total_cri,
        "trigger": "CURRENT" if curr_risk["cri"] >= 0.9 * cri_pred else "PREDICTED",
    })
    st.session_state.prev_levels[worker_id] = status

def process_frame(frame):
    """프레임 1개 처리: 작업자마다 위험 계산 -> 단계 판정 -> 기록. 가장 위험한 작업자를 화면 대표로 저장"""
    calc_start = time.perf_counter() # 계산 시작 시각 (성능 측정용)
    fl = frame["forklift"]
    workers = frame.get("workers") or [frame["worker"]] # 작업자 목록 (예전 형식의 프레임이면 1명짜리 목록)

    # 지게차의 행동 추정과 예측 경로는 한 번만 계산해 모든 작업자에게 같이 쓴다
    fl_act = world_engine.estimate_action(st.session_state.prev_fl, fl)
    fl_traj = world_engine.imagine_rollout(fl, fl_act, horizon_steps=25)

    sim_time = frame["timestamp"]
    tick_time = st.session_state.run_start + timedelta(seconds=sim_time)
    worker_results = []

    for wk in workers:
        wk_id = wk["entity_id"]

        # 현재 CRI
        curr_risk = risk_engine.calculate_risk(fl, wk)

        # 예측 CRI (첫 틱은 직전 값이 없어 추정 행동이 (0, 0))
        wk_act = world_engine.estimate_action(st.session_state.prev_wks.get(wk_id), wk)
        wk_traj = world_engine.imagine_rollout(wk, wk_act, horizon_steps=25)
        future_risk = world_engine.evaluate_future_risk(fl_traj, wk_traj, risk_engine)
        
        # 통합 CRI와 위험 단계
        total_cri = round(max(curr_risk["cri"], 0.9 * future_risk["max_pred_cri"]), 3)
        status = classify_level(total_cri)

        worker_results.append({
            "worker": wk,
            "wk_traj": wk_traj,
            "curr_risk": curr_risk,
            "future_risk": future_risk,
            "total_cri": total_cri,
            "status": status
        })

        # 틱별 계산 결과 저장 (data_spec.md 4절 risk_result): 작업자마다 한 줄
        cri_pred = future_risk["max_pred_cri"]
        st.session_state.risk_results.append({
            "run_id": f"run_{st.session_state.run_seq:03d}",
            "scenario_id": SCENARIO_IDS[st.session_state.scenario],
            "worker_id": wk_id,
            "tick": round(sim_time * 10),
            "simulation_time_s": sim_time,
            "timestamp": tick_time.isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "distance_m": round(curr_risk["distance"], 3),
            "relative_speed_mps": round(curr_risk["relative_speed"], 3),
            "closing_speed_mps": round(curr_risk["closing_speed"], 3),
            "ttc_s": None if curr_risk["ttc"] is None else round(curr_risk["ttc"], 3),
            "t_cpa_s": round(curr_risk["t_cpa"], 3),
            "dcpa_m": round(curr_risk["dcpa"], 3),
            "safe_radius_m": round(curr_risk["safe_radius"], 3),
            "cri": round(curr_risk["cri"], 3),
            "risk_level": status,
            "collision_predicted": curr_risk["collision_predicted"],
            "cri_pred": cri_pred,
            "cri_total": total_cri,
            "min_future_distance_m": future_risk["min_future_dist"],
            "critical_offset_s": None if cri_pred == 0 else future_risk["critical_offset"],
        })

        record_event(frame, wk_id, curr_risk, future_risk, total_cri, status)
        st.session_state.prev_wks[wk_id] = wk.copy()

    # 가장 위험한 작업자(통합 CRI가 가장 큰 사람)를 화면 대표로 삼는다. 값이 같으면 더 가까운 사람
    top = max(worker_results, key=lambda r: (r["total_cri"], -r["curr_risk"]["distance"]))

    st.session_state.latest_data = {
        "timestamp": frame["timestamp"],
        "forklift": fl,
        "worker": top["worker"],
        "fl_traj": fl_traj,
        "wk_traj": top["wk_traj"],
        "curr_risk": top["curr_risk"],
        "future_risk": top["future_risk"],
        "total_cri": top["total_cri"],
        "status": top["status"],
        "workers": worker_results, # 작업자별 결과 전체 (3D 화면과 작업자별 표에서 사용)
        "min_distance": min(r["curr_risk"]["distance"] for r in worker_results), # 가장 가까운 작업자와의 거리
    }

    st.session_state.prev_fl = fl.copy()

    # 이 틱의 위험 계산에 걸린 시간(ms): 헹동 추정·예측 롤아웃·위험 계산·기록 포함 (NFR-02)
    calc_ms = round((time.perf_counter() - calc_start) * 1000, 3)
    for row in st.session_state.risk_results[-len(workers):]:
        row["calc_time_ms"] = calc_ms

def find_risk_point(data):
    """화면에 표시할 위험 예상 지점 [x, y]를 구한다. 표시할 필요가 없으면 None"""
    if data["status"] == "SAFE":
        return None
    fl, wk = data["forklift"], data["worker"]
    ttc = data["curr_risk"]["ttc"]
    if ttc is not None:
        # 지금 속도를 유지한다고 볼 때, ttc초 뒤 두 객체 위치의 가운데
        fx = fl["x"] + fl["speed"] * math.cos(fl["heading"]) * ttc
        fy = fl["y"] + fl["speed"] * math.sin(fl["heading"]) * ttc
        wx = wk["x"] + wk["speed"] * math.cos(wk["heading"]) * ttc
        wy = wk["y"] + wk["speed"] * math.sin(wk["heading"]) * ttc
    else:
        # 예측 경로에서 두 객체가 가장 가까워지는 순간의 가운데
        f, w = min(zip(data["fl_traj"], data["wk_traj"]),
                   key=lambda pair:math.hypot(pair[0]["x"] - pair[1]["x"], pair[0]["y"] - pair[1]["y"]))
        fx, fy, wx, wy = f["x"], f["y"], w["x"], w["y"]
    return [round((fx + wx) / 2, 2), round((fy + wy) / 2, 2)]

# 4. 상단 타이틀 및 설명
st.title("지게차·작업자 충돌 위험 관제")
st.caption("지게차와 작업자의 위치를 추적해, 부딪힐 위험을 미리 알려 줍니다")

# 5. 사이드바: 시나리오 및 시뮬레이션 제어 패널
with st.sidebar:
    st.header("시나리오 제어")

    input_mode = st.radio("데이터 입력", ["내장 시뮬레이터", "실시간 수신"], horizontal=True)
    is_receive = input_mode == "실시간 수신"
    if st.session_state.input_mode != input_mode:
        # 입력 방식을 바꾸면 화면과 기록을 새로 시작한다
        st.session_state.input_mode = input_mode
        st.session_state.is_running = False
        st.session_state.reader = StreamReader() if is_receive else None
        st.session_state.rx_run_id = None
        st.session_state.rx_info = None
        reset_simulation(st.session_state.scenario)
        st.rerun()
    
    scenario_map = {
        "S01 정상 이동": "parallel",
        "S02 정지 접근": "stop_approach",
        "S03 정면 접근": "head_on",
        "S04 교차 이동": "crossing",
        "S05 접근 후 이탈": "retreat",
        "S06 코너 회전 (월드 모델 검증)": "corner_turn",
        "S07 다중 작업자 (5명)": "multi_worker",
    }
    selected_label = st.selectbox("시연 시나리오 선택", list(scenario_map.keys()))
    selected_code = scenario_map[selected_label]

    if not is_receive and selected_code != st.session_state.scenario:
        reset_simulation(selected_code)
        st.rerun()

    # 실시간 수신 모드: 송신 프로그램을 앱에서 바로 실행하는 버튼 (배포 주소에서도 수신 모드를 볼 수 있게 함)
    if is_receive:
        sender = st.session_state.get("sender_proc")
        sending = sender is not None and sender.poll() is None # 아직 보내는 중이면 True
        if st.button("시연용 송신 시작", use_container_width=True, disabled=sending):
            app_dir = os.path.dirname(os.path.abspath(__file__)) # app.py가 있는 폴더
            st.session_state.sender_proc = subprocess.Popen(
                [sys.executable, "sensor_sender.py", "--scenario", selected_code],
                cwd=app_dir,
            )

    view_labels = {"관제 시점 (위에서)": "overview", "지게차 1인칭": "forklift", "작업자 1인칭": "worker"}
    view_mode = view_labels[st.radio("3D 시점", list(view_labels.keys()))]
    auto_response = st.checkbox("자동 대응 (가상 서행·비상 정지)", value=False, disabled=is_receive)
    play_speed = st.radio("재생 속도", ["1배속", "0.5배속"], horizontal=True)

    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("▶️ 자동 재생", use_container_width=True, type="primary"):
            if not st.session_state.is_finished: # 종료된 시나리오는 다시 재생하지 않는다
                st.session_state.is_running = True
    with col_btn2:
        if st.button("⏸️ 일시 정지", use_container_width=True):
            st.session_state.is_running = False

    col_btn3, col_btn4 = st.columns(2)
    with col_btn3:
        step_clicked = st.button("⏭️ +0.2초 진행", use_container_width=True)
    with col_btn4:
        if st.button("🔄 초기화", use_container_width=True):
            st.session_state.is_running = False
            reset_simulation(st.session_state.scenario)
            st.rerun()

    st.divider()
    st.markdown("### 화면 보는 법")
    st.info(
        "- **바닥의 원**: 지게차 주변 안전 구역. 초록(정상), 노랑(주의), 빨강(위험)\n"
        "- **보라·하늘색 선**: 앞으로 2.5초 동안의 예상 이동 경로\n"
        "- 지금은 멀리 있어도 예상 경로가 가까워지면 미리 경보가 뜹니다"
    )

# 첫 화면: tick 0(시간이 흐르기 전) 상태를 판정하고 INIT 이벤트를 기록
if "latest_data" not in st.session_state:
    process_frame(st.session_state.sim.get_frame())

# 5-1. 실시간 수신: 송신 프로그램이 보낸 프레임을 읽어 같은 계산 과정에 넣는다
if is_receive:
    for msg in st.session_state.reader.read_new():
        if msg["run_id"] != st.session_state.rx_run_id:
            # 새 송신이 시작됨 -> 기록을 비우고 그 시나리오의 화면 배치로 바꾼다
            reset_simulation(msg["scenario"])
            st.session_state.rx_run_id = msg["run_id"]
        process_frame(msg["frame"])
        sent_at = datetime.fromisoformat(msg["sent_at"].replace("Z", "+00:00"))
        st.session_state.rx_delay_ms = (datetime.now(timezone.utc) - sent_at).total_seconds() * 1000
        st.session_state.rx_last_time = time.time()
        st.session_state.rx_info = msg

        for row in st.session_state.risk_results[-len(st.session_state.latest_data["workers"]):]:
            row["rx_delay_ms"] = round(st.session_state.rx_delay_ms, 1) # 생성 -> 처리 완료까지 걸린 시간

# 6. 시뮬레이션 스텝 진행 (자동 재생 중이거나 수동 스텝 클릭 시 2프레임=0.2초씩 전진)
if not is_receive and (st.session_state.is_running or step_clicked) and not st.session_state.is_finished:
    stop_distance = SCENARIO_STOP_DISTANCE[st.session_state.scenario]
    max_time = SCENARIO_MAX_TIME[st.session_state.scenario]

    steps = 1 if (play_speed == "0.5배속" and not step_clicked) else 2
    for _ in range(steps):
        # 직전 틱의 판정 결과를 시뮬레이터에 전달한다 (자동 대응이 꺼져 있으면 None)
        response_level = st.session_state.latest_data["status"] if auto_response else None
        frame = st.session_state.sim.step(response_level)
        process_frame(frame)

        # 종료 판정: 이 틱의 계산과 기록을 마친 뒤에 확인한다.
        reached_distance = stop_distance is not None and st.session_state.latest_data["min_distance"] <= stop_distance + EPSILON
        reached_time = frame["timestamp"] >= max_time
        stopped = st.session_state.sim.is_stopped() # 비상 정지로 둘 다 멈춤
        if reached_distance or reached_time or stopped:
            st.session_state.is_finished = True
            st.session_state.is_running = False
            break

data = st.session_state.latest_data

# 수신 상태 표시: 자리를 항상 한 칸 잡아 둔다 (모드에 따라 요소 개수가 달라지면 아래 화면이 밀려서 겹쳐 보인다)
rx_slot = st.empty() # 내용 없이 자리만 잡는 빈 칸
if is_receive:
    rx = st.session_state.rx_info
    if rx is None:
        rx_slot.info("실시간 수신 대기 중입니다. 왼쪽에서 시나리오를 고르고 [시연용 송신 시작]을 누르면, 송신 프로그램이 보내는 위치 데이터를 받아 판정합니다.")
    else:
        idle = time.time() - st.session_state.rx_last_time
        rx_state = "수신 중" if idle < 1.0 else f"수신 멈춤 (마지막 수신 {idle:.0f}초 전)"
        rx_slot.caption(f"실시간 {rx_state} · 송신 ID {rx['run_id']} · {SCENARIO_IDS[rx['scenario']]} · tick {rx['tick']} · 수신 지연 {st.session_state.rx_delay_ms:.0f} ms")

# 6-1. 경보 배너: 위험 단계, 남은 시간, 권장 조치를 화면 전체 폭으로 크게 보여 준다
sim = st.session_state.sim
status = data["status"]
distance = data["curr_risk"]["distance"]
ttc = data["curr_risk"]["ttc"]
fl_speed = data["forklift"]["speed"]
banner_stop_distance = SCENARIO_STOP_DISTANCE[st.session_state.scenario]
collided = banner_stop_distance is not None and data["min_distance"] <= banner_stop_distance + EPSILON
who = f"{data['worker']['entity_id']} · " if len(data["workers"]) > 1 else "" # 작업자가 여러 명일 때만 누구인지 표시

if ttc is not None:
    time_text = f"약 {ttc:.1f}초 뒤 충돌 예상"
else:
    time_text = f"예상 경로에서 {data['future_risk']['min_future_dist']:.1f} m까지 접근"

if collided:
    banner = ("#7f1d1d", "충돌 발생", f"{who}거리 {distance:.2f} m까지 접근했습니다")
elif sim.is_stopped():
    banner = ("#1d4ed8", "비상 정지 완료 · 충돌 회피", f"지게차와 작업자가 {distance:.2f} m 거리를 두고 멈췄습니다")
elif sim.response_state == "STOP":
    banner = ("#dc2626", "비상 정지 작동 중", f"지게차 제동 중 (속력 {fl_speed:.2f} m/s) · 작업자에게 정지 알림 · 현재 거리 {distance:.2f} m")
elif status == "CRITICAL":
    banner = ("#dc2626", f"위험 · {who}{time_text}", f"권장 조치: {RECOMMENDED_ACTIONS['CRITICAL']}")
elif status == "WARNING":
    banner = ("#b45309", f"주의 · {who}{time_text}", f"권장 조치: {RECOMMENDED_ACTIONS['WARNING']}")
else:
    banner = ("#15803d", "정상", "충돌 위험이 없습니다")

banner_color, banner_title, banner_detail = banner
if sim.response_state == "SLOW":
    banner_detail += f" · 자동 대응: 지게차 서행 중 ({fl_speed:.2f} m/s)"

banner_html = (
    f'<div style="background:{banner_color};color:white;padding:18px 24px;border-radius:12px;margin-bottom:12px;">'
    f'<div style="font-size:34px;font-weight:800;">{banner_title}</div>'
    f'<div style="font-size:18px;margin-top:6px;">{banner_detail}</div>'
    f'</div>'
)
st.markdown(banner_html, unsafe_allow_html=True)

# 7. 상단 핵심 지표(KPI) 카드 영역
m1, m2, m3, m4, m5 = st.columns(5)
m1.metric("경과 시간", f"{data['timestamp']:.1f} 초")
m2.metric("현재 거리", f"{data['curr_risk']['distance']:.2f} m")
m3.metric("현재 위험도", f"{data['curr_risk']['cri']:.3f}")
m4.metric("예측 위험도", f"{data['future_risk']['max_pred_cri']:.3f}")

status = data["status"]
if status == "CRITICAL":
    m5.error(f"위험 (통합 위험도: {data['total_cri']})")
elif status == "WARNING":
    m5.warning(f"주의 (통합 위험도: {data['total_cri']})")
else:
    m5.success(f"정상 (통합 위험도: {data['total_cri']})")

if st.session_state.is_finished:
    st.info(f"시나리오가 종료되었습니다 (종료 시각 {data['timestamp']:.1f}초). 다시 보려면 [초기화]를 누르세요.")

# 8. 메인 화면 분할: 왼쪽(Three.js 3D 관제 화면), 오른쪽(경보 이벤트 로그)
col_3d, col_log = st.columns([2.2, 1])

with col_3d:
    st.subheader("현장 3D 화면")
    
    # Python 데이터를 JSON으로 변환하여 Three.js HTML 코드에 주입
    scene_payload = json.dumps({
        "forklift": data["forklift"],
        "worker": data["worker"],
        "fl_traj": data["fl_traj"],
        "wk_traj": data["wk_traj"],
        "status": data["status"],
        "safe_radius": data["curr_risk"]["safe_radius"], # 동적 안전 버블 반경(m)
        "racks": SCENARIO_RACKS[st.session_state.scenario], # 시나리오별 랙 배치
        "view": SCENARIO_VIEW[st.session_state.scenario], # 시나리오별 화면 중심·카메라 거리
        "view_mode": view_mode, # 관제 시점 / 지게차 1인칭 / 작업자 1인칭
        "scenario_id": SCENARIO_IDS[st.session_state.scenario], # 시점 저장용 이름에 사용
        "risk_point": find_risk_point(data), # 위험 예상 지점 [x, y]. 없으면 None
        "workers": [
            {"id": r["worker"]["entity_id"], "x": r["worker"]["x"], "y": r["worker"]["y"],
             "speed": r["worker"]["speed"], "heading": r["worker"]["heading"],
             "status": r["status"], "traj": r["wk_traj"]}
             for r in data["workers"]
        ], # 작업자 전원의 위치·단계·예상 경로
        "time": data["timestamp"] # 경과 시간(초). 걷는 동작에 사용
    })

    threejs_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <style>
            body {{ margin: 0; overflow: hidden; background-color: #0f172a; border-radius: 12px; }}
            #info {{
                position: absolute; top: 12px; left: 12px; color: #f8fafc;
                font-family: sans-serif; font-size: 13px; background: rgba(15, 23, 42, 0.75);
                padding: 8px 12px; border-radius: 8px; border: 1px solid #334155;
            }}
        </style>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    </head>
    <body>
        <div id="info">
            <span style="color:#f59e0b;">■</span> <b>지게차</b> &nbsp;|&nbsp; <span style="color:#38bdf8;">■</span> <b>작업자</b> &nbsp;|&nbsp; <span style="color:#c084fc;">■</span> <b>예상 경로(2.5초)</b> &nbsp;|&nbsp; <span style="color:#ef4444;">◎</span> <b>위험 예상 지점</b><br>
            <span style="color:#94a3b8; font-size:11px;">드래그: 회전 &nbsp;|&nbsp; Shift+드래그: 이동 &nbsp;|&nbsp; 휠: 확대/축소 &nbsp;|&nbsp; 더블클릭: 초기화 &nbsp;|&nbsp; 안전 버블 반경: {data['curr_risk']['safe_radius']:.1f} m</span>
        </div>
        <script>
            const payload = {scene_payload};

            // 1. 씬, 카메라, 렌더러 설정
            const scene = new THREE.Scene();
            scene.background = new THREE.Color(0x0f172a);

            const camera = new THREE.PerspectiveCamera(45, window.innerWidth / 460, 0.1, 1000);
            // 카메라 상태: 화면 중심(cx, cz), 수평 거리(camDist), 수평 회전각(angle)
            let cx = payload.view[0], cz = payload.view[1], camDist = payload.view[2];
            let angle = 0;

            // 시점 상태를 브라우저에 저장했다가, 화면이 새로 만들어질 때 되살린다 (자동 재생 중 시점 유지)
            const viewKey = "safepredict_view_" + payload.scenario_id + "_" + payload.view_mode;
            try {{
                const saved = JSON.parse(sessionStorage.getItem(viewKey));
                if (saved) {{ cx = saved.cx; cz = saved.cz; camDist = saved.camDist; angle = saved.angle; }}
            }} catch (e) {{}}
            function saveView() {{
                try {{
                    sessionStorage.setItem(viewKey, JSON.stringify({{ cx: cx, cz: cz, camDist: camDist, angle: angle }}));
                }} catch (e) {{}}
            }}

            function updateCamera() {{
                if (payload.view_mode === "overview") {{
                    // 관제 시점: 화면 중심 둘레의 원 위에서 내려다본다
                    camera.fov = 45;
                    camera.position.set(cx + Math.sin(angle) * camDist, camDist * 1.22, cz + Math.cos(angle) * camDist);
                    camera.lookAt(cx, 0, cz);
                }} else {{
                    // 1인칭 시점: 선택한 객체의 눈높이에서 진행 방향을 본다 (드래그하면 좌우로 둘러봄)
                    const me = payload.view_mode === "forklift" ? payload.forklift : payload.workers[0];
                    const eye = payload.view_mode === "forklift" ? 2.0 : 1.6;
                    const look = me.heading + angle;
                    camera.fov = 70;
                    camera.position.set(me.x, eye, me.y);
                    camera.lookAt(me.x + Math.cos(look) * 10, eye - 1.5, me.y + Math.sin(look) * 10);
                }}
                camera.updateProjectionMatrix();
            }}
            updateCamera();

            const renderer = new THREE.WebGLRenderer({{ antialias: true }});
            renderer.setSize(window.innerWidth, 460);
            document.body.appendChild(renderer.domElement);

            // 2. 조명 및 바닥 그리드(물류창고)
            const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
            scene.add(ambientLight);
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
            dirLight.position.set(15, 30, 20);
            scene.add(dirLight);

            const gridHelper = new THREE.GridHelper(30, 30, 0x334155, 0x1e293b);
            gridHelper.position.set(payload.view[0], 0, payload.view[1]);
            scene.add(gridHelper);

            // 3. 물류 선반(랙) 배치: 시나리오별 위치를 payload.racks로 받는다 (화면 표시용)
            const rackMat = new THREE.MeshLambertMaterial({{ color: 0x475569, transparent: true, opacity: 0.85}});
            payload.racks.forEach(r => {{
                const rack = new THREE.Mesh(new THREE.BoxGeometry(r[2], 2.5, r[3]), rackMat);
                rack.position.set(r[0], 1.25, r[1]);
                scene.add(rack);
            }});

            // 4. 지게차 3D 객체 및 동적 안전 버블
            const flGroup = new THREE.Group();
            const flBody = new THREE.Mesh(
                new THREE.BoxGeometry(1.6, 1.2, 1.0),
                new THREE.MeshLambertMaterial({{ color: 0xf59e0b }})
            );
            flBody.position.y = 0.6;
            flGroup.add(flBody);

            // 위험 단계별 안전 버블 색상
            let bubbleColor = 0x22c55e; // SAFE (초록)
            if (payload.status === "WARNING") bubbleColor = 0xeab308; // WARNING (노랑)
            if (payload.status === "CRITICAL") bubbleColor = 0xef4444; // CRITICAL (빨강)

            // 동적 안전 버블: 반경 = safe_radius_m (3.0 + 0.5 x 지게차 속력)
            const safeRadius = payload.safe_radius;

            // 버블 내부 (옅게 채움)
            const bubbleFill = new THREE.Mesh(
                new THREE.CircleGeometry(safeRadius, 64),
                new THREE.MeshBasicMaterial({{ color: bubbleColor, side: THREE.DoubleSide, transparent: true, opacity: 0.12 }})
            );
            bubbleFill.rotation.x = -Math.PI / 2;
            bubbleFill.position.y = 0.04;
            flGroup.add(bubbleFill);

            // 버블 경계선 (진하게)
            const bubbleEdge = new THREE.Mesh(
                new THREE.RingGeometry(safeRadius - 0.12, safeRadius, 64),
                new THREE.MeshBasicMaterial({{ color: bubbleColor, side: THREE.DoubleSide, transparent: true, opacity: 0.8 }})
            );
            bubbleEdge.rotation.x = -Math.PI / 2;
            bubbleEdge.position.y = 0.05;
            flGroup.add(bubbleEdge);

            // 시뮬레이터 (x, y) -> Three.js (x, 0, y) 매핑
            flGroup.position.set(payload.forklift.x, 0, payload.forklift.y);
            flGroup.rotation.y = -payload.forklift.heading;
            scene.add(flGroup);

            // 5. 작업자 3D 객체: 머리·몸통·팔·다리로 만든 사람 모양. 작업자 수만큼 만든다
            const wkMat = new THREE.MeshLambertMaterial({{ color: 0x38bdf8 }});
            const levelColors = {{ SAFE: 0x22c55e, WARNING: 0xeab308, CRITICAL: 0xef4444 }};
            const showLabels = payload.workers.length > 1; // 여러 명일 때만 머리 위에 ID를 띄운다
            const markY = showLabels ? 3.4 : 2.3; // 경고 표시 높이 (ID 글자와 겹치지 않게)
            const headMarks = [];

            // 글자를 그린 작은 그림을 3D 공간에 띄운다 (항상 카메라 쪽을 향한다)
            function makeLabel(text, cssColor) {{
                const canvas = document.createElement('canvas');
                canvas.width = 128; canvas.height = 48;
                const ctx = canvas.getContext('2d');
                ctx.font = 'bold 34px sans-serif';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillStyle = cssColor;
                ctx.fillText(text, 64, 24);
                const sprite = new THREE.Sprite(new THREE.SpriteMaterial({{ map: new THREE.CanvasTexture(canvas), transparent: true, depthTest: false }}));
                sprite.scale.set(3.2, 1.2, 1);
                return sprite;
            }}


            // 팔다리 하나를 만든다: 위쪽 끝(어깨·골반)을 축으로 앞뒤로 흔들 수 있게 그룹에 담는다
            function addLimb(group, length, radius, jointY, sideZ, swingAngle) {{
                const joint = new THREE.Group();
                joint.position.set(0, jointY, sideZ);
                const limb = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 8), wkMat);
                limb.position.y = -length / 2;
                joint.add(limb);
                joint.rotation.z = swingAngle;
                group.add(joint);
            }}

            payload.workers.forEach((w, index) => {{
                // 작업자 1인칭에서는 자기 자신(첫 번째 작업자)을 그리지 않는다
                if (payload.view_mode === "worker" && index === 0) return;

                const wkGroup = new THREE.Group();

                const wkHead = new THREE.Mesh(new THREE.SphereGeometry(0.2, 16, 12), wkMat);
                wkHead.position.y = 1.55;
                wkGroup.add(wkHead);

                const wkBody = new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.13, 0.6, 12), wkMat);
                wkBody.position.y = 1.1;
                wkGroup.add(wkBody);

                // 걷는 동작: 경과 시간에 따라 팔다리 각도를 바꾼다 (멈춰 있으면 가만히 선다)
                const swing = w.speed > 0 ? Math.sin(payload.time * Math.PI / 0.4) * 0.6 : 0;
                addLimb(wkGroup, 0.8, 0.08, 0.8, 0.1, swing);   // 한쪽 다리
                addLimb(wkGroup, 0.8, 0.08, 0.8, -0.1, -swing); // 반대쪽 다리
                addLimb(wkGroup, 0.6, 0.06, 1.38, 0.2, -swing); // 한쪽 팔 (같은 쪽 다리와 반대로)
                addLimb(wkGroup, 0.6, 0.06, 1.38, -0.2, swing); // 반대쪽 팔

                wkGroup.position.set(w.x, 0, w.y);
                wkGroup.rotation.y = -w.heading; // 진행 방향을 보게 돌린다
                scene.add(wkGroup);

                // 머리 위 ID: 위험 단계에 따라 글자색이 바뀐다 (정상: 흰색, 주의: 노랑, 위험: 빨강)
                if (showLabels) {{
                    const cssColor = w.status === "CRITICAL" ? "#ef4444" : (w.status === "WARNING" ? "#eab308" : "#e2e8f0");
                    const label = makeLabel(w.id, cssColor);
                    label.position.set(w.x, 2.5, w.y);
                    scene.add(label);
                }}

                // 주의·위험인 작업자 머리 위에 마름모 모양 경고 표시를 띄운다
                if (w.status !== "SAFE") {{
                    const mark = new THREE.Mesh(
                        new THREE.OctahedronGeometry(0.22),
                        new THREE.MeshBasicMaterial({{ color: levelColors[w.status ]}})
                    );
                    mark.position.set(w.x, markY, w.y);
                    scene.add(mark);
                    headMarks.push(mark);
                }}
            }});



            // 6. 월드 모델 예측 미래 궤적 (Imagination Rollout 잔상 렌더링)
            function drawTrajectory(trajPoints, colorHex) {{
                const points = trajPoints.map(p => new THREE.Vector3(p.x, 0.15, p.y));
                const lineGeo = new THREE.BufferGeometry().setFromPoints(points);
                const lineMat = new THREE.LineBasicMaterial({{ color: colorHex, linewidth: 3 }});
                const line = new THREE.Line(lineGeo, lineMat);
                scene.add(line);

                // 미래 예측 위치 점(Node) 표시
                trajPoints.forEach((p, idx) => {{
                    if (idx % 4 === 0) {{
                        const dot = new THREE.Mesh(
                            new THREE.SphereGeometry(0.16, 8, 8),
                            new THREE.MeshBasicMaterial({{ color: colorHex, transparent: true, opacity: 1 - (idx / 30) }})
                        );
                        dot.position.set(p.x, 0.15, p.y);
                        scene.add(dot);
                    }}
                }});
            }}

            drawTrajectory(payload.fl_traj, 0xc084fc); // 지게차 미래 예측 궤적 (보라색)
            payload.workers.forEach(w => {{
                if (w.speed > 0) drawTrajectory(w.traj, 0x67e8f9); // 움직이는 작업자의 미래 예측 궤적 (하늘색)
            }});

            // 6-1. 경보 효과: 위험 예상 지점과 작업자 머리 위 경고 표시
            const alertColor = bubbleColor; // 위험 단계 색 (주의: 노랑, 위험: 빨강)
            let riskRing = null;

            if (payload.risk_point) {{
                // 바닥의 고리와 세로 기둥으로 위험 예상 지점을 표시한다
                riskRing = new THREE.Mesh(
                    new THREE.RingGeometry(0.45, 0.7, 32),
                    new THREE.MeshBasicMaterial({{ color: alertColor, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }})
                );
                riskRing.rotation.x = -Math.PI / 2;
                riskRing.position.set(payload.risk_point[0], 0.08, payload.risk_point[1]);
                scene.add(riskRing);

                const riskPole = new THREE.Mesh(
                    new THREE.CylinderGeometry(0.05, 0.05, 3.0, 8),
                    new THREE.MeshBasicMaterial({{ color: alertColor, transparent: true, opacity: 0.7 }})
                );
                riskPole.position.set(payload.risk_point[0], 1.5, payload.risk_point[1]);
                scene.add(riskPole);
            }}

            // 7. 카메라 조작: 드래그(회전), Shift+드래그(이동), 휠(확대/축소), 더블클릭(초기화)
            function redraw() {{
                updateCamera();
                renderer.render(scene, camera); // 바뀐 시점으로 다시 그린다
                saveView();
            }}

            let isDragging = false;
            let prevX = 0, prevY = 0;
            document.addEventListener('mousedown', e => {{ isDragging = true; prevX = e.clientX; prevY = e.clientY; }});
            document.addEventListener('mouseup', () => {{ isDragging = false; }});
            document.addEventListener('mousemove', e => {{
                if (!isDragging) return;
                const dx = e.clientX - prevX, dy = e.clientY - prevY;
                prevX = e.clientX; prevY = e.clientY;
                if (e.shiftKey && payload.view_mode === "overview") {{
                    // Shift+드래그: 화면 중심을 옮긴다 (이동)
                    const k = camDist * 0.002;
                    cx -= (Math.cos(angle) * dx + Math.sin(angle) * dy) * k;
                    cz -= (-Math.sin(angle) * dx + Math.cos(angle) * dy) * k;
                }} else {{
                    angle += dx * 0.01; // 드래그: 좌우 회전 (1인칭에서는 좌우로 둘러보기)
                }}
                redraw();
            }});
            document.addEventListener('wheel', e => {{
                if (payload.view_mode !== "overview") return;
                e.preventDefault(); // 페이지 스크롤 대신 확대/축소
                camDist = Math.min(40, Math.max(4, camDist * (e.deltaY > 0 ? 1.1 : 0.9)));
                redraw();
            }}, {{ passive: false }});
            document.addEventListener('dblclick', () => {{
                // 더블클릭: 시점을 처음 상태로 되돌린다
                cx = payload.view[0]; cz = payload.view[1]; camDist = payload.view[2]; angle = 0;
                redraw();
            }});

            // 8. 화면 그리기: 위험 단계에서는 계속 다시 그려서 깜빡이게 한다
            function animate() {{
                const pulse = 0.5 + 0.5 * Math.sin(Date.now() / 150); // 0~1 사이를 약 1초 주기로 오간다
                bubbleFill.material.opacity = 0.10 + 0.30 * pulse;
                bubbleEdge.material.opacity = 0.40 + 0.60 * pulse;
                if (riskRing) riskRing.scale.setScalar(1.0 + 0.4 * pulse);
                headMarks.forEach(m => {{ m.position.y = markY + 0.15 * pulse; }});
                renderer.render(scene, camera);
                requestAnimationFrame(animate);
            }}

            if (payload.status === "CRITICAL") {{
                animate();
            }} else {{
                renderer.render(scene, camera);
            }}
        </script>
    </body>
    </html>
    """
    components.html(threejs_html, height=465)

    # 작업자별 상태 표: 누가 어떤 단계인지 한눈에 본다
    level_names = {"SAFE": "정상", "WARNING": "주의", "CRITICAL": "위험"}
    level_styles = {"주의": "background-color: #b45309; color: white", "위험": "background-color: #dc2626; color: white"}
    df_workers = pd.DataFrame({
        "작업자": [r["worker"]["entity_id"] for r in data["workers"]],
        "단계": [level_names[r["status"]] for r in data["workers"]],
        "통합 위험도": [r["total_cri"] for r in data["workers"]],
        "거리(m)": [round(r["curr_risk"]["distance"], 2) for r in data["workers"]],
        "충돌 예상(초)": ["-" if r["curr_risk"]["ttc"] is None else f"{r['curr_risk']['ttc']:.1f}" for r in data["workers"]],
    })
    st.dataframe(df_workers.style.format({"통합 위험도": "{:.3f}", "거리(m)": "{:.2f}"}).map(lambda v: level_styles.get(v, ""), subset=["단계"]),
                 use_container_width=True, hide_index=True)

with col_log:
    st.subheader("위험 단계 변경 이력")
    if len(st.session_state.event_logs) > 0:
        df_logs = pd.DataFrame(st.session_state.event_logs)
        df_view = pd.DataFrame({
            "시간(s)": df_logs["simulation_time_s"].map(lambda t: f"{t:.1f}"),
            "작업자": df_logs["worker_id"],
            "이벤트": df_logs["event_type"].map({"INIT": "시작", "LEVEL_CHANGE": "변경"}),
            "이전 단계": df_logs["previous_level"].map(level_names).fillna("-"),
            "현재 단계": df_logs["current_level"].map(level_names),
            "통합 CRI": df_logs["cri_total"],
            "거리(m)": df_logs["distance_m"].round(2),
            "원인": df_logs["trigger"].map({"CURRENT": "현재", "PREDICTED": "예측"}),
        })
        st.dataframe(df_view.iloc[::-1], use_container_width=True, height=330, hide_index=True)
    else:
        st.info("현재까지 발생한 주의/위험 경보 이벤트가 없습니다.\n왼쪽에서 [자동 재생]을 눌러 시뮬레이션을 시작하세요.")

    # CSV 다운로드 (세션 기록 -> 화면 표시 -> CSV)
    run_label = f"{SCENARIO_IDS[st.session_state.scenario]}_run_{st.session_state.run_seq:03d}"
    st.download_button(
        "이벤트 로그 CSV",
        data=pd.DataFrame(st.session_state.event_logs).to_csv(index=False).encode("utf-8-sig"),
        file_name=f"events_{run_label}.csv",
        mime="text/csv",
        use_container_width=True,
    )
    st.download_button(
        "틱별 계산 결과 CSV",
        data=pd.DataFrame(st.session_state.risk_results).to_csv(index=False).encode("utf-8-sig"),
        file_name=f"risk_results_{run_label}.csv",
        mime="text/csv",
        use_container_width=True,
    )

    # 성능 요약: 틱당 위험 계산 시간, 실시간 수신일 때는 수신 지연도 함께 (NFR-02~04)
    df_perf = pd.DataFrame(st.session_state.risk_results).drop_duplicates("tick")
    perf_text = (f"위험 계산 시간({len(df_perf)}틱): 평균 {df_perf['calc_time_ms'].mean():.1f} ms · "
                 f"P95 {df_perf['calc_time_ms'].quantile(0.95):.1f} ms · 최대 {df_perf['calc_time_ms'].max():.1f} ms")
    if "rx_delay_ms" in df_perf.columns and df_perf["rx_delay_ms"].notna().any():
        perf_text += (f"  \n수신 지연(생성→처리 완료): 평균 {df_perf['rx_delay_ms'].mean():.0f} ms · "
                      f"P95 {df_perf['rx_delay_ms'].quantile(0.95):.0f} ms · 최대 {df_perf['rx_delay_ms'].max():.0f} ms")
    st.caption(perf_text)

# 9. 자동 재생 루프 처리 (부드러운 3D 프레임 갱신)
if st.session_state.is_running or is_receive:
    time.sleep(0.25)
    st.rerun()