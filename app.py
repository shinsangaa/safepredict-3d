# 실행 명령어 : python -m streamlit run app.py
import time
import json
import pandas as pd
import streamlit as st
import streamlit.components.v1 as components

from simulator import WarehouseSimulator, SCENARIO_MAX_TIME, SCENARIO_STOP_DISTANCE
from risk_engine import CollisionRiskEngine, EPSILON
from world_model import IndustrialWorldEngine

# 1. 페이지 기본 설정 (넓은 관제 화면 레이아웃)
st.set_page_config(
    page_title="SafePredict 3D | 월드 모델 기반 안전 관제",
    page_icon="🛡️",
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

# 3. 세션 상태(Session State) 초기화
if "sim" not in st.session_state:
    st.session_state.scenario = "parallel"
    st.session_state.sim = WarehouseSimulator(scenario="parallel")
    st.session_state.prev_fl = None
    st.session_state.prev_wk = None
    st.session_state.event_logs = []
    st.session_state.is_running = False
    st.session_state.is_finished = False # 시나리오 종료 여부

def reset_simulation(new_scenario: str):
    st.session_state.scenario = new_scenario
    st.session_state.sim = WarehouseSimulator(scenario=new_scenario)
    st.session_state.prev_fl = None
    st.session_state.prev_wk = None
    st.session_state.event_logs = []
    st.session_state.is_finished = False # 종료 상태 해제
    st.session_state.pop("latest_data", None) # 이전 화면 데이터 삭제

# 4. 상단 타이틀 및 설명
st.title("🛡️ SafePredict 3D : 산업현장 월드 모델 기반 안전 관제 시스템")
st.caption("물리·규칙 기반 충돌 지표(TTC, DCPA) + 경량 월드 모델(Imagination Rollout) 통합 관제 PoC")

# 5. 사이드바: 시나리오 및 시뮬레이션 제어 패널
with st.sidebar:
    st.header("🎛️ 관제 시나리오 제어")
    
    scenario_map = {
        "S01 정상 이동": "parallel",
        "S02 정지 접근": "stop_approach",
        "S03 정면 접근": "head_on",
        "S04 교차 이동": "crossing",
        "S05 접근 후 이탈": "retreat",
        "S06 코너 회전 (월드 모델 검증)": "corner_turn",
    }
    selected_label = st.selectbox("시연 시나리오 선택", list(scenario_map.keys()))
    selected_code = scenario_map[selected_label]

    if selected_code != st.session_state.scenario:
        reset_simulation(selected_code)
        st.rerun()

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
    st.markdown("### 🧠 월드 모델 작동 원리")
    st.info(
        "- **초록/파랑 실선**: 현재 위치 및 동적 안전 버블\n"
        "- **보라색 점선(잔상)**: 월드 모델이 상상(Rollout)한 향후 2.5초 예측 궤적\n"
        "- **선제 경보**: 현재 거리가 멀어도 예측 궤적이 교차하면 즉시 `CRITICAL` 경보 발령"
    )

# 6. 시뮬레이션 스텝 진행 (자동 재생 중이거나 수동 스텝 클릭 시 2프레임=0.2초씩 전진)
if (st.session_state.is_running or step_clicked) and not st.session_state.is_finished:
    stop_distance = SCENARIO_STOP_DISTANCE[st.session_state.scenario]
    max_time = SCENARIO_MAX_TIME[st.session_state.scenario]

    for _ in range(2):
        frame = st.session_state.sim.step()
        fl = frame["forklift"]
        wk = frame["worker"]

        # 현재 물리 기반 위험도
        curr_risk = risk_engine.calculate_risk(fl, wk)

        # 월드 모델 기반 미래 2.5초 롤아웃 및 위험 평가
        fl_act = world_engine.estimate_action(st.session_state.prev_fl, fl)
        wk_act = world_engine.estimate_action(st.session_state.prev_wk, wk)
        fl_traj = world_engine.imagine_rollout(fl, fl_act, horizon_steps=25)
        wk_traj = world_engine.imagine_rollout(wk, wk_act, horizon_steps=25)
        future_risk = world_engine.evaluate_future_risk(fl_traj, wk_traj, risk_engine)

        # 통합 CRI 및 상태 판정
        total_cri = round(max(curr_risk["cri"], 0.9 * future_risk["max_pred_cri"]), 3)
        if total_cri >= 0.85:
            status = "CRITICAL"
        elif total_cri >= 0.50:
            status = "WARNING"
        else:
            status = "SAFE"

        # 최신 프레임 결과 저장
        st.session_state.latest_data = {
            "timestamp": frame["timestamp"],
            "forklift": fl,
            "worker": wk,
            "fl_traj": fl_traj,
            "wk_traj": wk_traj,
            "curr_risk": curr_risk,
            "future_risk": future_risk,
            "total_cri": total_cri,
            "status": status
        }

        # WARNING 이상 경보 발생 시 이벤트 로그에 누락 없이 기록
        if status in ["WARNING", "CRITICAL"]:
            st.session_state.event_logs.insert(0, {
                "시간(s)": f"{frame['timestamp']:.1f}s",
                "위험단계": f"🚨 {status}" if status == "CRITICAL" else f"⚠️ {status}",
                "통합 CRI": total_cri,
                "현재 거리(m)": round(curr_risk["distance"], 2),
                "예측 최소거리(m)": future_risk["min_future_dist"],
                "사유": "월드모델 궤적 교차 예측" if future_risk["max_pred_cri"] > curr_risk["cri"] else "근접/직접 충돌 경로"
            })

        st.session_state.prev_fl = fl.copy()
        st.session_state.prev_wk = wk.copy()

        # 종료 판정: 이 틱의 계산과 기록을 마친 뒤에 확인한다.
        reached_distance = stop_distance is not None and curr_risk["distance"] <= stop_distance + EPSILON
        reached_time = frame["timestamp"] >= max_time
        if reached_distance or reached_time:
            st.session_state.is_finished = True
            st.session_state.is_running = False
            break

# 초기 화면 로딩 시 1스텝 기본 데이터 생성
if "latest_data" not in st.session_state:
    frame = st.session_state.sim.step()
    fl, wk = frame["forklift"], frame["worker"]
    curr_risk = risk_engine.calculate_risk(fl, wk)
    fl_traj = world_engine.imagine_rollout(fl, (0.0, 0.0), horizon_steps=25)
    wk_traj = world_engine.imagine_rollout(wk, (0.0, 0.0), horizon_steps=25)
    future_risk = world_engine.evaluate_future_risk(fl_traj, wk_traj, risk_engine)
    st.session_state.latest_data = {
        "timestamp": frame["timestamp"],
        "forklift": fl,
        "worker": wk,
        "fl_traj": fl_traj,
        "wk_traj": wk_traj,
        "curr_risk": curr_risk,
        "future_risk": future_risk,
        "total_cri": curr_risk["cri"],
        "status": "SAFE"
    }
    st.session_state.prev_fl = fl.copy()
    st.session_state.prev_wk = wk.copy()

data = st.session_state.latest_data

# 7. 상단 핵심 지표(KPI) 카드 영역
m1, m2, m3, m4, m5 = st.columns(5)
m1.metric("⏱️ 경과 시간", f"{data['timestamp']:.1f} 초")
m2.metric("📏 현재 거리", f"{data['curr_risk']['distance']:.2f} m")
m3.metric("📐 물리 CRI (현재)", f"{data['curr_risk']['cri']:.3f}")
m4.metric("🧠 월드모델 예측 CRI", f"{data['future_risk']['max_pred_cri']:.3f}")

status = data["status"]
if status == "CRITICAL":
    m5.error(f"🚨 위험 (CRI: {data['total_cri']})")
elif status == "WARNING":
    m5.warning(f"⚠️ 주의 (CRI: {data['total_cri']})")
else:
    m5.success(f"✅ 정상 (CRI: {data['total_cri']})")

if st.session_state.is_finished:
    st.info(f"시나리오가 종료되었습니다 (종료 시각 {data['timestamp']:.1f}초). 다시 보려면 [초기화]를 누르세요.")

# 8. 메인 화면 분할: 왼쪽(Three.js 3D 관제 화면), 오른쪽(경보 이벤트 로그)
col_3d, col_log = st.columns([2.2, 1])

with col_3d:
    st.subheader("🏭 3D 디지털 트윈 안전 관제 뷰포트")
    
    # Python 데이터를 JSON으로 변환하여 Three.js HTML 코드에 주입
    scene_payload = json.dumps({
        "forklift": data["forklift"],
        "worker": data["worker"],
        "fl_traj": data["fl_traj"],
        "wk_traj": data["wk_traj"],
        "status": data["status"],
        "safe_radius": data["curr_risk"]["safe_radius"] # 동적 안전 버블 반경(m)
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
            🟡 <b>지게차 (FL_01)</b> &nbsp;|&nbsp; 🔵 <b>작업자 (WK_01)</b> &nbsp;|&nbsp; 🟣 <b>월드 모델 예측 경로(2.5s)</b><br>
            <span style="color:#94a3b8; font-size:11px;">마우스 드래그: 시점 회전 &nbsp;|&nbsp; 안전 버블 반경: {data['curr_risk']['safe_radius']:.1f} m</span>
        </div>
        <script>
            const payload = {scene_payload};

            // 1. 씬, 카메라, 렌더러 설정
            const scene = new THREE.Scene();
            scene.background = new THREE.Color(0x0f172a);

            const camera = new THREE.PerspectiveCamera(45, window.innerWidth / 460, 0.1, 1000);
            camera.position.set(10, 22, 26);
            camera.lookAt(10, 0, 8);

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
            gridHelper.position.set(10, 0, 10);
            scene.add(gridHelper);

            // 3. 사각지대 유발 물류 선반(랙) 배치 (코너 시야 방해물 표현)
            const rackGeo = new THREE.BoxGeometry(4.5, 2.5, 4.5);
            const rackMat = new THREE.MeshLambertMaterial({{ color: 0x475569, transparent: true, opacity: 0.85 }});
            const rack1 = new THREE.Mesh(rackGeo, rackMat);
            rack1.position.set(9.0, 1.25, 8.5);
            scene.add(rack1);

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

            // 5. 작업자 3D 객체
            const wkMesh = new THREE.Mesh(
                new THREE.CylinderGeometry(0.35, 0.35, 1.7, 16),
                new THREE.MeshLambertMaterial({{ color: 0x38bdf8 }})
            );
            wkMesh.position.set(payload.worker.x, 0.85, payload.worker.y);
            scene.add(wkMesh);

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
            drawTrajectory(payload.wk_traj, 0x67e8f9); // 작업자 미래 예측 궤적 (하늘색)

            // 7. 간단한 마우스 드래그 카메라 회전
            let isDragging = false;
            let prevX = 0;
            let angle = 0.4;
            document.addEventListener('mousedown', e => {{ isDragging = true; prevX = e.clientX; }});
            document.addEventListener('mouseup', () => {{ isDragging = false; }});
            document.addEventListener('mousemove', e => {{
                if (!isDragging) return;
                angle += (e.clientX - prevX) * 0.01;
                prevX = e.clientX;
                camera.position.x = 10 + Math.sin(angle) * 22;
                camera.position.z = 10 + Math.cos(angle) * 22;
                camera.lookAt(10, 0, 9);
            }});

            renderer.render(scene, camera);
        </script>
    </body>
    </html>
    """
    components.html(threejs_html, height=465)

with col_log:
    st.subheader("📋 실시간 위험 경보 이력")
    if len(st.session_state.event_logs) > 0:
        df_logs = pd.DataFrame(st.session_state.event_logs)
        st.dataframe(df_logs, use_container_width=True, height=420, hide_index=True)
    else:
        st.info("현재까지 발생한 주의/위험 경보 이벤트가 없습니다.\n왼쪽에서 [▶️ 자동 재생]을 눌러 시뮬레이션을 시작하세요.")

# 9. 자동 재생 루프 처리 (부드러운 3D 프레임 갱신)
if st.session_state.is_running:
    time.sleep(0.25)
    st.rerun()