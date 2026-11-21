import time
import math
import json
from dataclasses import dataclass, asdict

# 시나리오별 최대 실행시간(초) - scenarios.md 3장
SCENARIO_MAX_TIME = {
    "parallel": 10.0,
    "stop_approach": 10.0,
    "head_on": 10.0,
    "crossing": 12.0,
    "retreat": 8.0,
    "corner_turn": 6.0,
}

# 시나리오별 종료 거리(m) - scenarios.md 3장. None이면 거리로는 종료하지 않는다.
SCENARIO_STOP_DISTANCE = {
    "parallel": None,       # S01
    "stop_approach": 1.0,   # S02
    "head_on": 1.0,         # S03
    "crossing": 1.0,        # S04
    "retreat": None,        # S05
    "corner_turn": 1.0,     # S06
}

# 가상 자동 대응 설정 (시뮬레이터 안에서만 동작하며, 실제 장비를 제어하지 않는다)
WARNING_SPEED_LIMIT = 1.0 # 주의 단계: 지게차를 이 속력까지 낮춘다 (m/s)
BRAKE_DECEL = 1.5         # 지게차 감속도 (m/s²)
WORKER_STOP_DECEL = 2.0   # 작업자가 멈출 때의 감속도 (m/s²)

# 객체(지게차, 작업자)의 현재 상태를 담는 데이터 구조 정의
@dataclass
class EntityState:
    entity_id: str      # 객체 고유 ID (예: FORKLIFT_01, WORKER_01)
    entity_type: str    # 객체 유형 ('forklift' 또는 'worker')
    timestamp: float    # 현재 시간 (초)
    x: float            # X 좌표 (미터)
    y: float            # Y 좌표 (미터)
    z: float            # Z 좌표 (높이, 미터 - 평면 기준 0.0)
    speed: float        # 이동 속력 (m/s)
    heading: float      # 이동 방향 각도 (라디안, 0은 동쪽/+X, pi/2는 북쪽/+Y)

class WarehouseSimulator:
    """
    물류창고 내 지게차와 작업자의 움직임을 10Hz(0.1초 간격)로 생성하는 시뮬레이터
    """
    def __init__(self, scenario: str = "crossing"):
        """시나리오 초기화"""
        self.scenario = scenario
        self.dt = 0.1  # 10Hz = 0.1초 간격
        self.current_time = 0.0
        self.reset()

    def reset(self):
        """시나리오에 따른 초기 위치 및 속도 설정"""
        self.current_time = 0.0
        self.response_state = "NONE" # 자동 대응 상태: NONE(없음) / SLOW(서행) / STOP(비상 정지)
        if self.scenario == "parallel": # S01
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=0.0, y=0.0, z=0.0, speed=2.0, heading=0.0)
            self.worker = EntityState("WK_01", "worker", 0.0, x=10.0, y=8.0, z=0.0, speed=1.0, heading=0.0)
        elif self.scenario == "stop_approach": # S02
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=0.0, y=0.0, z=0.0, speed=0.0, heading=0.0)
            self.worker = EntityState("WK_01", "worker", 0.0, x=10.0, y=0.0, z=0.0, speed=1.0, heading=math.pi)
        elif self.scenario == "head_on": #S03
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=0.0, y=0.0, z=0.0, speed=2.0, heading=0.0)
            self.worker = EntityState("WK_01", "worker", 0.0, x=28.0, y=0.0, z=0.0, speed=1.0, heading=math.pi)
        elif self.scenario == "crossing": #S04
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=0.0, y=0.0, z=0.0, speed=2.0, heading=0.0)
            self.worker = EntityState("WK_01", "worker", 0.0, x=20.0, y=10.0, z=0.0, speed=1.0, heading=-math.pi / 2)
        elif self.scenario == "retreat": #S05
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=0.0, y=0.0, z=0.0, speed=0.0, heading=0.0)
            self.worker = EntityState("WK_01", "worker", 0.0, x=4.0, y=0.0, z=0.0, speed=1.0, heading=math.pi)         
        elif self.scenario == "corner_turn": # S06 선반 코너 급회전 및 사각지대 접근 (월드 모델 예측 검증용)
            # 지게차: 동쪽으로 주행하다가 2.0초 시점에 북쪽(선반 통로)으로 좌회전(조향) 진입
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=6.0, y=4.0, z=0.0, speed=2.8, heading=0.0)
            # 작업자: 통로 안쪽 (12.5, 12.0)에서 남쪽으로 걸어 내려오는 중
            self.worker = EntityState("WK_01", "worker", 0.0, x=12.5, y=12.5, z=0.0, speed=1.1, heading=-math.pi / 2)
        else:
            raise ValueError("지원하지 않는 시나리오입니다. 'parallel', 'stop_approach', 'head_on', 'crossing', 'retreat', 'corner_turn' 중 선택하세요.")

    def get_frame(self):
        """시간을 진행하지 않고 현재 상태만 반환 (tick 0 초기 판정용)"""
        return {
            "timestamp": self.current_time,
            "forklift": asdict(self.forklift),
            "worker": asdict(self.worker)
        }

    def _slow_down(self, entity, target_speed, decel):
        """객체의 속력을 target_speed까지 한 틱만큼 줄인다"""
        if entity.speed > target_speed:
            entity.speed = round(max(target_speed, entity.speed - decel * self.dt), 4)

    def is_stopped(self):
        """비상 정지가 걸렸고 지게차와 작업자가 모두 멈췄으면 True"""
        return self.response_state == "STOP" and self.forklift.speed == 0.0 and self.worker.speed == 0.0

    def step(self, response_level=None):
        """0.1초(1프레임) 후의 객체 위치 업데이트 및 상태 반환"""
        self.current_time = round(self.current_time + self.dt, 2)

        # 가상 자동 대응: 직전 판정 결과(response_level)에 따라 서행 또는 비상 정지
        if response_level == "CRITICAL":
            self.response_state = "STOP"
        elif response_level == "WARNING" and self.response_state == "NONE":
            self.response_state = "SLOW"

        if self.response_state == "STOP":
            self._slow_down(self.forklift, 0.0, BRAKE_DECEL)
            self._slow_down(self.worker, 0.0, WORKER_STOP_DECEL)
        elif self.response_state == "SLOW":
            self._slow_down(self.forklift, WARNING_SPEED_LIMIT, BRAKE_DECEL)
        
        # corner_turn 시나리오일 경우 1.8초~3.3초 사이에 지게차가 좌회전(각속도 적용)
        if self.scenario == "corner_turn" and 1.8 <= self.current_time <= 3.3:
            yaw_rate = (math.pi / 2) / 1.5  # 1.5초 동안 90도 회전
            self.forklift.heading += yaw_rate * self.dt
        
        # 지게차 위치 업데이트 (x = x + v * cos(theta) * dt)
        self.forklift.x += self.forklift.speed * math.cos(self.forklift.heading) * self.dt
        self.forklift.y += self.forklift.speed * math.sin(self.forklift.heading) * self.dt
        self.forklift.timestamp = self.current_time
        
        # 작업자 위치 업데이트
        self.worker.x += self.worker.speed * math.cos(self.worker.heading) * self.dt
        self.worker.y += self.worker.speed * math.sin(self.worker.heading) * self.dt
        self.worker.timestamp = self.current_time

        # S05: 2.0초 시점부터 작업자가 +x 방향으로 돌아선다
        if self.scenario == "retreat" and self.current_time >= 2.0:
            self.worker.heading = 0.0
        
        return {
            "timestamp": self.current_time,
            "forklift": asdict(self.forklift),
            "worker": asdict(self.worker)
        }

# 직접 실행 시 동작 테스트 (터미널에서 python simulator.py 입력)
if __name__ == "__main__":
    print("=== [STEP 1] 가상 이동 데이터 시뮬레이터 테스트 (충돌 시나리오) ===")
    sim = WarehouseSimulator(scenario="crossing")
    
    # 6초(60프레임) 동안 데이터 생성 출력
    for _ in range(60):
        frame_data = sim.step()
        fl = frame_data["forklift"]
        wk = frame_data["worker"]
        
        # 0.5초마다 콘솔에 좌표 출력
        if int(frame_data["timestamp"] * 10) % 5 == 0:
            print(f"[{frame_data['timestamp']:4.1f}s] "
                  f"지게차: ({fl['x']:5.2f}, {fl['y']:5.2f}) | "
                  f"작업자: ({wk['x']:5.2f}, {wk['y']:5.2f})")
        time.sleep(0.05)  # 실제 10Hz보다 2배 빠르게 테스트 출력