import time
import math
import json
from dataclasses import dataclass, asdict

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
    def __init__(self, scenario: str = "collision"):
        """
        시나리오 초기화
        :param scenario: 'safe' (정상 통과) 또는 'collision' (사거리 충돌 위험)
        """
        self.scenario = scenario
        self.dt = 0.1  # 10Hz = 0.1초 간격
        self.current_time = 0.0
        self.reset()

    def reset(self):
        """시나리오에 따른 초기 위치 및 속도 설정"""
        self.current_time = 0.0
        
        if self.scenario == "safe":
            # [시나리오 1: 정상 주행] 서로 멀리 떨어져서 평행하게 지나감
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=0.0, y=2.0, z=0.0, speed=3.0, heading=0.0)
            self.worker = EntityState("WK_01", "worker", 0.0, x=20.0, y=10.0, z=0.0, speed=1.2, heading=math.pi)
            
        elif self.scenario == "collision":
            # [시나리오 2: 교차 충돌 위험] (15, 10) 지점에서 약 5초 후 만나는 경로
            # 지게차: (0, 10)에서 동쪽(0 rad)으로 3.0 m/s 이동 -> 5초 뒤 (15, 10) 도달
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=0.0, y=10.0, z=0.0, speed=3.0, heading=0.0)
            # 작업자: (15, 4)에서 북쪽(pi/2 rad)으로 1.2 m/s 이동 -> 5초 뒤 (15, 10) 도달
            self.worker = EntityState("WK_01", "worker", 0.0, x=15.0, y=4.0, z=0.0, speed=1.2, heading=math.pi / 2)
        elif self.scenario == "corner_turn":
            # [시나리오 3: 선반 코너 급회전 및 사각지대 접근] (월드 모델 예측 검증용)
            # 지게차: 동쪽으로 주행하다가 2.0초 시점에 북쪽(선반 통로)으로 좌회전(조향) 진입
            self.forklift = EntityState("FL_01", "forklift", 0.0, x=6.0, y=4.0, z=0.0, speed=2.8, heading=0.0)
            # 작업자: 통로 안쪽 (12.5, 12.0)에서 남쪽으로 걸어 내려오는 중
            self.worker = EntityState("WK_01", "worker", 0.0, x=12.5, y=12.5, z=0.0, speed=1.1, heading=-math.pi / 2)
        else:
            raise ValueError("지원하지 않는 시나리오입니다. 'safe', 'collision', 'corner_turn' 중 선택하세요.")

    def step(self):
        """0.1초(1프레임) 후의 객체 위치 업데이트 및 상태 반환"""
        self.current_time = round(self.current_time + self.dt, 2)
        
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
        
        return {
            "timestamp": self.current_time,
            "forklift": asdict(self.forklift),
            "worker": asdict(self.worker)
        }

# 직접 실행 시 동작 테스트 (터미널에서 python simulator.py 입력)
if __name__ == "__main__":
    print("=== [STEP 1] 가상 이동 데이터 시뮬레이터 테스트 (충돌 시나리오) ===")
    sim = WarehouseSimulator(scenario="collision")
    
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