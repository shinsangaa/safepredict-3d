# 실행 명령어: python sensor_sender.py --scenario retreat
import argparse
import json
import math
import os
import time
from datetime import datetime, timezone

from simulator import WarehouseSimulator, SCENARIO_MAX_TIME, SCENARIO_STOP_DISTANCE
from stream_reader import STREAM_DIR

SEND_INTERVAL_S = 0.1 # 10Hz

def utc_now_text() -> str:
    """현재 UTC 시각을 문서 형식(밀리초, Z)으로 만든다."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

def main():
    parser = argparse.ArgumentParser(description="가상 센서: 지게차·작업자 위치를 10Hz로 내보낸다")
    parser.add_argument("--scenario", default="retreat", choices=list(SCENARIO_MAX_TIME.keys()))
    args = parser.parse_args()

    os.makedirs(STREAM_DIR, exist_ok=True)
    run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S") # 실행마다 새 파일
    path = os.path.join(STREAM_DIR, f"{run_id}.jsonl")

    sim = WarehouseSimulator(scenario=args.scenario)
    max_time = SCENARIO_MAX_TIME[args.scenario]
    stop_distance = SCENARIO_STOP_DISTANCE[args.scenario]

    print(f"송신 시작: {args.scenario} -> {path}")
    start = time.perf_counter()
    tick = 0

    with open(path, "a", encoding="utf-8", newline="\n") as f:
        while True:
            # tick 0은 시작 상태 그대로, 그다음부터는 0.1초씩 진행한 상태를 보낸다
            frame = sim.get_frame() if tick == 0 else sim.step()

            message = {
                "run_id": run_id,
                "scenario": args.scenario,
                "tick": tick,
                "sent_at": utc_now_text(),
                "frame": frame,
            }
            f.write(json.dumps(message) + "\n") # 한 줄 = 한 시점
            f.flush()                           # 바로 파일에 내보내서 수신 쪽이 읽을 수 있게 한다

            # 종료 판정: 최대 실행시간 또는 거리 1m 이하 (scenarios.md 3장)
            fl, wk = frame["forklift"], frame["worker"]
            distance = math.hypot(wk["x"] - fl["x"], wk["y"] - fl["y"])
            if frame["timestamp"] >= max_time:
                break
            if stop_distance is not None and distance <= stop_distance + 1e-6:
                break

            # 다음 틱 시각까지 기다린다 (시작 시각 기준이라 오차가 쌓이지 않는다)
            tick += 1
            wait = start + tick * SEND_INTERVAL_S - time.perf_counter()
            if wait > 0:
                time.sleep(wait)

    print(f"송신 종료: {tick + 1}건, 마지막 시각 {frame['timestamp']:.1f}초")

if __name__ == "__main__":
    main()