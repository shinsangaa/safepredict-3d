# 실행 명령어(수신 확인용): python stream_reader.py
import glob
import json
import os
import time
from datetime import datetime, timezone

# 송신 프로그램과 수신 쪽이 함께 쓰는 폴더. 환경 변수 STREAM_DIR이 있으면 그 값을 쓴다 (Docker에서 사용)
STREAM_DIR = os.environ.get("STREAM_DIR", "stream")

class StreamReader:
    """송신 프로그램이 파일에 한 줄씩 쌓는 위치 데이터를, 새로 들어온 만큼만 읽어 온다."""

    def __init__(self, folder: str = STREAM_DIR, skip_existing: bool = True):
        self.folder = folder
        self.current_file = None # 지금 읽고 있는 파일
        self.offset = 0          # 그 파일에서 이미 읽은 위치(바이트)

        # 이미 쌓여 있는 예전 데이터는 건너뛰고, 지금부터 들어오는 것만 읽는다
        if skip_existing:
            files = self._list_files()
            if files:
                self.current_file = files[-1]
                self.offset = os.path.getsize(files[-1])

    def _list_files(self) -> list:
        # 파일 이름에 시작 시각이 들어 있어서, 이름순 정렬이 곧 시간순이다
        return sorted(glob.glob(os.path.join(self.folder, "run_*.jsonl")))

    def read_new(self) -> list:
        """지난번 호출 이후 새로 들어온 메시지 목록을 돌려준다. 없으면 빈 목록."""
        files = self._list_files()
        if not files:
            return []

        latest = files[-1]
        if latest != self.current_file: # 송신이 새로 시작됨 -> 새 파일을 처음부터 읽는다
            self.current_file = latest
            self.offset = 0

        with open(latest, "rb") as f:
            f.seek(self.offset)         # 이미 읽은 곳 다음으로 이동
            chunk = f.read()

        end = chunk.rfind(b"\n")
        if end == -1:
            return []                   # 완성된 줄이 아직 없음
        complete = chunk[:end + 1]      # 줄바꿈으로 끝난 부분까지만 사용 (쓰는 중인 줄은 다음에 읽는다)
        self.offset += len(complete)

        messages = []
        for line in complete.splitlines():
            try:
                messages.append(json.loads(line))
            except json.JSONDecodeError:
                continue                # 깨진 줄은 건너뛴다
        return messages

# 직접 실행 시: 들어오는 데이터를 터미널에 출력 (수신 확인용)
if __name__ == "__main__":
    reader = StreamReader()
    print(f"수신 대기 중... 폴더: {STREAM_DIR} (Ctrl+C로 종료)")
    try:
        while True:
            for msg in reader.read_new():
                sent_at = datetime.fromisoformat(msg["sent_at"].replace("Z", "+00:00"))
                delay_ms = (datetime.now(timezone.utc) - sent_at).total_seconds() * 1000
                fl = msg["frame"]["forklift"]
                wk = msg["frame"]["worker"]
                print(f"[{msg['run_id']}] {msg['scenario']} tick {msg['tick']:3d} | "
                      f"지게차 ({fl['x']:5.2f}, {fl['y']:5.2f}) | "
                      f"작업자 ({wk['x']:5.2f}, {wk['y']:5.2f}) | 지연 {delay_ms:4.0f}ms")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("수신 종료")