# 파이썬이 설치된 가벼운 리눅스 이미지에서 시작한다
FROM python:3.12-slim

# 컨테이너 안에서 작업할 폴더
WORKDIR /app

# 패키지 목록만 먼저 복사해 설치한다 (코드만 고쳤을 때는 이 단계를 다시 하지 않아 빌드가 빨라진다)
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 나머지 코드를 복사한다
COPY . .

# 앱이 쓰는 포트
EXPOSE 8501

# 컨테이너가 시작될 때 실행할 명령
CMD ["streamlit", "run", "app.py", "--server.address=0.0.0.0", "--server.port=8501", "--server.headless=true"]