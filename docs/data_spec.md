# SafePredict 3D - 데이터 명세서
- 문서명: `data_spec.md`
- 버전: v0.4 (이벤트 로그·틱별 계산 결과 저장 방식 수정)
- 작성일: 2026-11-16
- 수정일: 2026-11-19
- 적용 범위: 지게차 1대·작업자 1명, S01~S06 시뮬레이션

## 1. 목적 및 데이터 흐름
시뮬레이터, 충돌 위험 계산 엔진, Three.js 관제 화면 사이에서 주고받는 데이터의 필드명·자료형·단위·결측 처리 규칙을 통일한다.
```text
시나리오 설정 -> 10Hz 위치·속도 생성 -> 입력 검증 -> TTC·DCPA·CRI 계산 -> 월드 모델 예측·통합 CRI -> 위험 단계 판정 -> 3D 관제 화면 및 이벤트 로그
```
- 기본 객체: `forklift_01`, `worker_01`
- 데이터 생성 간격: 0.1초(10Hz), 한 시점마다 객체별 2개 레코드 생성
- 주의: 10Hz는 시뮬레이션 데이터 생성 및 계산 주기다. Streamlit 화면 전체를 0.1초마다 재실행한다는 의미는 아니다.

## 2. 공통 규약
| 항목 | 규약 |
|---|---|
| 좌표계 | 시뮬레이션 계산은 2D 직교좌표 `(x, y)`, `z=0` |
| 거리·위치 | 미터(`m`) |
| 속도 | 미터/초(`m/s`) |
| 방향 | 라디안(`rad`), +x축 기준 반시계 방향 |
| 시간 | 초(`s`); `simulation_time_s`는 실행 시작부터 경과 시간 |
| 타임스탬프 | `timestamp`는 UTC ISO 8601 문자열(`Z` 포함, 밀리초 포함) |
| 생성 주기 | `simulation_time_s = tick / 10` (`tick`은 0부터 시작) |
| 객체 유형 | `forklift` 또는 `worker` |
| 위험 단계 | `SAFE`, `WARNING`, `CRITICAL` |
| 유효하지 않은 숫자 | `NaN`, `Infinity`, 음수 속력 불허 |
| TTC 없음 | `null`(충돌 예상 없음); 0 또는 -1로 대체하지 않음 |
| DCPA | 현재 속도를 유지한다고 가정했을 때 현재 이후의 최소 중심 간 거리 |

- Three.js 좌표 매핑: 계산 좌표 (x, y, z=0)는 Three.js의 (x, y=0, z)에 각각 (sim_x, 0, sim_y)로 전달한다. 계산상의 y가 화면상의 수평 바닥 방향(높이=0, 바닥 평면 위 이동 방향)이 되도록 매핑을 고정한다.
- Three.js에서는 일반적으로 바닥을 XZ 평면으로 표현한다.
- Three.js에서 Z축은 카메라로부터 화면 안쪽으로 깊어지는 방향을 의미한다.
- => 충돌계산은 XY평면, Three.js 시각화는 XZ평면으로 통일한다.

## 3. 입력 데이터 - 객체 위치 · 속도 (`object_state`)
> 단위: 객체 1개 X 시점 1개 = 1행. 같은 `run_id`, `tick`에 지게차 1행과 작업자 1행이 있어야 한다.

| 필드명 | 자료형 | 필수 | 예시 | 설명·검증 기준 |
|---|---|---|---|---|
| `run_id` | string | Y | `run_001` | 시뮬레이션 실행마다 부여하는 고유 식별자 |
| `scenario_id` | string | Y | `S03` | 시나리오 종류 식별자 (`S01`~`S06`) |
| `tick` | int | Y | `0` | 0부터 시작, 시점마다 1씩 증가 |
| `simulation_time_s` | float | Y | `0.0` | `tick * 0.1` |
| `timestamp` | string | Y | `2026-11-16T00:00:00Z` | 실행 기준 UTC 시간 + 경과 시간 |
| `object_id` | string | Y | `forklift_01` | 객체별 고유 식별자 |
| `object_type` | string | Y | `forklift` | `forklift` / `worker` |
| `x_m` | float | Y | `0.0` | 계산 공간 x 위치(m) |
| `y_m` | float | Y | `0.0` | 계산 공간 y 위치(m) |
| `z_m` | float | Y | `0.0` | 본 PoC에서는 0으로 고정 |
| `vx_mps` | float | Y | `2.0` | x방향 속도 성분(m/s) |
| `vy_mps` | float | Y | `0.0` | y방향 속도 성분(m/s) |
| `speed_mps` | float | Y | `2.0` | `sqrt(vx² + vy²)`에서 계산 |
| `heading_rad` | float/null | Y | `0.0` | 객체가 이동하는 방향을 라디안 단위로 나타낸 값. 이동 중 `atan2(vy_mps, vx_mps)`; 정지 시 `null` |

- (참고) `atan2(y, x)`: 입력값 2개인 역탄젠트 함수. X와 Y 부호를 각각 확인해서 어느 사분면을 향하는지 정확하게 판단가능하다.

### 입력 데이터 예시 (S03, 시작 시점)
```json
[
    {
        "run_id": "run_001", "scenario_id":"S03", "tick": 0,
        "simulation_time_s": 0.0, "timestamp": "2026-11-16T00:00:00Z",
        "object_id": "forklift_01", "object_type": "forklift",
        "x_m": 0.0, "y_m": 0.0, "z_m": 0.0,
        "vx_mps": 2.0, "vy_mps":0.0, "speed_mps": 2.0, "heading_rad": 0.0
    },
    {
        "run_id": "run_001", "scenario_id":"S03", "tick": 0,
        "simulation_time_s": 0.0, "timestamp": "2026-11-16T00:00:00Z",
        "object_id": "worker_01", "object_type": "worker",
        "x_m": 10.0, "y_m": 0.0, "z_m": 0.0,
        "vx_mps": -1.0, "vy_mps":0.0, "speed_mps": 1.0, "heading_rad": 3.141593        
    }
]
```

### 문서-코드 필드명 대응
| 문서 | 코드 |
|---|---|
| `object_id`, `object_type` | `entity_id`, `entity_type` |
| `simulation_time_s` | `timestamp` (경과 초) |
| `x_m`, `y_m`, `z_m` | `x`, `y`, `z` |
| `speed_mps`, `heading_rad` | `speed`, `heading` |
| `vx_mps`, `vy_mps` | 없음 (`speed × cos/sin(heading)`으로 계산) |
| `run_id`, `scenario_id`, `tick`, UTC `timestamp` | 없음 |


## 4. 출력 데이터 - 충돌 위험 계산 결과 (`risk_result`)
> 단위: 동일한 `run_id`, `tick`의 두 객체를 비교한 결과 1행

| 필드명 | 자료형 | 필수 | 예시 | 설명 |
|---|---|---|---|---|
| `run_id` | string | Y | `run_001` | 입력 실행 식별자 |
| `scenario_id` | string | Y | `S03` | 시나리오 ID |
| `tick` | int | Y | `0` | 계산 시점 번호 |
| `simulation_time_s` | float | Y | `0.0` | 시뮬레이션 경과 시간(s) |
| `timestamp` | string | Y | `2026-11-16T00:00:00Z` | 입력과 동일한 기준 시각 |
| `distance_m` | float | Y | `10.0` | 객체 중심 간 현재 거리(m) |
| `relative_speed_mps` | float | Y | `3.0` | 상대속도 **벡터의 크기**(m/s) |
| `closing_speed_mps` | float | Y | `3.0` | 현재 거리의 감소율(m/s), 이탈 시 음수 |
| `ttc_s` | float/null | Y | `3.0` | 두 객체가 충돌 판정 거리(1m)에 처음 진입할 예상 시간(s); 진입하지 않으면 `null` |
| `t_cpa_s` | float | Y | `3.333333` | 최근접 시점까지 남은 시간(Time to Closest Point of Approach)(s) |
| `dcpa_m` | float | Y | `0.0` | 현재 속도와 이동 방향을 유지한다고 가정했을 때, 두 객체가 미래에 가장 가까워지는 거리(m) |
| `safe_radius_m` | float | Y | `4.0` | 동적 안전 버블 반경(m) |
| `cri` | float | Y | `0.60` | 현재 CRI(범위 0~1) |
| `risk_level` | string | Y | `CRITICAL` | 위험 단계 (SAFE, WARNING, CRITICAL). `cri_total` 기준 |
| `collision_predicted` | bool | Y | `true` | 현재 속도 유지 시 충돌거리 1m에 진입할지 여부 |
| `cri_pred` | float | Y | `1.00` | 예측 2.5초 구간의 최대 CRI |
| `cri_total` | float | Y | `0.90` | 통합 CRI, `max(cri, 0.9 × cri_pred)` |
| `min_future_distance_m` | float | Y | `2.5` | 예측 구간의 최소 중심 간 거리(m) |
| `critical_offset_s` | float/null | Y | `2.0` | 예측 CRI가 최대가 되는 시점까지 남은 시간(s), 0.1~2.5; `cri_pred = 0`이면 `null` |

- 코드 대응: `cri_pred` ↔ `max_pred_cri`, `min_future_distance_m` ↔ `min_future_dist`, `critical_offset_s` ↔ `critical_offset`
- `relative_speed_mps`는 상대속도 벡터 크기, `closing_speed_mps`는 서로 가까워지는 속도
- `ttc_s = null`이면 화면에는 `충돌 예상 없음`으로 표시한다.
- 계산 공식, CRI 임계값, 예외 처리는 `risk_rules.md`를 기준으로 한다.
- 현재 단계의 충돌거리 1m, 안전 버블 계산식 및 CRI는 PoC용 초기 설정이며 실제 산업 안전기준이 아니다.

### 위험 계산 결과 예시 (S03, 시작 시점)
```json
{
  "run_id": "run_001", "scenario_id": "S03", "tick": 0,
  "simulation_time_s": 0.0, "timestamp": "2026-11-16T00:00:00Z",
  "distance_m": 10.0, "relative_speed_mps": 3.0,
  "closing_speed_mps": 3.0, "ttc_s": 3.0,
  "t_cpa_s": 3.333333, "dcpa_m": 0.0,
  "safe_radius_m": 4.0, "cri": 0.60,
  "risk_level": "CRITICAL", "collision_predicted": true,
  "cri_pred": 1.00, "cri_total": 0.90,
  "min_future_distance_m": 2.5, "critical_offset_s": 2.0
}
```

### 4.1 예측 궤적 (`predicted_trajectory`)
> 객체 1개당 25행. 화면 표시용이며 저장하지 않는다.

| 필드명 | 자료형 | 설명 |
|---|---|---|
| `step` | int | 1~25 |
| `time_offset_s` | float | 현재로부터의 시간(s), `step × 0.1` |
| `x_m`, `y_m` | float | 예측 위치(m) |
| `speed_mps` | float | 예측 속력(m/s) |
| `heading_rad` | float | 예측 이동 방향(rad) |

## 5. 이벤트 로그 (`risk_event`)
> 위험 상태가 바뀔 때마다 1건 기록한다. 시나리오 시작 직후의 초기 판정도 `INIT`으로 1건 기록한다. 매 0.1초마다 같은 상태를 중복 기록하지 않는다.

| 필드명 | 자료형 | 필수 | 예시 | 설명 |
|---|---|---|---|---|
| `event_id` | string | Y | `evt_0001` | 실행 내 이벤트 고유 ID |
| `run_id` | string | Y | `run_001` | 실행 ID |
| `scenario_id` | string | Y | `S03` | 시나리오 ID |
| `tick` | int | Y | `0` | 이벤트 발생 시점 |
| `simulation_time_s` | float | Y | `0.0` | 발생 시각(s) |
| `timestamp` | string | Y | `2026-11-16T00:00:00Z` | 이벤트 발생 UTC 시각 |
| `event_type` | string | Y | `INIT` | `INIT` 또는 `LEVEL_CHANGE` |
| `previous_level` | string/null | Y | `null` | 이전 위험 단계; 최초 이벤트에서만 `null` |
| `current_level` | string | Y | `CRITICAL` | 변경된 위험 단계 |
| `distance_m` | float | Y | `10.0` | 이벤트 발생 당시 거리 |
| `ttc_s` | float/null | Y | `3.0` | 발생 당시 TTC |
| `dcpa_m` | float | Y | `0.0` | 발생 당시 DCPA |
| `cri` | float | Y | `0.60` | 발생 당시 CRI |
| `rule_version` | string | Y | `v0.2` | 위험 규칙 버전 |
| `cri_pred` | float | Y | `1.00` | 예측 CRI |
| `cri_total` | float | Y | `0.90` | 통합 CRI |
| `trigger` | string | Y | `PREDICTED` | 통합 CRI를 결정한 쪽. `CURRENT`(현재 CRI) 또는 `PREDICTED`(예측 CRI) |

- 로그 완전성 기준: 실제로 발생시킨 `INIT` 및 `LEVEL_CHANGE` 이벤트 수와 저장한 이벤트 레코드 수가 같아야 한다. 시뮬레이션이 종료되는 것 자체는 이벤트로 기록하지 않는다(종료용 `event_type` 없음).

## 6. 시나리오 설정 데이터 (`scenario_config`)

| 필드명 | 자료형 | 설명 |
|---|---|---|
| `scenario_id` | string | `S01`~`S06` |
| `name` | string | 시나리오 이름 |
| `forklift_start_xy` | list[float] | 지게차 초기 `(x, y)` |
| `worker_start_xy` | list[float] | 작업자 초기 `(x, y)` |
| `forklift_velocity_xy` | list[float] | 지게차 속도 `(vx, vy)` |
| `worker_velocity_xy` | list[float] | 작업자 초기 속도 `(vx, vy)` |
| `duration_s` | float | 최대 실행시간 |
| `stop_distance_m` | float/null | 거리 도달 시 종료; 해당 없으면 `null` |
| `velocity_changes` | list | 시각별 속도 변경 목록 (S05에서 사용) |
| `yaw_rate_changes` | list | 구간별 회전 속도 목록 (S06에서 사용) |

| ID | 지게차 시작 좌표 | 작업자 시작 좌표 | 지게차 속도 벡터 | 작업자 초기 속도 벡터 | 최대 시간 | 종료 거리 |
|---|---|---|---|---|---|---|
| S01 | (0, 0) | (10, 8) | (2, 0) | (1, 0) | 10초 | 없음 |
| S02 | (0, 0) | (10, 0) | (0, 0) | (-1, 0) | 10초 | 1m 이하 |
| S03 | (0, 0) | (10, 0) | (2, 0) | (-1, 0) | 5초 | 1m 이하 |
| S04 | (0, 0) | (10, 10) | (2, 0) | (0, -2) | 6초 | 1m 이하 |
| S05 | (0, 0) | (4, 0) | (0, 0) | (-1, 0) | 8초 | 없음 |
| S06 | (6, 4) | (12.5, 12.5) | (2.8, 0) | (0, -1.1) | 6초 | 1m 이하 |

- S05 속도 변경: `simulation_time_s = 2.0`에 작업자 속도를 `(-1, 0)`에서 `(1, 0)`으로 변경한다. `t = 2.0`의 위치는 `(2, 0)`이고, 그 시점의 위험 판정에는 변경된 속도를 적용한다.
- S06 회전: 1.8 ~ 3.3초 동안 지게차 방향을 틱당 약 0.105rad씩 반시계로 변경

## 7. 유효성 검증 및 예외 처리

| 검사 ID | 검증 항목 | 통과 기준 / 처리 방식 |
|---|---|---|
| VAL-01 | 필수 필드 | 누락 필드가 없고 자료형이 일치해야 함 |
| VAL-02 | 객체 페어 | 동일 `run_id`·`tick`에 `forklift_01`, `worker_01`이 각 1개 존재 |
| VAL-03 | 타임스탬프 | UTC ISO 8601 파싱 가능, 시뮬레이션 시간과 대응 |
| VAL-04 | 시간 순서 | 객체별 `tick` 증가, 0.1초 간격 유지(부동소수점 허용 오차 적용) |
| VAL-05 | 위치·속도 | 좌표·속도 성분이 유한한 숫자, `z_m=0` |
| VAL-06 | 파생 필드 일치 | `speed_mps ≈ sqrt(vx²+vy²)`; 이동 중 방향과 속도 벡터 일치 |
| VAL-07 | 위험 점수 | `cri`, `cri_pred`, `cri_total` 모두 0~1, 단계 임계값 일치 |
| VAL-08 | TTC 예외 | 충돌 예상 없으면 `null`; 이미 충돌거리 이내면 `0.0` |
| VAL-09 | 재현성 | 동일한 시나리오 설정·규칙 버전으로 동일 결과 재생성 |
| VAL-10 | 이벤트 로그 | 초기 판정 및 위험 단계 변경당 로그 1개, 중복 기록 없음 |

- 입력 데이터가 유효하지 않으면 해당 프레임을 계산하지 않고 검증 오류를 반환한다. 임의로 0을 대입하지 않는다.
- 허용 오차는 수치 비교 시 적용한다. (초기 권장값: 1e-6)
- 객체가 충돌거리(1m)에 들어온 시점은 `ttc_s=0.0`으로 처리한다.
- 상대속도 벡터 크기가 0이고 현재 충돌거리 바깥이면 `ttc_s = null`, `t_cpa_s = 0`, `dcpa_m = distance_m`

(부가설명)
- 허용 오차 1e-6 = 0.000001 설정
```python
EPSILON = 1e-6

a = 0.1 + 0.2
b = 0.3

if abs(a - b) < EPSILON:
    print("두 값은 사실상 동일")
```
- 상대속도가 0이면 dcpa_m = distance_m
- -> 두 객체 사이의 거리가 변하지 않기 때문이다.

## 8. 파일 저장 및 책임 모듈 (초안)

| 데이터 | 권장 위치 | 생성·사용 모듈 |
|---|---|---|
| 시나리오 설정 | `data/scenarios.json` | 시뮬레이터 입력 |
| 위치·속도 데이터 | 실행 중 메모리; 테스트 시 `data/sample/object_states.csv` | 시뮬레이터 → 검증 모듈 |
| 위험 계산 결과 | 실행 세션에 틱별 보관, CSV 다운로드 (`risk_results_S01_run_001.csv` 형식) | 위험 엔진 → 대시보드 |
| 이벤트 기록 | 실행 세션에 보관, CSV 다운로드 (`events_S01_run_001.csv` 형식) | 위험 판정 → 대시보드 |
| 월드 모델 가중치 | `models/world_model_weights.pt` (산출물 보관용. 앱은 시작 시 학습 후 메모리에 보관) | `save_weights.py`(파일 저장), `world_model.py` → 대시보드 |

- Streamlit Cloud: 로컬 파일에 저장한 이벤트 이력을 영구 저장소로 간주하지 않는다. PoC에서는 실행 세션 내 기록과 다운로드 기능을 우선한다.
- 실제 DB, 메시지 브로커 및 FastAPI 도입은 이번 기본 범위에서 필수가 아니다.
- (데이터베이스 구축 시 개발 범위 커져서 우선 다음 방식으로 구현함)
```text
경보 발생 -> Streamlit 세션에 기록 -> 대시보드에 표시 -> CSV 다운로드
```
- CSV는 UTF-8(BOM) 인코딩이며, null 값은 빈 칸으로 저장한다.

## 9. 관련 문서

- `requirements.md` : 기능 · 비기능 요구사항
- `scenarios.md` : 6종 시나리오 상세 이동 · 종료 조건, 수동 검산값
- `risk_rules.md` : TTC · DCPA · CRI 공식, 위험 단계 및 경보 규칙

## 10. 변경 이력

| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| v0.1 | 2026-11-16 | 최초 작성 |
| v0.2 | 2026-11-17 | `risk_result`에 예측·통합 CRI 필드 4개 추가, 예측 궤적(4.1) 절 추가, `risk_event`에 필드 3개 추가, S06 설정 추가, 문서-코드 필드명 대응표 추가 |
| v0.3 | 2026-11-18 | 월드 모델 가중치 파일 저장 위치 추가 (models/world_model_weights.pt) |
| v0.4 | 2026-11-19 | 이벤트 로그·틱별 계산 결과의 저장 방식을 세션 보관 + CSV 다운로드로 수정 |