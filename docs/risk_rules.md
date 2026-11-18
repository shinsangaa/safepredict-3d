# SafePredict 3D - 충돌 위험 계산 및 판정 규칙 명세서
- 문서명: `risk_rules.md`
- 버전: v0.2 (예측 CRI·통합 CRI 추가)
- 작성일: 2026-11-16
- 수정일: 2026-11-18
- 적용 범위: 지게차 1대·작업자 1명, S01~S06, 10Hz 가상 시뮬레이션
- 규칙 버전(`rule_version`): v0.2

## 1. 목적 및 적용 범위
두 객체의 현재 위치·속도를 사용하여 거리, 상대속도, 접근 속도, TTC, 최근접 시점·거리, 동적 안전 버블, CRI를 계산하고, SAFE/WARNING/CRITICAL의 3단계 위험 상태를 판정한다.
- 입력: 동일한 `run_id`와 `tick`을 갖는 지게차·작업자 `object_state` 2건
- 출력: 같은 시점에 대응하는 `risk_result` 1건
- 이벤트: 최초 판정(`INIT`)과 위험 단계 변경(`LEVEL_CHANGE`) 시에만 `risk_event` 기록
- 기본 가정: 2D 평면에서 두 객체의 중심점을 사용하며, 현재 속도 벡터가 앞으로도 일정하게 유지된다고 가정해 미래를 예측한다. 단, 예측 CRI(7.4)는 월드 모델이 추정한 가속·회전을 반영한다.
- 제외: 실제 장비 크기, 제동 거리, 가속도·회전 반경, 시야 가림, 실제 센서·PLC 제어 및 산업 안전 인증 기준
> 아래 거리 기준과 CRI 공식은 시연·단위 테스트를 위한 임시 규칙이다. 실제 현장 안전 제어에 사용하지 않는다.

## 2. 공통 변수 및 초기 설정
| 변수 | 값 / 정의 | 설명 |
|---|---|---|
| `DT_S` | `0.1` | 10Hz, 시뮬레이션 계산 간격(s) |
| `COLLISION_RADIUS_M` | `1.0` | 두 중심점 간 거리가 1m 이하이면 충돌 판정 구역 안으로 간주 |
| `SAFE_RADIUS_BASE_M` | `3.0` | 정지한 지게차의 시연용 안전 버블 기본 반경(m) |
| `SAFE_RADIUS_SPEED_FACTOR_S` | `0.5` | 지게차 속력에 따른 버블 반경 증가 계수(s) |
| `TTC_SCORE_START_S` | `6.0` | TTC 위험 점수가 0에서 증가하기 시작하는 구간의 상한(s) |
| `TTC_SCORE_FULL_S` | `1.0` | TTC 위험 점수가 1이 되는 시간(s) |
| `WARNING_THRESHOLD` | `0.50` | 주의 단계 시작 CRI |
| `CRITICAL_THRESHOLD` | `0.85` | 위험 단계 시작 CRI |
| `EPSILON` | `1e-6` | 속도 0 판단 및 기본 수치 비교에 쓰는 작은 허용 오차(단위에 유의) |
| `PRED_HORIZON_STEPS` | `25` | 월드 모델이 예측하는 미래 스텝 수. 0.1초 간격이므로 2.5초 |
| `PRED_ALPHA` | `0.9` | 통합 CRI 계산 시 예측 CRI에 곱하는 계수. 예측이 현재 측정보다 불확실하므로 1보다 작게 둠 |

좌표 및 입력 단위
- 입력 좌표: `(x_m, y_m)`(m), 속도 성분: `(vx_mps, vy_mps)`(m/s)
- 속력: `speed_mps = sqrt(vx_mps^2 + vy_mps^2)`
- 시간: `simulation_time_s = tick x 0.1`(s)
- 충돌 위험 계산은 2D `(x, y)` 평면에서 수행한다. Three.js에는 `(sim_x, 0, sim_y)`로 전달한다.
- `None`은 Python의 값이며, JSON 응답에서는 `null`로 변환한다. 예측 가능한 TTC가 없을 때 사용한다.

(부가설명)
- `TTC_SCORE_START_S`: 충돌까지 남은 시간이 6초 미만일 때부터 TTC 위험 점수를 증가시키도록 설정한 초기 기준값이다.
    - 충돌 직전인 1~2초부터 위험도를 높이는 것보다, 6초 전부터 점진적으로 위험도를 증가시킨다.
    - $$S_{TTC} = clip(\frac{6-TTC}{5},0,1)$$
    - 충돌 예상 시간이 감소함에 따라 위험도가 점진적으로 증가하도록 설계한 PoC용 초기 설정값이며, 실제 산업안전 기준을 의미하지 않는다. 시나리오 검증 결과에 따라 조정할 수 있다.

## 3. 거리·상대 속도·접근 속도
지게차(forklift)를 `f`, 작업자(worker)를 `w`로 표시한다.
### 상대 위치 벡터
```text
r = (x_w - x_f, y_w - y_f)
```
### 상대 속도 벡터
```text
v = (vx_w - vx_f, vy_w - vy_f)
```
### 현재 중심 간 거리
```text
distance_m = sqrt(r_x² + r_y²)
```
### 상대 속도 벡터 크기
```text
relative_speed_mps = sqrt(v_x² + v_y²)
```
### 접근 속도 (현재 거리의 감소 속도)
```text
closing_speed_mps = -(r_x × v_x + r_y × v_y) / distance_m
```
- `closing_speed_mps > 0`: 현재 두 객체가 가까워짐
- `closing_speed_mps < 0`: 현재 두 객체가 멀어짐
- `closing_speed_mps = 0`: 현재 거리 변화율이 0
- `distance_m <= EPSILON`이면 0으로 나누지 않도록 `closing_speed_mps = 0.0`으로 처리한다. 이 경우 중심점이 사실상 겹친 상태이므로 별도 충돌 구역 판정을 우선한다.
- 주의: `closing_speed_mps`만으로 교차 이동의 TTC를 계산하지 않는다. TTC는 2D 상대 위치·속도 전체를 사용한다.

## 4. 최근접 시점 `t_cpa_s` 및 예상 최근접 거리 `dcpa_m`
### 정의
- `t_cpa_s`: 현재 속도를 유지할 때 현재 이후 두 중심점 사이 거리가 최소가 되는 시점까지의 시간(s)
- `dcpa_m`: 그때의 예상 최근접 거리(m)
- 상대속도 크기가 `EPSILON`보다 크면 다음과 같이 계산한다.
$$t\_cpa\_s = max(0, -\frac{r·v}{v·v})$$
$$dcpa\_m = |r + v * t\_cpa\_s|$$
- 이미 서로 멀어지는 중이면 수학적인 최근접 시점이 과거일 수 있다. 이때 미래 구간만 고려하여 `t_cpa_s = 0`, `dcpa_m = distance_m`로 처리한다.
- `relative_speed_mps <= EPSILON`이면 상대적으로 거의 정지한 상태로 근사한다. 이때 `t_cpa_s = 0`, `dcpa_m = distance_m`로 처리한다.
- `dcpa_m`은 항상 0 이상의 유한한 숫자로 반환한다.
- 미래 방향 전환이 예정되어 있더라도, 현재 시점의 계산값에는 아직 일어나지 않은 전환을 미리 반영하지 않는다(현재 CRI 기준, 예측 CRI는 7.4 참조). 실제로 속도가 바뀐 틱부터 재계산한다.

## 5. TTC `ttc_s` 및 `collision_predicted`
### 5.1 TTC 정의
`ttc_s`는 현재 위치와 속도가 유지될 때, 두 객체 중심 간 거리가 처음으로 1m 이하가 되기까지 걸리는 시간(s)이다.
- 이미 `distance_m <= COLLISION_RADIUS_M`이면 `ttc_s = 0.0`
- 앞으로 1m 이내 진입이 없으면 `ttc_s = None` (JSON에서는 `null`)
- 중심점이 완전히 일치하는 시각과 1m 구역에 진입하는 시각은 다르다. 이번 문서는 후자로 통일한다.

### 5.2 계산식
상대 위치 `r`, 상대 속도 `v`, 충돌 판정 거리 `R = 1.0`에 대해 다음 방정식을 푼다.
```text
||r + v × t||² = R²

a = v · v
b = 2 × (r · v)
c = (r · r) - R²

a × t² + b × t + c = 0
D = b² - 4ac
```
- 처리 순서
    - `c <= 0` : 이미 충돌 판정 구역 안이므로 `ttc_s = 0.0`
    - `relative_speed_mps <= EPSILON`: 구역 밖에서 상대 이동이 거의 없으므로 `ttc_s = None`
    - 판별식 `D < 0`: 충돌 판정 거리 안에 들어오지 않으므로 `ttc_s = None`
    - `D >= 0`: 두 근을 구해 0 이상의 가장 작은 근을 `ttc_s`로 선택한다. 양의 근이 없으면 `None`
    - 판별식이 아주 작은 음수로 나온 경우에 한해 계산 오차인지 확인하고 0으로 보정할 수 있다. 이때는 무조건적인 보정 대신 입력값 크기에 맞는 허용 오차를 적용한다.

### 5.3 충돌 예측 플래그
```text
collision_predicted = (ttc_s is not None)
```
- 미래 시간 제한 없이, 현재 속도가 유지될 때 언젠가 1m 구역에 진입할지를 뜻한다.
- 이미 1m 구역 안인 경우에도 `ttc_s = 0.0`, `collision_predicted = true`로 처리한다.
- 이 값은 '실제로 향후 3초 이내 충돌했는가'라는 정답 라벨과 다르다. 방향 전환이 있는 S05에서 특히 차이가 발생한다.
- 향후 3초 이내 실제 충돌 정답이 필요해지는 경우, 별도 평가 라벨인 `collision_within_3s_gt`로 관리한다. 이번 필수 기능에는 포함하지 않는다.

## 6. 동적 안전 버블 `safe_radius_m`
지게차의 속력에 따라 커지는 원형 안전 버블을 사용한다.
```text
safe_radius_m = 3.0 + 0.5 × forklift.speed_mps
```
- 위 반경은 객체 중심 간 거리와 비교하며 3D 화면에서 지게차 주변의 시연용 원형 경계로 표시한다.
- 실제 지게차의 외형 치수나 법정 안전거리를 의미하지 않는다.
- 안전 버블 반경과 충돌 판정 거리(1m)는 서로 다른 기준이다.

## 7. CRI 계산 및 위험 단계 판정
### 7.1 TTC 위험 점수 `s_ttc`
```text
if ttc_s is None:
    s_ttc = 0.0
else:
    s_ttc = clip((6.0 - ttc_s) / 5.0, 0.0, 1.0)
```

### 7.2 거리 위험 점수 `s_distance`
```text
s_distance = clip(
    (safe_radius_m - distance_m) / (safe_radius_m - 1.0),
    0.0,
    1.0
)
```
- 두 중심 간 거리가 안전 버블 바깥이면 0에 가까워진다.
- 거리가 1m 이하이면 1이 된다.
- 현재 설정에서 `safe_radius_m >= 3m`이므로 분모는 항상 양수다.

### 7.3 현재 CRI
```text
cri = max(s_ttc, s_distance)
```
- 현재 `cri`는 0~1의 규칙 기반 위험 점수이며, 충돌 확률을 의미하지 않는다.
- `dcpa_m`과 `t_cpa_s`는 별도 예측·설명 지표로 기록한다. 현재 CRI 공식에는 직접 포함하지 않는다.
- 접근하지 않고 이탈 중이더라도 가까이 있으면 거리 점수 때문에 경보가 잠시 유지될 수 있다. 이는 이번 초기 규칙의 의도된 동작이다.

### 7.4 예측 CRI `cri_pred`
행동 추정 (직전 틱 대비, 첫 틱은 둘 다 0)
```text
accel    = (speed_now - speed_prev) / 0.1
yaw_rate = (heading_now - heading_prev) / 0.1    # -π~π로 정규화
```
롤아웃: 월드 모델에 현재 상태와 추정 행동을 넣어 0.1초 뒤 상태를 예측하고,
그 결과를 다시 입력해 25회(2.5초) 반복한다.
```text
cri_pred = 예측 25개 시점 각각에 7.1~7.3을 적용한 값 중 최댓값
```
- 롤아웃 중 가속은 0.5배로, 회전 속도는 20스텝에 걸쳐 0까지 줄여서 입력한다(코드 기준).

### 7.5 통합 CRI `cri_total`
```text
cri_total = max(cri, 0.9 × cri_pred)
```
- 위험 단계는 `cri_total`로 판정한다.
- 0.9는 예측이 현재 측정보다 불확실하다는 점을 반영한 계수다.

### 7.6 위험 단계 및 화면 동작
| `cri_total` 범위 | `risk_level` | 화면·이벤트 동작 |
|---|---|---|
| `cri_total < 0.50` | `SAFE` | 정상 표시; 기존 경보가 해제되면 상태 변경 기록 |
| `0.50 <= cri_total < 0.85` | `WARNING` | 주의 표시, 진입·변경 시 이벤트 기록 |
| `cri_total >= 0.85` | `CRITICAL` | 위험 표시, 진입·변경 시 이벤트 기록 |

- 정확히 0.50이면 WARNING, 정확히 0.85이면 CRITICAL이다.
- 임계값의 별도 히스테리시스, 경보 유지시간, 알림음은 v0.2에서도 적용하지 않는다. 필요할 경우 이후 버전에서 추가한다.

## 8. 이벤트 기록 규칙
한 실행(`run_id`)에서 `tick` 순서대로 위험 판정을 진행한다.
1. `tick = 0`: 최초 위험 단계를 `INIT` 1건으로 기록한다(`previous_level = null`). 최초 단계가 SAFE라도 기록한다.
2. 다음 틱부터: 이전 `risk_level`과 현재 값이 다를 때만 `LEVEL_CHANGE` 1건을 기록한다.
3. 같은 단계가 계속 유지되면 중복 기록하지 않는다.
4. `WARNING → SAFE`, `CRITICAL → WARNING`, `CRITICAL → SAFE`도 단계 변경으로 기록한다.
5. 새 시뮬레이션을 실행하면 새로운 `run_id`를 발급하고 판정·이벤트 상태를 초기화한다.
- 저장 필드: `event_id`, `run_id`, `scenario_id`, `tick`, `simulation_time_s`, `timestamp`, `event_type`, `previous_level`, `current_level`, `distance_m`, `ttc_s`, `dcpa_m`, `cri`, `rule_version`, `cri_pred`, `cri_total`, `trigger`(`CURRENT`/`PREDICTED`)
- `trigger`: 통합 CRI를 결정한 쪽. `cri >= 0.9 × cri_pred`이면 `CURRENT`, 아니면 `PREDICTED`
- 정합성 기준: 발생시킨 INIT·LEVEL_CHANGE 이벤트 수 = 저장한 이벤트 건수. 매 틱의 `risk_result`는 이벤트 로그와 별도로 관리한다.
- 저장 방식: PoC에서는 Streamlit 실행 세션에 로그를 보관하고 CSV 다운로드를 우선 구현한다. 영구 DB는 필수가 아니다.

## 9. S01~S06 초기 수동 검산
> 아래 수치는 `simulation_time_s = 0`의 현재 위치·현재 속도로 계산한다. TTC는 중심 간 1m 구역 진입 기준이며, DCPA는 미래 최소 중심 간 거리다.

| ID | 초기 `distance_m` | 초기 `relative_speed_mps` | 초기 `closing_speed_mps` | 초기 `t_cpa_s` | 초기 `dcpa_m` | 초기 `ttc_s` | 초기 `cri` | 초기 단계 | 초기 `cri_total` | 초기 단계(통합) |
|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| S01 정상 이동 | 약 12.81m | 1.00m/s | 약 0.78m/s | 10.00초 | 8.00m | 없음 | 0.00 | SAFE | 0.00 | SAFE |
| S02 정지 접근 | 10.00m | 1.00m/s | 1.00m/s | 10.00초 | 0.00m | 9.00초 | 0.00 | SAFE | 0.00 | SAFE |
| S03 정면 접근 | 10.00m | 3.00m/s | 3.00m/s | 약 3.33초 | 0.00m | 3.00초 | 0.60 | WARNING | 약 0.90 | CRITICAL |
| S04 교차 이동 | 약 14.14m | 약 2.83m/s | 약 2.83m/s | 5.00초 | 0.00m | 약 4.65초 | 약 0.27 | SAFE | 약 0.69 | WARNING |
| S05 접근 후 이탈 | 4.00m | 1.00m/s | 1.00m/s | 4.00초 | 0.00m | 3.00초* | 0.60 | WARNING | 약 0.90 | CRITICAL |
| S06 코너 회전 | 약 10.70m | 약 3.01m/s | 약 2.57m/s | 약 3.04초 | 약 5.53m | 없음 | 0.00 | SAFE | 0.00 | SAFE |

- `cri_total` 값은 월드 모델이 등속 직선을 정확히 예측한다고 가정한 기대값이다(첫 틱은 추정 행동이 0). 실제 신경망 출력은 약간 다를 수 있다.
- S05의 초기 TTC는 작업자가 방향을 바꾸지 않는다고 가정했을 때의 값이다. 실제 S05는 2초에 방향을 반대로 바꾸므로 1m 구역에 진입하지 않는다.

### 시나리오별 추가 기대 동작
S01~S05 설명은 현재 CRI(`cri`) 기준이다. 통합 기준 초기 단계는 위 표를 따른다.
- S01: 10초 동안 SAFE 유지, INIT 이외 경보 상태 변경 없음. 가장 가까운 거리는 8m
- S02: 정지 지게차를 향해 이동하며 TTC 감소; 거리 1m 이하인 9.0초에 종료
- S03: 시작 시 WARNING, 접근 중 CRITICAL로 상승; 거리 1m 이하인 3.0초에 종료
- S04: 서로 수직인 경로에서 충돌 구역 진입 예상 시간이 약 4.6464초; 10Hz 기록 기준 4.7초에 최초 거리 1m 이하로 종료
- S05: 처음에는 접근하여 위험도가 상승한다. 2.0초 시점의 위치는 (2, 0)이며, 이 틱부터 변경된 작업자 속도 (1, 0)으로 판정한다. 이때 `ttc_s = None`, `t_cpa_s = 0`, `dcpa_m = 2m`, `cri = 0.50`으로 `WARNING`이 된다. 2.1초에는 거리 2.1m, CRI 0.45로 `SAFE`가 되어 경보가 해제된다. 8초에 종료한다.
- S06: 현재 CRI는 3.0초까지 SAFE다(TTC 없음, 거리가 안전 버블 4.4m 밖). 회전으로 지게차 진행 방향이 작업자 쪽을 향하는 3.1초에 `ttc_s ≈ 0.59`, `cri = 1.00`이 되어 WARNING 없이 CRITICAL로 직행한다. 3.6초에 거리 약 0.91m로 1m 이하가 되어 종료한다. 통합 CRI 기준으로는 예측 궤적이 회전을 반영하므로 3.1초보다 먼저 경보가 발생해야 한다(WARNING 1.8초, CRITICAL 2.2초. 학습 횟수 300, 시드 42 기준).
> CRI의 구간별 판정이 해당 시나리오의 예상 변화를 따르는지 테스트한다. 수동 검산은 위 규칙 버전 v0.2에 대한 값이며, 공식을 변경하면 예상값도 함께 갱신한다.

## 10. 예외 처리 및 단위 테스트

| 테스트 ID | 입력 / 상황 | 기대 결과 |
|---|---|---|
| TEST-01 | 두 객체가 같은 속도로 평행 이동 | `relative_speed_mps = 0`, `t_cpa_s = 0`, `dcpa_m = distance_m` |
| TEST-02 | 이미 중심 간 거리 `<= 1m` | `ttc_s = 0`, `collision_predicted = true`, `cri = 1`, `CRITICAL` |
| TEST-03 | 서로 멀어짐, 거리 `> 1m` | `ttc_s = None`, `t_cpa_s = 0`, `dcpa_m = distance_m` |
| TEST-04 | 교차 이동 S04 시작점 | `ttc_s ≈ 4.6464`, `dcpa_m = 0` |
| TEST-05 | S03 시작점 | `ttc_s = 3`, `t_cpa_s ≈ 3.3333`, `cri = 0.60` |
| TEST-06 | S05 방향 전환 직후(2.0초) | `ttc_s = None`, `dcpa_m = 2`, `cri = 0.50` |
| TEST-07 | S05 방향 전환 후(2.1초) | `cri = 0.45`, `SAFE` 및 상태 변경 기록 |
| TEST-08 | CRI가 정확히 `0.50` / `0.85` | 각각 `WARNING` / `CRITICAL` |
| TEST-09 | 필수 좌표·속도에 `NaN` / `Infinity` / 결측 | 위험 계산 중단, 입력 오류 반환 |
| TEST-10 | 동일 시나리오·동일 틱 반복 계산 | 동일 수치와 위험 단계 재현 |
| TEST-11 | 매 틱 동일 단계 유지 | `LEVEL_CHANGE` 중복 생성 없음 |
| TEST-12 | 10Hz 데이터의 종료 조건 | 첫 번째 `distance_m <= 1.0` 틱에서 종료(최대 실행시간 내) |
| TEST-13 | 첫 틱(직전 값 없음) | 추정 행동 = (0, 0) |
| TEST-14 | 모든 시나리오·모든 틱 | `cri_total >= cri` |
| TEST-15 | S06 | 통합 경보가 현재 CRI 경보(3.1초)보다 먼저 발생 |

- 부동소수점 검산은 초기값으로 절대 오차 `1e-6`을 사용하되, 단위·값 크기에 따라 테스트 허용 오차를 조정할 수 있다.
- 시간·거리 단위를 혼용하지 않는다. `EPSILON`을 계산식의 모든 항에 무조건 동일하게 적용하지 않는다.
- 시나리오 종료 시 해당 틱의 위험 계산과 상태 변경 기록을 완료한 뒤 재생을 중지한다.
- 동일한 `run_id` · `tick`에는 지게차·작업자 상태가 각 1개씩 있어야 한다. 유효하지 않은 입력을 임의로 0으로 대체하지 않는다.


## 11. 구현 순서와 관련 문서
### 권장 구현 순서
1. `distance_m`, `relative_speed_mps`, `closing_speed_mps`
2. `t_cpa_s`, `dcpa_m`
3. `ttc_s`, `collision_predicted`
4. `safe_radius_m`, `cri`, `risk_level`
5. `INIT` / `LEVEL_CHANGE` 이벤트 기록
6. S01~S06 단위 테스트와 수동 검산
7. 월드 모델 롤아웃과 `cri_pred`
8. `cri_total` 판정 및 `trigger` 기록

### 관련 문서
- `requirements.md` — 기능·성능 요구사항
- `scenarios.md` — S01~S06 초기 조건·이동·종료 조건
- `data_spec.md` — `object_state`, `risk_result`, `risk_event` 필드 명세


## 12. 변경 이력
| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| v0.1 | 2026-11-16 | 최초 작성 (현재 CRI 규칙, S01~S05 검산) |
| v0.2 | 2026-11-17 | 예측 CRI(7.4)·통합 CRI(7.5) 추가, 위험 단계 판정 기준을 `cri_total`로 변경, 변수 `PRED_HORIZON_STEPS`·`PRED_ALPHA` 추가, 이벤트 저장 필드 3개 추가, S06 검산 및 TEST-13~15 추가 |
| v0.2 | 2026-11-18 | S06 통합 CRI 경보 시각 기입(WARNING 1.8초, CRITICAL 2.2초). 규칙 변경 없음 |

- 산출식 또는 임계값 변경 시 `rule_version`과 테스트 기대값을 함께 갱신한다.