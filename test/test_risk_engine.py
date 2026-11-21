"""
risk_engine.py 단위 테스트 (docs/risk_rules.md 9장 수동 검산표, 10장 TEST-01 ~ TEST-10)

실행:  python -m pytest -v test/test_risk_engine.py (프로젝트 폴더에서 실행)

테스트 이름에 문서의 테스트 ID 를 넣어, 실패했을 때 문서의 어느 줄이 깨졌는지 바로 알 수 있게 했다.
TEST-11 ~ TEST-16 는 이벤트 기록·시뮬레이터·월드 모델이 대상이라 이 파일에서 다루지 않는다.
"""
import math

import pytest

from risk_engine import (
    CRITICAL_THRESHOLD,
    WARNING_THRESHOLD,
    CollisionRiskEngine,
    classify_level,
)
from simulator import WarehouseSimulator

TOL = 1e-6  # 문서 10장의 절대 허용 오차


def make_obj(x: float, y: float, vx: float, vy: float) -> dict:
    """
    문서는 속도를 (vx, vy) 로 적고 코드는 (speed, heading) 을 받으므로 바꿔 주는 도우미.
    시뮬레이션을 돌리지 않고 상태를 직접 만드는 이유: 0.1초씩 더해 가면 2.0 이 1.9999999 가 되어
    정확히 임계값(0.50)에 걸리는 경우의 판정이 흔들리기 때문이다.
    """
    return {"x": x, "y": y, "speed": math.hypot(vx, vy), "heading": math.atan2(vy, vx)}


@pytest.fixture
def engine() -> CollisionRiskEngine:
    return CollisionRiskEngine()


def approx(value: float):
    return pytest.approx(value, abs=TOL)


# ---------------------------------------------------------------------------
# 9장. S01~S06 초기 수동 검산표
# 표의 "약 12.81m" 같은 값은 반올림된 것이라, 비교용 기대값은 식으로 직접 적었다.
# 열 순서: 지게차(x,y,vx,vy), 작업자(x,y,vx,vy), distance, relative_speed, closing_speed,
#          t_cpa, dcpa, ttc, cri, status
# ---------------------------------------------------------------------------
S06_R = (6.5, 8.5)      # 작업자 - 지게차 위치
S06_V = (-2.8, -1.1)    # 작업자 - 지게차 속도
S06_TCPA = -(S06_R[0] * S06_V[0] + S06_R[1] * S06_V[1]) / (S06_V[0] ** 2 + S06_V[1] ** 2)

INITIAL_CASES = [
    pytest.param((0, 0, 2, 0), (10, 8, 1, 0),
                 math.sqrt(164), 1.0, 10 / math.sqrt(164), 10.0, 8.0, None, 0.0, "SAFE",
                 id="S01"),
    pytest.param((0, 0, 0, 0), (10, 0, -1, 0),
                 10.0, 1.0, 1.0, 10.0, 0.0, 9.0, 0.0, "SAFE",
                 id="S02"),
    pytest.param((0, 0, 2, 0), (28, 0, -1, 0),
                 28.0, 3.0, 3.0, 28 / 3, 0.0, 9.0, 0.0, "SAFE",
                 id="S03"),
    pytest.param((0, 0, 2, 0), (20, 10, 0, -1),
                 math.sqrt(500), math.sqrt(5), math.sqrt(5), 10.0, 0.0, 10 - 1 / math.sqrt(5),
                 0.0, "SAFE",
                 id="S04"),
    pytest.param((0, 0, 0, 0), (4, 0, -1, 0),
                 4.0, 1.0, 1.0, 4.0, 0.0, 3.0, 0.60, "WARNING",
                 id="S05"),
    pytest.param((6, 4, 2.8, 0), (12.5, 12.5, 0, -1.1),
                 math.hypot(*S06_R), math.hypot(*S06_V), 27.55 / math.hypot(*S06_R), S06_TCPA,
                 math.hypot(S06_R[0] + S06_V[0] * S06_TCPA, S06_R[1] + S06_V[1] * S06_TCPA),
                 None, 0.0, "SAFE",
                 id="S06"),
]


@pytest.mark.parametrize(
    "forklift, worker, distance, relative_speed, closing_speed, t_cpa, dcpa, ttc, cri, status",
    INITIAL_CASES,
)
def test_initial_values_match_manual_check(
    engine, forklift, worker, distance, relative_speed, closing_speed, t_cpa, dcpa, ttc, cri, status
):
    result = engine.calculate_risk(make_obj(*forklift), make_obj(*worker))

    assert result["distance"] == approx(distance)
    assert result["relative_speed"] == approx(relative_speed)
    assert result["closing_speed"] == approx(closing_speed)
    assert result["t_cpa"] == approx(t_cpa)
    assert result["dcpa"] == approx(dcpa)
    if ttc is None:
        assert result["ttc"] is None
        assert result["collision_predicted"] is False
    else:
        assert result["ttc"] == approx(ttc)
        assert result["collision_predicted"] is True
    assert result["cri"] == approx(cri)
    assert result["status"] == status


def test_initial_values_match_rounded_numbers_in_doc(engine):
    """표에 적힌 반올림 값(약 12.81m, 약 9.55초 등)과도 맞는지 한 번 더 확인한다."""
    s01 = engine.calculate_risk(make_obj(0, 0, 2, 0), make_obj(10, 8, 1, 0))
    s04 = engine.calculate_risk(make_obj(0, 0, 2, 0), make_obj(20, 10, 0, -1))
    s06 = engine.calculate_risk(make_obj(6, 4, 2.8, 0), make_obj(12.5, 12.5, 0, -1.1))

    assert round(s01["distance"], 2) == 12.81
    assert round(s01["closing_speed"], 2) == 0.78
    assert round(s04["distance"], 2) == 22.36
    assert round(s04["ttc"], 2) == 9.55
    assert round(s04["cri"], 2) == 0.0
    assert round(s06["distance"], 2) == 10.70
    assert round(s06["relative_speed"], 2) == 3.01
    assert round(s06["closing_speed"], 2) == 2.57
    assert round(s06["t_cpa"], 2) == 3.04
    assert round(s06["dcpa"], 2) == 5.53


# ---------------------------------------------------------------------------
# 10장. 예외 처리 및 단위 테스트
# ---------------------------------------------------------------------------
def test_01_parallel_same_velocity(engine):
    """TEST-01: 같은 속도로 평행 이동하면 거리가 변하지 않는다."""
    result = engine.calculate_risk(make_obj(0, 0, 2, 0), make_obj(0, 5, 2, 0))

    assert result["relative_speed"] == approx(0.0)
    assert result["t_cpa"] == 0.0
    assert result["dcpa"] == approx(result["distance"])
    assert result["ttc"] is None


@pytest.mark.parametrize("distance", [0.0, 0.5, 1.0])
def test_02_already_inside_collision_radius(engine, distance):
    """TEST-02: 이미 1m 이하(정확히 1m 포함)이면 속도와 무관하게 최고 위험."""
    result = engine.calculate_risk(make_obj(0, 0, 0, 0), make_obj(distance, 0, 1, 0))

    assert result["ttc"] == 0.0
    assert result["collision_predicted"] is True
    assert result["cri"] == 1.0
    assert result["status"] == "CRITICAL"


def test_03_moving_apart(engine):
    """TEST-03: 서로 멀어지는 중이면 TTC 없음, 지금 거리가 최근접 거리."""
    result = engine.calculate_risk(make_obj(0, 0, 0, 0), make_obj(5, 0, 1, 0))

    assert result["ttc"] is None
    assert result["t_cpa"] == 0.0
    assert result["dcpa"] == approx(result["distance"])
    assert result["closing_speed"] < 0


def test_04_s04_crossing(engine):
    """TEST-04: 교차 이동 S04 시작점."""
    result = engine.calculate_risk(make_obj(0, 0, 2, 0), make_obj(20, 10, 0, -1))

    assert result["ttc"] == pytest.approx(9.5528, abs=1e-4)
    assert result["dcpa"] == approx(0.0)


def test_05_s03_head_on(engine):
    """TEST-05: S03 시작점."""
    result = engine.calculate_risk(make_obj(0, 0, 2, 0), make_obj(28, 0, -1, 0))

    assert result["ttc"] == approx(9.0)
    assert result["t_cpa"] == pytest.approx(9.3333, abs=1e-4)
    assert result["cri"] == approx(0.00)


def test_06_s05_right_after_turn(engine):
    """TEST-06: S05 방향 전환 직후(2.0초). 작업자는 (2, 0)에서 +x 로 멀어지기 시작한다."""
    result = engine.calculate_risk(make_obj(0, 0, 0, 0), make_obj(2, 0, 1, 0))

    assert result["ttc"] is None
    assert result["dcpa"] == approx(2.0)
    assert result["cri"] == approx(0.50)
    assert result["status"] == "WARNING"


def test_07_s05_after_turn(engine):
    """TEST-07: S05 방향 전환 후(2.1초). 경보가 해제된다. (상태 변경 '기록' 은 이벤트 모듈에서 검증)"""
    result = engine.calculate_risk(make_obj(0, 0, 0, 0), make_obj(2.1, 0, 1, 0))

    assert result["cri"] == approx(0.45)
    assert result["status"] == "SAFE"


def test_08_threshold_boundaries():
    """TEST-08: 경계값은 높은 단계에 포함된다."""
    assert classify_level(WARNING_THRESHOLD) == "WARNING"
    assert classify_level(CRITICAL_THRESHOLD) == "CRITICAL"
    assert classify_level(WARNING_THRESHOLD - 1e-9) == "SAFE"
    assert classify_level(CRITICAL_THRESHOLD - 1e-9) == "WARNING"


@pytest.mark.parametrize(
    "key, bad_value",
    [("x", float("nan")), ("y", float("inf")), ("speed", float("-inf")),
     ("speed", -1.0), ("x", None), ("heading", float("nan"))],
)
def test_09_invalid_input_is_rejected(engine, key, bad_value):
    """TEST-09: 잘못된 값은 0 으로 바꿔 계산하지 않고 오류를 낸다."""
    worker = make_obj(10, 0, -1, 0)
    worker[key] = bad_value

    with pytest.raises(ValueError):
        engine.calculate_risk(make_obj(0, 0, 2, 0), worker)


def test_09_missing_field_is_rejected(engine):
    """TEST-09: 필수 값이 아예 없는 경우."""
    worker = make_obj(10, 0, -1, 0)
    del worker["y"]

    with pytest.raises(ValueError):
        engine.calculate_risk(make_obj(0, 0, 2, 0), worker)


def test_10_same_input_same_output(engine):
    """TEST-10: 같은 입력이면 항상 같은 결과."""
    forklift, worker = make_obj(0, 0, 2, 0), make_obj(10, 10, 0, -2)

    assert engine.calculate_risk(forklift, worker) == engine.calculate_risk(forklift, worker)


# ---------------------------------------------------------------------------
# 시뮬레이터 연동: S06 코너 회전의 "현재 CRI" 동작 (9장 시나리오별 추가 기대 동작)
# ---------------------------------------------------------------------------
def test_s06_current_cri_jumps_to_critical(engine):
    sim = WarehouseSimulator(scenario="corner_turn")
    status_by_time, result_by_time = {}, {}
    for _ in range(60):
        frame = sim.step()
        result = engine.calculate_risk(frame["forklift"], frame["worker"])
        status_by_time[frame["timestamp"]] = result["status"]
        result_by_time[frame["timestamp"]] = result
        if result["distance"] <= 1.0:
            end_time = frame["timestamp"]
            break

    # 3.0초까지는 SAFE, WARNING 을 거치지 않고 3.1초에 CRITICAL
    assert all(status == "SAFE" for t, status in status_by_time.items() if t <= 3.0)
    assert "WARNING" not in status_by_time.values()
    assert status_by_time[3.1] == "CRITICAL"
    assert result_by_time[3.1]["ttc"] == pytest.approx(0.59, abs=0.005)
    assert result_by_time[3.1]["cri"] == 1.0

    # 3.6초에 거리 약 0.91m 로 종료
    assert end_time == 3.6
    assert result_by_time[3.6]["distance"] == pytest.approx(0.91, abs=0.005)
    