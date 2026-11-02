from robot_dog_lab.gait_criteria import CASES, judge_case, judge_stand


def _case(name):
    return next(c for c in CASES if c.name == name)


def test_cases_match_spec():
    cmds = {c.command for c in CASES}
    expected = {(s * v, 0.0, 0.0) for v in (0.1, 0.3, 0.5) for s in (1, -1)} \
        | {(0.0, s * v, 0.0) for v in (0.1, 0.3) for s in (1, -1)} \
        | {(0.0, 0.0, s * v) for v in (0.2, 0.5, 0.8) for s in (1, -1)} | {(0.0, 0.0, 0.0)}
    assert len(CASES) == 17 and cmds == expected
    assert sum(c.axis is None for c in CASES) == 1


def test_linear_tolerance():
    assert judge_case(_case("vx+0.30"), (0.38, 0.0, 0.0), 0.0, 0)[0]
    ok, why = judge_case(_case("vx+0.30"), (0.41, 0.0, 0.0), 0.0, 0)
    assert not ok and "vx" in why
    assert not judge_case(_case("vy-0.10"), (0.0, 0.01, 0.0), 0.0, 0)[0]


def test_standing_still_fails_small_commands():
    # a policy that ignores 0.1 m/s commands has error exactly 0.1 and must not pass (Nav2 sends such commands)
    assert not judge_case(_case("vx+0.10"), (0.0, 0.0, 0.0), 0.0, 0)[0]
    assert not judge_case(_case("vy-0.10"), (0.0, 0.0, 0.0), 0.0, 0)[0]
    assert not judge_case(_case("wz+0.20"), (0.0, 0.0, 0.07), 0.0, 0)[0]
    assert judge_case(_case("vx+0.10"), (0.06, 0.0, 0.0), 0.0, 0)[0]


def test_angular_tolerance():
    assert judge_case(_case("wz+0.50"), (0.0, 0.0, 0.64), 0.0, 0)[0]
    assert not judge_case(_case("wz+0.50"), (0.0, 0.0, 0.66), 0.0, 0)[0]


def test_stand_uses_drift():
    assert judge_case(_case("stand"), (0.05, 0.0, 0.0), 0.09, 0)[0]
    assert not judge_case(_case("stand"), (0.0, 0.0, 0.0), 0.11, 0)[0]


def test_any_fall_fails():
    ok, why = judge_case(_case("vx+0.10"), (0.1, 0.0, 0.0), 0.0, 1)
    assert not ok and "fall" in why


def test_judge_stand():
    assert judge_stand([0.22] * 16, 0)[0]
    assert not judge_stand([0.22] * 15 + [0.15], 0)[0]
    assert not judge_stand([0.22] * 16, 1)[0]


def test_crawling_fails():
    ok, why = judge_case(_case("vx+0.30"), (0.30, 0.0, 0.0), 0.0, 0, height=0.09)
    assert not ok and "height" in why
    assert judge_case(_case("vx+0.30"), (0.30, 0.0, 0.0), 0.0, 0, height=0.22)[0]
    assert not judge_case(_case("stand"), (0.0, 0.0, 0.0), 0.0, 0, height=0.10)[0]
