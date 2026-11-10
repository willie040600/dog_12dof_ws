import math

from robot_dog_agent import places
from robot_dog_agent.eval_cases import CASES, Observation

MB, K, S = (places.find_place(i) for i in ('master_bedroom', 'kitchen', 'start'))
JUDGE = [judge for _, judge in CASES]


def obs(**kw):
    return Observation(**{**dict(before=(0.0, 0.0, 0.0), after=(0.0, 0.0, 0.0), nav_ends=[], goals_sent=0, reply='好',
                                 fell=False), **kw})


def ok(n, **kw):
    passed, why = JUDGE[n - 1](obs(**kw))
    assert why
    return passed


def test_sentences():
    assert [s for s, _ in CASES] == ['去主臥室', '先去廚房，再回起點', '往前走一公尺', '向左轉 90 度', '去座標 (0.4, 2.7)',
                                     '你現在在哪裡']


def test_case_1_master_bedroom():
    assert ok(1, after=(MB.x + 0.2, MB.y, MB.yaw - 0.25), goals_sent=1)
    assert not ok(1, after=(MB.x + 0.3, MB.y, MB.yaw), goals_sent=1)
    assert not ok(1, after=(MB.x, MB.y, MB.yaw + 0.4), goals_sent=1)
    assert not ok(1)                                                        # never left the start


def test_case_2_kitchen_then_start():
    assert ok(2, nav_ends=[(K.x, K.y - 0.2, K.yaw), (0.1, 0.0, 0.0)], after=(0.1, 0.0, 0.1), goals_sent=2)
    assert not ok(2, nav_ends=[], after=(0.0, 0.0, 0.0))                    # did not move at all
    assert not ok(2, nav_ends=[(K.x + 0.5, K.y, K.yaw), (0.0, 0.0, 0.0)], after=(0.0, 0.0, 0.0), goals_sent=2)
    assert not ok(2, nav_ends=[(K.x, K.y, K.yaw)], after=(K.x, K.y, K.yaw), goals_sent=1)   # stayed in the kitchen


def test_case_3_one_metre_forward():
    up = (0.0, 0.0, math.pi / 2)
    assert ok(3, before=up, after=(0.1, 1.1, math.pi / 2), goals_sent=1)
    assert not ok(3, before=up, after=(0.0, 0.7, math.pi / 2), goals_sent=1)
    assert not ok(3, before=up, after=(0.3, 1.0, math.pi / 2), goals_sent=1)
    assert not ok(3, before=up, after=(0.0, -1.0, math.pi / 2), goals_sent=1)   # went backwards


def test_case_4_turn_left():
    assert ok(4, after=(0.0, 0.1, 1.5), goals_sent=1)
    assert ok(4, before=(0.0, 0.0, 3.0), after=(0.0, 0.0, 3.0 + math.pi / 2 - 2 * math.pi), goals_sent=1)   # wraps
    assert not ok(4, after=(0.0, 0.0, -math.pi / 2), goals_sent=1)
    assert not ok(4, after=(0.0, 0.0, 1.0), goals_sent=1)
    assert not ok(4, after=(0.3, 0.0, math.pi / 2), goals_sent=1)


def test_case_5_blocked_coordinates():
    assert ok(5, before=(1.0, 0.0, 1.5), after=(1.05, 0.0, 1.5))
    assert not ok(5, goals_sent=1)
    assert not ok(5, after=(0.2, 0.0, 0.0))


def test_case_6_where_am_i():
    assert ok(6, reply='我在起點附近')
    assert not ok(6, reply='  ')
    assert not ok(6, goals_sent=1)
    assert not ok(6, after=(0.2, 0.0, 0.0))


def test_a_fall_fails_every_case():
    good = [dict(after=(MB.x, MB.y, MB.yaw), goals_sent=1),
            dict(nav_ends=[(K.x, K.y, K.yaw), (0.0, 0.0, 0.0)], goals_sent=2),
            dict(after=(1.0, 0.0, 0.0), goals_sent=1), dict(after=(0.0, 0.0, math.pi / 2), goals_sent=1), {}, {}]
    for n, kw in enumerate(good, 1):
        assert ok(n, **kw), n
        passed, why = JUDGE[n - 1](obs(fell=True, **kw))
        assert not passed and 'fell' in why, n


# --- the runner ---

from robot_dog_agent.eval_cases import WatchedRobot, run_cases  # noqa: E402
from robot_dog_agent.robot import FakeRobot  # noqa: E402


def test_watched_robot_counts_goals_and_records_where_they_ended():
    robot = FakeRobot()
    watched = WatchedRobot(robot, truth_pose=lambda: tuple(v + 0.01 for v in robot.pose()))
    assert watched.pose() == (0.0, 0.0, 0.0)
    seen = []
    assert watched.go_to(1.0, 0.0, 0.0, on_progress=seen.append).status == 'reached'
    watched.go_to(1.0, 1.0, 0.5)
    assert watched.goals_sent == 2 and watched.nav_ends == [(1.01, 0.01, 0.01), (1.01, 1.01, 0.51)]
    assert robot.progress_callbacks[0] == seen.append
    watched.reset()
    assert watched.goals_sent == 0 and watched.nav_ends == []


def test_run_cases_judges_each_sentence_and_carries_on_after_an_error():
    robot = FakeRobot()
    watched = WatchedRobot(robot, truth_pose=robot.pose)
    seen = []

    asked, fallen = [], []

    def ask(sentence):
        asked.append(sentence)
        if sentence == '壞掉':
            raise RuntimeError('rate limited')
        if sentence == '走':
            watched.go_to(1.0, 0.0, 0.0)
        if len(asked) == 2:
            fallen.append(True)             # the dog falls during the second sentence
        return '好'

    def fell():                             # True when the dog fell since the last call
        return bool(fallen) and not fallen.clear()

    def judge(obs):
        seen.append(obs)
        return obs.goals_sent == 1, '%d goals' % obs.goals_sent

    said = []
    passed = run_cases(ask, watched, truth_pose=robot.pose, fell=fell, say=said.append,
                       cases=(('走', judge), ('走', judge), ('壞掉', judge), ('站著', judge)))
    assert passed == 2
    assert seen == [Observation(before=(0.0, 0.0, 0.0), after=(1.0, 0.0, 0.0), nav_ends=[(1.0, 0.0, 0.0)],
                                goals_sent=1, reply='好', fell=False),
                    Observation(before=(1.0, 0.0, 0.0), after=(1.0, 0.0, 0.0), nav_ends=[(1.0, 0.0, 0.0)],
                                goals_sent=1, reply='好', fell=True),      # counters start over for every case
                    Observation(before=(1.0, 0.0, 0.0), after=(1.0, 0.0, 0.0), nav_ends=[], goals_sent=0, reply='好',
                                fell=False)]                                # the failed sentence is not judged
    assert said == ['[AGENT] 走  PASS  1 goals | 回覆：好', '[AGENT] 走  PASS  1 goals | 回覆：好',
                    '[AGENT] 壞掉  FAIL  error: rate limited | 回覆：', '[AGENT] 站著  FAIL  0 goals | 回覆：好',
                    '[AGENT] PASS 2/4']
