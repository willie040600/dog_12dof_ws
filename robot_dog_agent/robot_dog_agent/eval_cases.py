"""The six agent_eval sentences and how each is judged from ground truth (spec 2026-11-10, 7). Plain Python."""
import math
from collections import namedtuple

from robot_dog_agent import places

# before / after and the entries of nav_ends are ground-truth (x, y, yaw); nav_ends holds the pose at the end of
# every navigation the robot was sent on, goals_sent how many there were
Observation = namedtuple('Observation', 'before after nav_ends goals_sent reply fell')

XY_TOLERANCE, YAW_TOLERANCE = 0.25, 0.3  # m, rad (the same as nav_eval)
STILL = 0.1                              # m; moved less than this = did not move


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def yaw_error(yaw, target):
    return abs(places.wrap(yaw - target))


def at_place(pose, place_id, check_yaw=True):
    """(ok, why) for pose against a named place."""
    place = places.find_place(place_id)
    d, e = distance(pose, (place.x, place.y)), yaw_error(pose[2], place.yaw)
    ok = d <= XY_TOLERANCE and (e <= YAW_TOLERANCE or not check_yaw)
    return ok, '%.2f m and %.2f rad from %s' % (d, e, place_id)


def judge_master_bedroom(obs):
    return at_place(obs.after, 'master_bedroom')


def judge_kitchen_then_start(obs):
    if not obs.nav_ends:
        return False, 'no navigation was sent'
    first, why = at_place(obs.nav_ends[0], 'kitchen', check_yaw=False)
    if not first:
        return False, 'first stop ' + why
    return at_place(obs.after, 'start')


def judge_one_metre_forward(obs):
    dx, dy, yaw = obs.after[0] - obs.before[0], obs.after[1] - obs.before[1], obs.before[2]
    along, across = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
    return (abs(along - 1.0) <= XY_TOLERANCE and abs(across) <= XY_TOLERANCE,
            'moved %.2f m forward and %.2f m sideways' % (along, across))


def judge_turn_left(obs):
    turned, moved = places.wrap(obs.after[2] - obs.before[2]), distance(obs.before, obs.after)
    return (abs(turned - math.pi / 2) <= YAW_TOLERANCE and moved <= XY_TOLERANCE,
            'turned %.0f deg and moved %.2f m' % (math.degrees(turned), moved))


def judge_stays_put(obs):
    moved = distance(obs.before, obs.after)
    return obs.goals_sent == 0 and moved <= STILL, '%d goals sent, moved %.2f m' % (obs.goals_sent, moved)


def judge_answers_without_moving(obs):
    ok, why = judge_stays_put(obs)
    return ok and bool(obs.reply.strip()), why + (', empty reply' if not obs.reply.strip() else '')


def unless_fallen(judge):
    def judged(obs):
        return (False, 'the dog fell') if obs.fell else judge(obs)
    return judged


CASES = tuple((sentence, unless_fallen(judge)) for sentence, judge in (
    ('去主臥室', judge_master_bedroom),
    ('先去廚房，再回起點', judge_kitchen_then_start),
    ('往前走一公尺', judge_one_metre_forward),
    ('向左轉 90 度', judge_turn_left),
    ('去座標 (0.4, 2.7)', judge_stays_put),          # the middle of the bed: must be refused
    ('你現在在哪裡', judge_answers_without_moving),
))
