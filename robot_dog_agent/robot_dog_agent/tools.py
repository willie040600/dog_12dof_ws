"""The tools the model can call and what it is told about them (spec 2026-11-10, 5.1). Plain Python."""
import json
import math

from robot_dog_agent import places
from robot_dog_agent.objects import MERGE_DISTANCE
from robot_dog_agent.robot import SAME_SPOT

NUMBER = {'type': 'number'}
DECLARATIONS = [
    {'type': 'function', 'name': 'get_pose',
     'description': 'Where the robot is now: position, heading, area and nearest named place. '
                    'Does not move the robot.',
     'parameters': {'type': 'object', 'properties': {}}},
    {'type': 'function', 'name': 'go_to_place',
     'description': 'Walk to a named place and face the way listed for it.',
     'parameters': {'type': 'object',
                    'properties': {'place': {'type': 'string', 'enum': [p.id for p in places.PLACES]}},
                    'required': ['place']}},
    {'type': 'function', 'name': 'go_to_pose',
     'description': 'Walk to map coordinates in metres. Without yaw_deg the robot ends facing its direction '
                    'of travel.',
     'parameters': {'type': 'object', 'properties': {'x': NUMBER, 'y': NUMBER, 'yaw_deg': NUMBER},
                    'required': ['x', 'y']}},
    {'type': 'function', 'name': 'move_relative',
     'description': 'Move relative to the current pose: forward_m (negative = back), left_m (negative = right), '
                    'turn_left_deg (negative = right turn).',
     'parameters': {'type': 'object',
                    'properties': {'forward_m': NUMBER, 'left_m': NUMBER, 'turn_left_deg': NUMBER}}},
]
EYE_DECLARATIONS = [
    {'type': 'function', 'name': 'list_objects',
     'description': 'The things the robot has seen and remembered: id, name, map position and area.',
     'parameters': {'type': 'object', 'properties': {}}},
    {'type': 'function', 'name': 'go_to_object',
     'description': 'Walk next to a remembered thing and face it.',
     'parameters': {'type': 'object', 'properties': {'object_id': {'type': 'string'}}, 'required': ['object_id']}},
    {'type': 'function', 'name': 'find_object',
     'description': 'Search the apartment with the camera for a thing that is not remembered yet, then walk next '
                    'to it. name: what to look for, in Traditional Chinese.',
     'parameters': {'type': 'object', 'properties': {'name': {'type': 'string'}}, 'required': ['name']}},
    {'type': 'function', 'name': 'look_around',
     'description': 'Turn on the spot and look in four directions; remembers what is seen.',
     'parameters': {'type': 'object', 'properties': {}}},
    {'type': 'function', 'name': 'survey',
     'description': 'Walk to every observation point and look around, to remember where things are. '
                    'Forgets what was remembered before.',
     'parameters': {'type': 'object', 'properties': {}}},
    {'type': 'function', 'name': 'forget_objects',
     'description': 'Forget every remembered thing.',
     'parameters': {'type': 'object', 'properties': {}}},
]
MAX_RELATIVE = 5.0   # m, per axis of move_relative
TURNS = 4            # looks per spot, a quarter turn apart
MAX_LOOKS = 32       # pictures one survey or one search may send to the model
EYE_TOOLS = {d['name'] for d in EYE_DECLARATIONS}
MAX_FAILED_IN_A_ROW = 3  # pictures that failed one after another: stop instead of walking on blind

INSTRUCTION = """\
You control a quadruped robot dog in a simulated apartment through the tools. The user types commands in Chinese.

Coordinates: map frame, metres. The dog started at (0, 0) facing +x; +y is to its left when it faces +x.
Angles are degrees, counter-clockwise (a left turn) positive.

Places (id: name, x, y):
%s

Rules:
- For a named place always call go_to_place with its id. Never guess coordinates for a place or an object that is
  not in the list; say you do not know where it is.
- For forward / back / left / right / turn commands call move_relative.
- For several steps in one command, call the tools in the order the user gave.
- When a tool returns status "blocked", tell the user the reason. Do not try a nearby point unless the user asks.
- When a tool returns any status other than "ok" or "reached", stop and tell the user what happened.
- Do not call a navigation tool for a question that needs no movement. Use get_pose to answer where the dog is.
- Reply in Traditional Chinese (Taiwan), in one or two short sentences.
"""
EYE_INSTRUCTION = """\
- The robot has a camera and remembers things it has seen. When the user names a thing (a sofa, the fridge...),
  call list_objects first. If it is there, call go_to_object with its id; if not, call find_object with its name.
  Never guess coordinates for a thing. survey walks around and remembers everything; forget_objects forgets.
"""


def declarations(with_eyes=False):
    """The tools the model is told about; the camera tools only for a robot with eyes."""
    return DECLARATIONS + (EYE_DECLARATIONS if with_eyes else [])


def system_instruction(with_eyes=False):
    text = INSTRUCTION % '\n'.join('- %s: %s, (%s, %s)' % (p.id, p.name, p.x, p.y) for p in places.PLACES)
    return text + (EYE_INSTRUCTION if with_eyes else '')


def tool_line(name, args, result):
    """How a tool call and its result are shown to the user (terminal chat and window alike)."""
    reason = '（%s）' % result['reason'] if result.get('reason') else ''
    return '  [工具] %s %s → %s%s' % (name, json.dumps(args, ensure_ascii=False), result.get('status'), reason)


class BadArguments(Exception):
    pass


def number(args, name, default=None):
    """args[name] as a finite float; `default` when it is absent (no default: required)."""
    value = args.get(name)
    if value is None:
        if default is None:
            raise BadArguments('%s is required' % name)
        return default
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise BadArguments('%s must be a finite number' % name)
    return float(value)


def pose_fields(pose):
    return {'x': round(pose[0], 2) + 0.0, 'y': round(pose[1], 2) + 0.0, 'yaw_deg': round(math.degrees(pose[2]))}


class Toolbox:
    """Runs the tool calls against a robot (robot.py) and the truth floor plan. call() always returns a dict."""

    def __init__(self, robot, truth, on_progress=None, eyes=None, on_note=None):
        self.robot, self.truth, self.on_progress, self.eyes = robot, truth, on_progress, eyes
        self.on_note = on_note or (lambda text: None)
        self._failed_in_a_row, self._last_failure = 0, ''

    def call(self, name, args):
        handler = {'get_pose': self._get_pose, 'go_to_place': self._go_to_place, 'go_to_pose': self._go_to_pose,
                   'move_relative': self._move_relative, 'list_objects': self._list_objects,
                   'go_to_object': self._go_to_object, 'find_object': self._find_object,
                   'look_around': self._look_around, 'survey': self._survey,
                   'forget_objects': self._forget_objects}.get(name)
        try:
            if handler is None:
                raise BadArguments('no tool named %s' % name)
            if not isinstance(args, dict):
                raise BadArguments('the arguments must be an object')
            if name in EYE_TOOLS and self.eyes is None:
                raise BadArguments('no camera on this robot')
            return handler(args, self.robot.pose())
        except (BadArguments, RuntimeError) as e:  # RuntimeError: the robot does not know its pose
            return {'status': 'error', 'reason': str(e)}

    def _get_pose(self, args, pose):
        place, distance = places.nearest_place(pose[0], pose[1])
        return {'status': 'ok', **pose_fields(pose), 'area': places.area_of(pose[0], pose[1]),
                'nearest_place': place.id, 'distance_to_place': round(distance, 2)}

    def _go_to_place(self, args, pose):
        place = places.find_place(args.get('place'))
        if place is None:
            raise BadArguments('place must be one of %s' % ', '.join(p.id for p in places.PLACES))
        return self._navigate(pose, (place.x, place.y, place.yaw))

    def _go_to_pose(self, args, pose):
        x, y = number(args, 'x'), number(args, 'y')
        if args.get('yaw_deg') is not None:
            yaw = places.wrap(math.radians(number(args, 'yaw_deg')))
        elif math.hypot(x - pose[0], y - pose[1]) < SAME_SPOT:
            yaw = pose[2]
        else:
            yaw = math.atan2(y - pose[1], x - pose[0])
        return self._navigate(pose, (x, y, yaw))

    def _move_relative(self, args, pose):
        forward, left = number(args, 'forward_m', 0.0), number(args, 'left_m', 0.0)
        turn = number(args, 'turn_left_deg', 0.0)
        if forward == 0.0 and left == 0.0 and turn == 0.0:
            raise BadArguments('nothing to do: give forward_m, left_m or turn_left_deg')
        if abs(forward) > MAX_RELATIVE or abs(left) > MAX_RELATIVE:
            raise BadArguments('forward_m and left_m are limited to %.0f m' % MAX_RELATIVE)
        return self._navigate(pose, places.relative_target(pose, forward, left, turn))

    def _navigate(self, pose, target):
        reason = places.check_target(self.truth, target[0], target[1])
        if reason:
            return {'status': 'blocked', 'reason': reason, **pose_fields(pose)}
        result = self.robot.go_to(*target, on_progress=self.on_progress)
        out = {'status': result.status}
        if result.reason:
            out['reason'] = result.reason
        return {**out, **pose_fields(self.robot.pose()), 'seconds': round(result.seconds, 1)}

    # --- the camera tools (spec 2026-11-18, 4.3-4.5) ---

    def _list_objects(self, args, pose):
        return {'status': 'ok', 'objects': [
            {'id': i.id, 'name': i.name, 'x': round(i.x, 2) + 0.0, 'y': round(i.y, 2) + 0.0,
             'area': places.area_of(i.x, i.y)} for i in self.eyes.memory.items]}

    def _go_to_object(self, args, pose):
        item = self.eyes.memory.find(args.get('object_id')) if isinstance(args.get('object_id'), str) else None
        if item is None:
            raise BadArguments('object_id must be one of the ids from list_objects')
        return self._go_near(pose, item)

    def _go_near(self, pose, item):
        spot = places.stand_near(self.truth, (item.x, item.y), pose[:2])
        if spot is None:
            return {'status': 'blocked', 'reason': 'no reachable place to stand near it', **pose_fields(pose)}
        return {**self._navigate(pose, spot), 'object': item.id}

    def _turn(self):
        """A quarter turn left on the spot; the NavResult status."""
        x, y, yaw = self.robot.pose()
        return self.robot.go_to(x, y, places.wrap(yaw + math.pi / 2.0), on_progress=self.on_progress).status

    def _look_four_ways(self, target, budget):
        """(sightings of each look, status): looks and turns until something is seen (with a target), the budget is
        spent, or a turn is canceled; a look that fails counts as nothing seen and as a failure (None)."""
        looks = []
        for k in range(TURNS):
            if len(looks) >= budget:
                break
            try:
                seen = self.eyes.look(target)
                self._failed_in_a_row = 0
            except Exception as e:  # the model or the camera failed for this picture; the next one may work
                seen, self._failed_in_a_row = None, self._failed_in_a_row + 1
                self._last_failure = str(e) or type(e).__name__
            looks.append(seen)
            if self._failed_in_a_row >= MAX_FAILED_IN_A_ROW:
                return looks, 'failing'
            if target and seen:
                return looks, 'found'
            if k < TURNS - 1 or not target:
                status = self._turn()
                if status == 'canceled':
                    return looks, 'canceled'
        return looks, 'done'

    def _failing(self):
        return {'status': 'error', 'reason': 'the camera pictures keep failing: %s' % self._last_failure}

    def _look_around(self, args, pose):
        self._failed_in_a_row = 0
        looks, status = self._look_four_ways(None, TURNS)
        if status == 'failing':
            return self._failing()
        if status == 'canceled':
            return {'status': 'canceled', 'reason': 'canceled by the user'}
        names = []
        for seen in looks:
            names += [s.name for s in seen or [] if s.name not in names]
        return {'status': 'ok', 'seen': names, 'remembered': len(self.eyes.memory.items)}

    def _survey(self, args, pose):
        before = self.eyes.memory.snapshot()  # put back when no picture works at all
        self.eyes.memory.clear()
        looks = failed = 0
        self._failed_in_a_row = 0
        skipped = []
        for i, place in enumerate(places.PLACES, 1):
            self.on_note('巡邏中 %d/%d' % (i, len(places.PLACES)))
            here = self.robot.pose()
            went = self._navigate(here, (place.x, place.y, place.yaw))
            if went['status'] == 'canceled':
                return {'status': 'canceled', 'reason': 'canceled by the user',
                        'remembered': len(self.eyes.memory.items)}
            if went['status'] != 'reached':
                skipped.append(place.id)
                continue
            seen, status = self._look_four_ways(None, MAX_LOOKS - looks)
            looks += len(seen)
            failed += sum(s is None for s in seen)
            if status == 'failing':
                if failed == looks:
                    self.eyes.memory.restore(before)
                return self._failing()
            if status == 'canceled':
                return {'status': 'canceled', 'reason': 'canceled by the user',
                        'remembered': len(self.eyes.memory.items)}
        return {'status': 'ok', 'remembered': len(self.eyes.memory.items), 'looks': looks, 'failed_looks': failed,
                'skipped': skipped}

    def _find_object(self, args, pose):
        name = args.get('name')
        if not isinstance(name, str) or not name.strip():
            raise BadArguments('name must be the thing to look for')
        name, looks = name.strip(), 0
        self._failed_in_a_row = 0
        order = sorted(places.PLACES, key=lambda p: math.hypot(p.x - pose[0], p.y - pose[1]))
        for place in order:
            if looks >= MAX_LOOKS:
                break
            here = self.robot.pose()
            went = self._navigate(here, (place.x, place.y, place.yaw))
            if went['status'] == 'canceled':
                return {'status': 'canceled', 'reason': 'canceled by the user'}
            if went['status'] != 'reached':
                continue
            seen, status = self._look_four_ways(name, MAX_LOOKS - looks)
            looks += len(seen)
            if status == 'failing':
                return self._failing()
            if status == 'canceled':
                return {'status': 'canceled', 'reason': 'canceled by the user'}
            if status == 'found':
                here = self.robot.pose()
                nearest = min(seen[-1], key=lambda s: math.hypot(s.x - here[0], s.y - here[1]))
                item = next(i for i in self.eyes.memory.items if i.name == nearest.name
                            and math.hypot(i.x - nearest.x, i.y - nearest.y) <= MERGE_DISTANCE)
                return {**self._go_near(here, item), 'found_at': place.id}
        return {'status': 'not_found', 'reason': 'looked from every observation point', 'looks': looks}

    def _forget_objects(self, args, pose):
        self.eyes.memory.clear()
        return {'status': 'ok'}
