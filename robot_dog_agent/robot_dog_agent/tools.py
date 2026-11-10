"""The tools the model can call and what it is told about them (spec 2026-11-10, 5.1). Plain Python."""
import math

from robot_dog_agent import places

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
MAX_RELATIVE = 5.0   # m, per axis of move_relative
SAME_SPOT = 0.1      # m; a go_to_pose target this close keeps the current heading

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


def system_instruction():
    return INSTRUCTION % '\n'.join('- %s: %s, (%s, %s)' % (p.id, p.name, p.x, p.y) for p in places.PLACES)


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

    def __init__(self, robot, truth, on_progress=None):
        self.robot, self.truth, self.on_progress = robot, truth, on_progress

    def call(self, name, args):
        handler = {'get_pose': self._get_pose, 'go_to_place': self._go_to_place, 'go_to_pose': self._go_to_pose,
                   'move_relative': self._move_relative}.get(name)
        try:
            if handler is None:
                raise BadArguments('no tool named %s' % name)
            if not isinstance(args, dict):
                raise BadArguments('the arguments must be an object')
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
