"""What the dog has seen and where (spec 2026-11-18, 4.2). Plain Python.

One item per thing: a sighting with the same name within MERGE_DISTANCE of an item is the same thing, and moves it
to the average of all its sightings. Kept in a JSON file so it survives a restart.
"""
import json
import math
import os
import time
from collections import namedtuple

MERGE_DISTANCE = 0.6  # m
MEMORY_PATH = os.path.expanduser('~/.local/share/robot_dog_agent/objects.json')

Item = namedtuple('Item', 'id name x y width seen last_seen')  # seen: how many sightings; last_seen: time.time()


class ObjectMemory:
    def __init__(self, path=MEMORY_PATH, clock=time.time):
        self.path, self.clock = path, clock
        self._items, self._next = {}, 1
        self._load()

    @property
    def items(self):
        return sorted(self._items.values(), key=lambda i: int(i.id.split('_')[1]))

    def find(self, item_id):
        return self._items.get(item_id)

    def add(self, seen):
        """Remember a sighting (vision.Seen); returns the id of the item it now belongs to."""
        match = min((i for i in self._items.values()
                     if i.name == seen.name and math.hypot(i.x - seen.x, i.y - seen.y) <= MERGE_DISTANCE),
                    key=lambda i: math.hypot(i.x - seen.x, i.y - seen.y), default=None)
        if match is None:
            item = Item('obj_%d' % self._next, seen.name, seen.x, seen.y, seen.width, 1, self.clock())
            self._next += 1
        else:
            n = match.seen
            item = match._replace(x=(match.x * n + seen.x) / (n + 1), y=(match.y * n + seen.y) / (n + 1),
                                  width=(match.width * n + seen.width) / (n + 1), seen=n + 1, last_seen=self.clock())
        self._items[item.id] = item
        self._save()
        return item.id

    def clear(self):
        """Forget everything; ids go on counting, so an old id never names a new thing."""
        self._items = {}
        self._save()

    def snapshot(self):
        """Everything remembered, to put back with restore()."""
        return dict(self._items), self._next

    def restore(self, state):
        self._items, self._next = dict(state[0]), max(state[1], self._next)
        self._save()

    def _load(self):
        try:
            with open(self.path) as f:
                data = json.load(f)
            items = [Item(**i) for i in data['items']]
            next_id = int(data['next'])
        except (OSError, ValueError, KeyError, TypeError):
            return  # no file or a broken one: start empty and leave the file as it is
        self._items = {i.id: i for i in items}
        self._next = next_id

    def _save(self):
        data = {'next': self._next, 'items': [i._asdict() for i in self.items]}
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
            os.replace(tmp, self.path)
        except OSError:
            pass  # the memory still works for this run
