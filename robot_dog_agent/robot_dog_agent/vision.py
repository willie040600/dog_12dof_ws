"""From a picture and its depth to things on the map (spec 2026-11-18, 4.1). numpy and OpenCV only: no ROS, no Qt.

Camera optical frame: x right, y down, z along the optical axis; the depth image gives z in metres.
Gemini's boxes are [ymin, xmin, ymax, xmax] scaled to 0-1000.
"""
import json
import re
from collections import namedtuple

import cv2
import numpy as np

Detection = namedtuple('Detection', 'name box')                       # box: (ymin, xmin, ymax, xmax), 0-1000
Seen = namedtuple('Seen', 'name x y distance width')                  # map metres; distance and width in metres
Snapshot = namedtuple('Snapshot', 'rgb width height depth K T_map_cam')  # RGB bytes, float32 depth, 3x3, 4x4

MIN_RANGE, MAX_RANGE = 0.1, 5.0  # m of depth that is trusted
INNER = 0.5                      # share of a box's width and height, around its centre, whose depth is used
MIN_PIXELS = 20                  # fewer valid depth pixels than this: no distance

PROMPT = """\
List the furniture and large objects you can see in this picture from a robot's camera in an apartment.
Answer with a JSON array only, no other text. Each item: {"label": <short name in Traditional Chinese>,
"box_2d": [ymin, xmin, ymax, xmax]} with the coordinates scaled to 0-1000. One item per object.
If there is nothing, answer []."""
TARGET_PROMPT = """\
Is there a %s in this picture from a robot's camera in an apartment? Answer with a JSON array only, no other text:
one item per %s you can see, {"label": <short name in Traditional Chinese>, "box_2d": [ymin, xmin, ymax, xmax]}
with the coordinates scaled to 0-1000. If there is none, answer []."""


def detection_prompt(target=None):
    return TARGET_PROMPT % (target, target) if target else PROMPT


def parse_detections(text):
    """The usable items of the first JSON array in the model's answer; anything malformed is dropped."""
    match = re.search(r'\[.*\]', text, re.S)
    if not match:
        return []
    try:
        items = json.loads(match.group(0))
    except ValueError:
        return []
    found = []
    for item in items if isinstance(items, list) else []:
        if not isinstance(item, dict) or not isinstance(item.get('label'), str) or not item['label'].strip():
            continue
        box = item.get('box_2d')
        if not isinstance(box, list) or len(box) != 4 or not all(
                isinstance(v, (int, float)) and not isinstance(v, bool) for v in box):
            continue
        ymin, xmin, ymax, xmax = (min(max(int(round(v)), 0), 1000) for v in box)
        if ymax > ymin and xmax > xmin:
            found.append(Detection(item['label'].strip(), (ymin, xmin, ymax, xmax)))
    return found


def box_pixels(box, width, height):
    """(u0, v0, u1, v1): column and row bounds of a 0-1000 box in a width x height picture."""
    ymin, xmin, ymax, xmax = box
    return (int(round(xmin * width / 1000.0)), int(round(ymin * height / 1000.0)),
            int(round(xmax * width / 1000.0)), int(round(ymax * height / 1000.0)))


def box_depth(depth, pixels):
    """Median depth of the middle of the box, or None when too little of it is valid and in range."""
    u0, v0, u1, v1 = pixels
    du, dv = (u1 - u0) * (1.0 - INNER) / 2.0, (v1 - v0) * (1.0 - INNER) / 2.0
    patch = np.asarray(depth)[int(v0 + dv):int(np.ceil(v1 - dv)), int(u0 + du):int(np.ceil(u1 - du))]
    valid = patch[np.isfinite(patch) & (patch >= MIN_RANGE) & (patch <= MAX_RANGE)]
    return float(np.median(valid)) if valid.size >= MIN_PIXELS else None


def transform_matrix(translation, quaternion):
    """4x4 homogeneous transform from a translation and a quaternion (x, y, z, w)."""
    x, y, z, w = quaternion
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = translation
    return m


def pixel_to_map(u, v, z, K, T_map_cam):
    """Map (x, y) of the point seen at pixel (u, v) at depth z."""
    point = np.array([(u - K[0][2]) * z / K[0][0], (v - K[1][2]) * z / K[1][1], z, 1.0])
    x, y = (np.asarray(T_map_cam) @ point)[:2]
    return float(x), float(y)


def locate(detections, snapshot):
    """Seen things of the detections whose box has a usable depth."""
    found = []
    for detection in detections:
        pixels = box_pixels(detection.box, snapshot.width, snapshot.height)
        z = box_depth(snapshot.depth, pixels)
        if z is None:
            continue
        u0, v0, u1, v1 = pixels
        x, y = pixel_to_map((u0 + u1) / 2.0, (v0 + v1) / 2.0, z, snapshot.K, snapshot.T_map_cam)
        found.append(Seen(detection.name, x, y, z, (u1 - u0) * z / snapshot.K[0][0]))
    return found


def depth_from_msg(encoding, width, height, step, data):
    """float32 [height, width] metres from the fields of a sensor_msgs/Image, or None."""
    kinds = {'32FC1': (np.float32, 1.0), '16UC1': (np.uint16, 0.001)}
    if encoding not in kinds or width <= 0 or height <= 0:
        return None
    dtype, scale = kinds[encoding]
    size = np.dtype(dtype).itemsize
    raw = np.frombuffer(data, dtype=np.uint8)
    if step < width * size or raw.size < step * height:
        return None
    rows = raw[:step * height].reshape(height, step)[:, :width * size]
    return np.ascontiguousarray(rows).view(dtype).reshape(height, width).astype(np.float32) * np.float32(scale)


def encode_jpeg(rgb, width, height, quality=85):
    """JPEG bytes of packed RGB bytes."""
    bgr = np.frombuffer(rgb, dtype=np.uint8).reshape(height, width, 3)[:, :, ::-1]
    ok, data = cv2.imencode('.jpg', np.ascontiguousarray(bgr), [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError('could not encode the picture')
    return data.tobytes()
