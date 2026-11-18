import cv2
import numpy as np

from robot_dog_agent.vision import (Detection, Seen, Snapshot, box_depth, box_pixels, depth_from_msg,
                                    detection_prompt, encode_jpeg, locate, parse_detections, pixel_to_map,
                                    transform_matrix)

K = np.array([[466.0, 0.0, 320.0], [0.0, 466.0, 240.0], [0.0, 0.0, 1.0]])
OPTICAL = transform_matrix((0.178, 0.0, 0.28), (-0.5, 0.5, -0.5, 0.5))   # dog at the origin facing +x


def test_transform_matrix_optical_frame():
    assert np.allclose(OPTICAL[:3, :3], [[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    assert np.allclose(OPTICAL[:3, 3], (0.178, 0.0, 0.28)) and np.allclose(OPTICAL[3], (0, 0, 0, 1))


def test_pixel_to_map_centre_and_right():
    assert np.allclose(pixel_to_map(320, 240, 2.0, K, OPTICAL), (2.178, 0.0))
    assert np.allclose(pixel_to_map(320 + 466, 240, 2.0, K, OPTICAL), (2.178, -2.0))   # image right = robot's right


def test_box_pixels():
    assert box_pixels((250, 500, 750, 1000), 640, 480) == (320, 120, 640, 360)


def test_box_depth_uses_the_middle():
    depth = np.full((480, 640), 9.0, np.float32)
    depth[180:300, 240:400] = 2.0                         # the middle half of the box (160..480, 120..360)
    assert box_depth(depth, (160, 120, 480, 360)) == 2.0


def test_box_depth_rejects_far_and_empty():
    box = (160, 120, 480, 360)
    assert box_depth(np.full((480, 640), np.inf, np.float32), box) is None
    assert box_depth(np.full((480, 640), 6.0, np.float32), box) is None
    sparse = np.full((480, 640), np.inf, np.float32)
    sparse[240, 300:310] = 2.0                            # 10 valid pixels only
    assert box_depth(sparse, box) is None


def snapshot(depth):
    return Snapshot(bytes(640 * 480 * 3), 640, 480, depth, K, OPTICAL)


def test_locate():
    depth = np.full((480, 640), np.inf, np.float32)
    depth[200:280, 280:360] = 2.0
    found = locate([Detection('沙發', (375, 375, 625, 625)), Detection('窗戶', (0, 0, 100, 100))], snapshot(depth))
    assert len(found) == 1 and found[0].name == '沙發'
    u0, _, u1, _ = box_pixels((375, 375, 625, 625), 640, 480)
    assert np.allclose((found[0].x, found[0].y, found[0].distance), (2.178, 0.0, 2.0), atol=1e-6)
    assert abs(found[0].width - (u1 - u0) * 2.0 / 466.0) < 1e-9
    assert isinstance(found[0], Seen)


def test_parse_detections():
    text = '[{"label": "沙發", "box_2d": [100, 200, 300, 400]}]'
    assert parse_detections(text) == [Detection('沙發', (100, 200, 300, 400))]


def test_parse_detections_tolerates_mess():
    text = ('好的：```json\n[{"label": "沙發", "box_2d": [100, 200, 300, 400]},'
            ' {"box_2d": [1, 2, 3, 4]}, {"label": "桌子", "box_2d": [1, 2, 3]},'
            ' {"label": "平板", "box_2d": [5, 5, 5, 9]}, {"label": "櫃子", "box_2d": [-5, 0, 1200, 900]}]\n```')
    assert parse_detections(text) == [Detection('沙發', (100, 200, 300, 400)), Detection('櫃子', (0, 0, 1000, 900))]
    for text in ('no objects', '[]', 'not json [1,', '{"label": "x"}', '[1, 2, "a"]'):
        assert parse_detections(text) == [], text


def test_detection_prompt():
    plain, target = detection_prompt(), detection_prompt('洗衣機')
    for prompt in (plain, target):
        assert 'box_2d' in prompt and '1000' in prompt and 'label' in prompt
    assert '洗衣機' in target and '洗衣機' not in plain
    assert 'walls' in plain and 'floor' in plain                 # the room itself is not a thing to remember


def test_depth_from_msg():
    two = np.array([1.5, 2.5], np.float32).tobytes()
    assert np.allclose(depth_from_msg('32FC1', 2, 1, 8, two), [[1.5, 2.5]])
    mm = np.array([1500, 0], np.uint16).tobytes()
    assert np.allclose(depth_from_msg('16UC1', 2, 1, 4, mm), [[1.5, 0.0]])
    padded = two + b'\x00' * 4 + two + b'\x00' * 4
    assert np.allclose(depth_from_msg('32FC1', 2, 2, 12, padded), [[1.5, 2.5], [1.5, 2.5]])
    assert depth_from_msg('rgb8', 2, 1, 6, bytes(6)) is None and depth_from_msg('32FC1', 2, 2, 8, two) is None


def test_encode_jpeg_round_trip():
    rgb = bytes([255, 0, 0]) * 8
    picture = cv2.imdecode(np.frombuffer(encode_jpeg(rgb, 4, 2), np.uint8), cv2.IMREAD_COLOR)
    assert picture.shape == (2, 4, 3)
    b, g, r = picture[0, 0]
    assert r > 200 and g < 60 and b < 60
