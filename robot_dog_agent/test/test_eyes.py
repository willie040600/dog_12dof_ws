import numpy as np
import pytest

from robot_dog_agent.eyes import Eyes
from robot_dog_agent.objects import ObjectMemory
from robot_dog_agent.vision import Snapshot, transform_matrix

K = np.array([[466.0, 0.0, 320.0], [0.0, 466.0, 240.0], [0.0, 0.0, 1.0]])
OPTICAL = transform_matrix((0.178, 0.0, 0.28), (-0.5, 0.5, -0.5, 0.5))
ANSWER = '[{"label": "沙發", "box_2d": [375, 375, 625, 625]}]'


def picture():
    depth = np.full((480, 640), 2.0, np.float32)
    return Snapshot(bytes(640 * 480 * 3), 640, 480, depth, K, OPTICAL)


def eyes(tmp_path, answer=ANSWER, snap=picture):
    asked = []

    def ask_image(prompt, jpeg):
        asked.append((prompt, jpeg))
        if isinstance(answer, BaseException):
            raise answer
        return answer
    return Eyes(snap, ask_image, ObjectMemory(str(tmp_path / 'objects.json'))), asked


def test_look_sends_the_picture_and_remembers(tmp_path):
    e, asked = eyes(tmp_path)
    (seen,) = e.look()
    assert seen.name == '沙發' and abs(seen.x - 2.178) < 1e-6 and abs(seen.y) < 1e-6
    assert [i.name for i in e.memory.items] == ['沙發']
    prompt, jpeg = asked[0]
    assert jpeg[:2] == b'\xff\xd8' and 'box_2d' in prompt


def test_look_without_a_picture(tmp_path):
    e, _ = eyes(tmp_path, snap=lambda: None)
    with pytest.raises(RuntimeError, match='no camera image yet'):
        e.look()


def test_look_with_a_target_uses_it_in_the_prompt(tmp_path):
    e, asked = eyes(tmp_path)
    e.look(target='洗衣機')
    assert '洗衣機' in asked[0][0]


def test_look_lets_model_errors_through(tmp_path):
    e, _ = eyes(tmp_path, answer=ValueError('quota'))
    with pytest.raises(ValueError, match='quota'):
        e.look()
    assert e.memory.items == []
