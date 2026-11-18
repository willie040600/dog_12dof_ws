import json
import os

from robot_dog_agent.objects import ObjectMemory
from robot_dog_agent.vision import Seen


def seen(name, x, y, width=1.0):
    return Seen(name, x, y, 2.0, width)


def memory(tmp_path, name='objects.json'):
    now = [100.0]
    return ObjectMemory(str(tmp_path / name), clock=lambda: now[0]), now


def test_add_new_and_ids(tmp_path):
    m, _ = memory(tmp_path)
    assert m.add(seen('沙發', 0.0, 0.0)) == 'obj_1' and m.add(seen('茶几', 3.0, 0.0)) == 'obj_2'
    assert [i.name for i in m.items] == ['沙發', '茶几']


def test_same_name_close_merges_weighted(tmp_path):
    m, now = memory(tmp_path)
    m.add(seen('沙發', 0.0, 0.0, width=1.0))
    m.add(seen('沙發', 0.0, 0.0, width=1.0))
    now[0] = 200.0
    assert m.add(seen('沙發', 0.3, 0.0, width=2.5)) == 'obj_1'
    (item,) = m.items
    assert abs(item.x - 0.1) < 1e-9 and item.seen == 3 and abs(item.width - 1.5) < 1e-9 and item.last_seen == 200.0


def test_same_name_far_does_not_merge(tmp_path):
    m, _ = memory(tmp_path)
    m.add(seen('沙發', 0.0, 0.0))
    m.add(seen('沙發', 0.7, 0.0))
    assert len(m.items) == 2


def test_different_names_do_not_merge(tmp_path):
    m, _ = memory(tmp_path)
    m.add(seen('沙發', 0.0, 0.0))
    m.add(seen('茶几', 0.1, 0.0))
    assert len(m.items) == 2


def test_saved_and_loaded(tmp_path):
    m, _ = memory(tmp_path)
    m.add(seen('沙發', 0.0, 0.0))
    m.add(seen('茶几', 3.0, 0.0))
    again, _ = memory(tmp_path)
    assert again.items == m.items and again.add(seen('床', 5.0, 5.0)) == 'obj_3'


def test_clear_is_saved_and_ids_go_on(tmp_path):
    m, _ = memory(tmp_path)
    m.add(seen('沙發', 0.0, 0.0))
    m.add(seen('茶几', 3.0, 0.0))
    m.clear()
    again, _ = memory(tmp_path)
    assert again.items == [] and again.add(seen('床', 5.0, 5.0)) == 'obj_3'


def test_a_broken_file_starts_empty(tmp_path):
    (tmp_path / 'objects.json').write_text('not json')
    m, _ = memory(tmp_path)
    assert m.items == [] and (tmp_path / 'objects.json').read_text() == 'not json'
    (tmp_path / 'objects.json').write_text(json.dumps({'next': 'x', 'items': [{'id': 1}]}))
    assert memory(tmp_path)[0].items == []


def test_the_folder_is_created(tmp_path):
    m = ObjectMemory(str(tmp_path / 'a' / 'b' / 'objects.json'))
    m.add(seen('沙發', 0.0, 0.0))
    assert os.path.isfile(tmp_path / 'a' / 'b' / 'objects.json')


def test_an_unwritable_path_does_not_raise(tmp_path):
    os.mkdir(tmp_path / 'objects.json')                  # a directory where the file should be
    m, _ = memory(tmp_path)
    assert m.add(seen('沙發', 0.0, 0.0)) == 'obj_1' and len(m.items) == 1


def test_find(tmp_path):
    m, _ = memory(tmp_path)
    m.add(seen('沙發', 0.0, 0.0))
    assert m.find('obj_1').name == '沙發' and m.find('obj_9') is None
