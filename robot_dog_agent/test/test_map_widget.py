import math
import os

os.environ['QT_QPA_PLATFORM'] = 'offscreen'   # before PyQt5 is imported: no window appears

import numpy as np  # noqa: E402
from PyQt5.QtCore import QPoint, Qt  # noqa: E402
from PyQt5.QtTest import QSignalSpy, QTest  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from robot_dog_agent.map_view import load_saved_map  # noqa: E402
from robot_dog_agent.map_widget import NO_MAP, MapWidget  # noqa: E402

APP = QApplication.instance() or QApplication([])
GRID, RES, ORIGIN = load_saved_map()


def widget(size=(600, 360), with_map=True):
    w = MapWidget()
    w.resize(*size)
    w.show()
    if with_map:
        w.set_map(GRID, RES, ORIGIN)
    APP.processEvents()
    return w


def click(w, x, y):
    px, py = w.view.to_pixel(x, y)
    QTest.mouseClick(w, Qt.LeftButton, Qt.NoModifier, QPoint(round(px), round(py)))


def near(got, want, w):
    tolerance = RES / w.view.scale + 1e-9                        # one pixel, in metres
    return abs(got[0] - want[0]) <= tolerance and abs(got[1] - want[1]) <= tolerance


def test_no_map_yet():
    w = widget(with_map=False)
    spy = QSignalSpy(w.clicked)
    w.set_clickable(True)
    QTest.mouseClick(w, Qt.LeftButton, Qt.NoModifier, QPoint(300, 180))
    assert not w.has_map and w.view is None and len(spy) == 0 and NO_MAP
    assert not w.grab().isNull()


def test_click_emits_map_coordinates():
    w = widget()
    spy = QSignalSpy(w.clicked)
    w.set_clickable(True)
    click(w, 2.6, 2.0)
    assert len(spy) == 1 and near(spy[0], (2.6, 2.0), w)


def test_not_clickable_emits_nothing():
    w = widget()
    spy = QSignalSpy(w.clicked)
    click(w, 2.6, 2.0)
    assert len(spy) == 0


def test_click_in_the_margin_emits_nothing():
    w = widget(size=(1200, 200))                                 # far wider than the map: margins left and right
    spy = QSignalSpy(w.clicked)
    w.set_clickable(True)
    assert w.view.left > 10
    QTest.mouseClick(w, Qt.LeftButton, Qt.NoModifier, QPoint(2, 100))
    assert len(spy) == 0


def test_click_after_resize():
    w = widget()
    w.resize(900, 400)
    APP.processEvents()
    spy = QSignalSpy(w.clicked)
    w.set_clickable(True)
    assert w.view.scale == min(900 / GRID.shape[1], 400 / GRID.shape[0])
    click(w, -2.5, -0.3)
    assert len(spy) == 1 and near(spy[0], (-2.5, -0.3), w)


def test_right_button_emits_nothing():
    w = widget()
    spy = QSignalSpy(w.clicked)
    w.set_clickable(True)
    px, py = w.view.to_pixel(2.6, 2.0)
    QTest.mouseClick(w, Qt.RightButton, Qt.NoModifier, QPoint(round(px), round(py)))
    assert len(spy) == 0


def test_cursor_follows_clickable():
    w = widget()
    w.set_clickable(True)
    assert w.cursor().shape() == Qt.PointingHandCursor
    w.set_clickable(False)
    assert w.cursor().shape() == Qt.ArrowCursor


def test_a_new_map_replaces_the_old_one():
    w = widget()
    w.set_map(np.zeros((2, 4), np.int8), 0.5, (-1.0, -1.0))
    assert w.view.width == 4 and w.view.height == 2 and w.view.origin == (-1.0, -1.0)
    assert w.view.to_world(*w.view.to_pixel(0.0, -0.5)) is not None


def test_no_pose_draws_no_dog():
    w = widget()
    w.set_pose(None)
    assert w.pose is None and not w.grab().isNull()


def test_everything_drawn():
    w = widget()
    w.set_pose((0.0, 0.0, math.pi / 2))
    w.set_path([(0.0, 0.0), (1.0, 0.5), (2.6, 2.0)])
    w.set_target((2.6, 2.0))
    assert w.pose == (0.0, 0.0, math.pi / 2) and len(w.path) == 3 and w.target == (2.6, 2.0)
    picture = w.grab().toImage()
    assert not picture.isNull()
    px, py = w.view.to_pixel(0.0, 0.0)
    assert picture.pixelColor(round(px), round(py)).blue() > 150   # the dog is blue
