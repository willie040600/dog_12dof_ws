"""The map in the window: the apartment, the places, the dog, its path; a click sends the dog (spec 2026-11-17).

Qt only (no ROS): the window feeds it the map, the pose and the path, and listens to `clicked`.
"""
import math

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget

from robot_dog_agent import places
from robot_dog_agent.map_view import MapView, grid_to_gray

NO_MAP = '等地圖…（nav.launch.py 開了嗎？）'
DOG, PATH, TARGET, PLACE = QColor('#1565c0'), QColor('#2e7d32'), QColor('#d32f2f'), QColor('#6d4c41')
THING = QColor('#ef6c00')  # a remembered thing (orange)
DOG_SIZE = 0.35  # m from the dog's centre to the tip of its triangle


class MapWidget(QWidget):
    clicked = pyqtSignal(float, float)  # map coordinates of a left click on the map, while clickable

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._image, self._map = None, None  # QImage of the map; (resolution, origin, width, height)
        self.pose, self.path, self.target, self.clickable, self.objects = None, [], None, False, []

    @property
    def has_map(self):
        return self._map is not None

    @property
    def view(self):
        """The MapView for the current size, or None before the first map."""
        if self._map is None:
            return None
        resolution, origin, width, height = self._map
        return MapView(resolution, origin, width, height, self.width(), self.height())

    def set_map(self, grid, resolution, origin):
        height, width = grid.shape
        self._image = QImage(grid_to_gray(grid), width, height, width, QImage.Format_Grayscale8).copy()
        self._map = (resolution, tuple(origin), width, height)
        self.update()

    def set_pose(self, pose):
        if pose != self.pose:
            self.pose = pose
            self.update()

    def set_path(self, points):
        points = list(points)
        if points != self.path:
            self.path = points
            self.update()

    def set_objects(self, objects):
        """[(name, x, y), ...] of the remembered things."""
        objects = list(objects)
        if objects != self.objects:
            self.objects = objects
            self.update()

    def set_target(self, target):
        if target != self.target:
            self.target = target
            self.update()

    def set_clickable(self, clickable):
        self.clickable = clickable
        self.setCursor(Qt.PointingHandCursor if clickable else Qt.ArrowCursor)

    def mousePressEvent(self, event):
        view = self.view
        if not self.clickable or view is None or event.button() != Qt.LeftButton:
            return
        point = view.to_world(event.x(), event.y())
        if point is not None:  # to the centimetre: the map has 5 cm cells, and the coordinates are shown to the user
            self.clicked.emit(round(point[0], 2), round(point[1], 2))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#e0e0e0'))
        view = self.view
        if view is None:
            painter.setPen(QColor('#606060'))
            painter.drawText(self.rect(), Qt.AlignCenter, NO_MAP)
            return
        painter.setRenderHint(QPainter.Antialiasing)
        painter.drawImage(QRectF(view.left, view.top, view.width * view.scale, view.height * view.scale), self._image)
        pixels_per_m = view.scale / view.resolution

        painter.setFont(QFont(painter.font().family(), 8))
        for place in places.PLACES:
            px, py = view.to_pixel(place.x, place.y)
            painter.setPen(Qt.NoPen)
            painter.setBrush(PLACE)
            painter.drawEllipse(QPointF(px, py), 3.5, 3.5)
            painter.setPen(PLACE)
            painter.drawText(QPointF(px + 5, py - 4), place.name.split('（')[0])

        for name, x, y in self.objects:
            px, py = view.to_pixel(x, y)
            painter.setPen(Qt.NoPen)
            painter.setBrush(THING)
            painter.drawRect(QRectF(px - 4.5, py - 4.5, 9, 9))
            painter.setPen(THING)
            painter.drawText(QPointF(px + 6, py + 12), name)

        if len(self.path) >= 2:
            painter.setPen(QPen(PATH, 2.5))
            painter.setBrush(Qt.NoBrush)
            painter.drawPolyline(QPolygonF([QPointF(*view.to_pixel(x, y)) for x, y in self.path]))

        if self.target is not None:
            px, py = view.to_pixel(*self.target)
            painter.setPen(QPen(TARGET, 3))
            for dx, dy in ((7, 7), (7, -7)):
                painter.drawLine(QPointF(px - dx, py - dy), QPointF(px + dx, py + dy))

        if self.pose is not None:
            x, y, yaw = self.pose
            size = max(DOG_SIZE * pixels_per_m, 9.0)
            px, py = view.to_pixel(x, y)
            corners = []
            for angle, length in ((0.0, size), (2.5, size * 0.7), (-2.5, size * 0.7)):
                corners.append(QPointF(px + length * math.cos(yaw + angle), py - length * math.sin(yaw + angle)))
            painter.setPen(QPen(QColor('white'), 1.5))
            painter.setBrush(QBrush(DOG))
            painter.drawPolygon(QPolygonF(corners))
            painter.drawEllipse(QPointF(px, py), 2.0, 2.0)
