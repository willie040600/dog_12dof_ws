"""The chat with the robot dog as a window, with its camera image (spec 2026-11-11).

    ros2 run robot_dog_agent gui [--model gemini-3.5-flash-lite] [--no-robot]
Needs GEMINI_API_KEY, and Isaac Sim (with its camera: no --no-camera) and nav.launch.py running unless --no-robot.
The same agent as the terminal chat; Stop cancels a navigation, closing the window ends everything.
"""
import argparse
import os
import signal
import sys
import time

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPlainTextEdit,
                             QPushButton, QSizePolicy, QVBoxLayout, QWidget)

from robot_dog_agent import chat, places
from robot_dog_agent.robot import FakeRobot
from robot_dog_agent.session import Session, pose_text

NO_CAMERA = '沒有相機畫面（Isaac Sim 是不是加了 --no-camera？）'
NO_ROBOT_CAMERA = '--no-robot：沒有相機'
FRAME_PERIOD_MS, STATUS_PERIOD_MS = 100, 500
STALE_AFTER = 3.0  # s; a frame older than this is no longer shown
PREFIX = {'user': '你：', 'reply': '狗：', 'error': '[錯誤] ', 'log': ''}


class MainWindow(QMainWindow):
    """session: a session.Session. latest_frame() -> (width, height, RGB bytes, arrival time by `clock`) or None;
    None instead of a function when there is no camera. Everything here runs in the Qt main thread; other threads
    reach it through the `events` signal only."""

    events = pyqtSignal(str, str)  # (kind, text) of Session.on_event

    def __init__(self, session, latest_frame=None, no_camera_text=NO_CAMERA, clock=time.monotonic):
        super().__init__()
        self.session, self.latest_frame, self.no_camera_text, self.clock = session, latest_frame, no_camera_text, clock
        self.clean_shutdown, self._closed, self._shown_stamp = None, False, None
        self.setWindowTitle('機器狗')
        self.setMinimumSize(1000, 600)

        self.image_label = QLabel(no_camera_text)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(480, 360)
        self.image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)  # the picture never sizes the window
        self.image_label.setStyleSheet('background: #202020; color: #c0c0c0;')

        self.place_buttons = []
        grid = QGridLayout()
        for i, place in enumerate(places.PLACES):
            name = place.name.split('（')[0]
            button = QPushButton(name)
            button.clicked.connect(lambda _checked=False, name=name: self.session.submit('去' + name))
            self.place_buttons.append(button)
            grid.addWidget(button, i // 4, i % 4)
        self.stop_button = QPushButton('停 止')
        self.stop_button.setMinimumHeight(40)
        self.stop_button.setStyleSheet('QPushButton:enabled { background: #c62828; color: white; font-weight: bold; }')
        self.stop_button.clicked.connect(lambda: self.session.stop())

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.input = QLineEdit()
        self.input.setPlaceholderText('輸入指令，例如：去廚房')
        self.input.returnPressed.connect(self._send)
        self.send_button = QPushButton('送出')
        self.send_button.clicked.connect(self._send)

        left = QVBoxLayout()
        left.addWidget(self.image_label, 1)
        left.addLayout(grid)
        left.addWidget(self.stop_button)
        entry = QHBoxLayout()
        entry.addWidget(self.input, 1)
        entry.addWidget(self.send_button)
        right = QVBoxLayout()
        right.addWidget(self.log, 1)
        right.addLayout(entry)
        columns = QHBoxLayout()
        columns.addLayout(left, 3)
        columns.addLayout(right, 2)
        central = QWidget()
        central.setLayout(columns)
        self.setCentralWidget(central)

        self.status_left, self.status_right = QLabel(), QLabel()
        self.statusBar().addWidget(self.status_left, 1)
        self.statusBar().addPermanentWidget(self.status_right)

        self.events.connect(self.on_event)
        self._apply_state(session.state)
        self.refresh_status()
        self._timers = []
        for period, slot in ((FRAME_PERIOD_MS, self.refresh_frame), (STATUS_PERIOD_MS, self.refresh_status)):
            timer = QTimer(self)
            timer.timeout.connect(slot)
            timer.start(period)
            self._timers.append(timer)

    def _send(self):
        if self.session.submit(self.input.text()):
            self.input.clear()

    def _apply_state(self, state):
        """idle: the user may type; busy: only Stop; starting / failed: nothing."""
        for widget in [self.input, self.send_button] + self.place_buttons:
            widget.setEnabled(state == 'idle')
        self.stop_button.setEnabled(state == 'busy')
        if state == 'idle':
            self.input.setFocus()

    def on_event(self, kind, text):
        """A Session event, in the main thread."""
        if self._closed:
            return
        if kind == 'state':
            self._apply_state(text)
            self.refresh_status()
        else:
            self.log.appendPlainText(PREFIX.get(kind, '') + text)
            self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def refresh_frame(self):
        """Show the newest camera frame, or the hint when there is none worth showing."""
        if self._closed:
            return
        frame = self.latest_frame() if self.latest_frame else None
        if frame is not None:
            width, height, rgb, stamp = frame
            if self.clock() - stamp <= STALE_AFTER and len(rgb) == width * height * 3:
                if stamp != self._shown_stamp:
                    image = QImage(rgb, width, height, width * 3, QImage.Format_RGB888).copy()  # own the pixels
                    self.image_label.setPixmap(QPixmap.fromImage(image).scaled(
                        self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
                    self._shown_stamp = stamp
                return
        self._shown_stamp = None
        if self.image_label.text() != self.no_camera_text:
            self.image_label.setText(self.no_camera_text)  # also drops the picture

    def refresh_status(self):
        if self._closed:
            return
        self.status_left.setText(pose_text(self.session.pose()))
        self.status_right.setText(self.session.status_text())

    def closeEvent(self, event):
        if not self._closed:
            self._closed = True
            for timer in self._timers:
                timer.stop()
            self.clean_shutdown = self.session.shutdown()
        event.accept()


def create(args, api_key):
    """(window, session, camera feed or None), wired up and starting. A QApplication must exist."""
    session, feed = Session(), None
    if args.no_robot:
        make_robot, hint = FakeRobot, NO_ROBOT_CAMERA
    else:
        import rclpy  # only here: --no-robot needs no ROS
        from rclpy.signals import SignalHandlerOptions

        from robot_dog_agent.camera_feed import CameraFeed
        from robot_dog_agent.nav2_robot import Nav2Robot
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        feed, make_robot, hint = CameraFeed(), Nav2Robot, NO_CAMERA
    window = MainWindow(session, latest_frame=feed.latest if feed else None, no_camera_text=hint)
    session.on_event = window.events.emit
    session.start(make_robot, lambda robot, **callbacks: chat.build_agent(robot, api_key, args.model, **callbacks))
    return window, session, feed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default=chat.DEFAULT_MODEL, help='Gemini model id')
    parser.add_argument('--no-robot', action='store_true', help='a stand-in robot: no ROS, no simulator, no camera')
    args, _ = parser.parse_known_args(argv)
    api_key = chat.usable_key(os.environ.get('GEMINI_API_KEY'))
    if not api_key:
        print(chat.BAD_KEY if os.environ.get('GEMINI_API_KEY', '').strip() else chat.NO_KEY)
        return 2
    app = QApplication.instance() or QApplication(sys.argv[:1])
    window, session, feed = create(args, api_key)
    # Ctrl+C in the terminal closes the window like its close button; the timers let Python run the handler
    signal.signal(signal.SIGINT, lambda *_: window.close())
    window.show()
    app.exec_()
    window.close()  # no-op when the user closed it; needed when the event loop ended another way
    if not window.clean_shutdown:
        # a background thread would not end: the start-up still waiting for Nav2 (nothing was ever sent), or a
        # command still waiting for the model (the robot refuses every goal after the cancel that shutdown set)
        os._exit(1)
    if feed is not None:
        feed.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
