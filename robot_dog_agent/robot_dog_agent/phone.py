"""The robot dog for the phone app: the same agent as the window, served over the home Wi-Fi (spec 2026-11-19).

    ros2 run robot_dog_agent phone [--port 8765] [--model …] [--vision-model …] [--no-robot] [--new-code]
Needs GEMINI_API_KEY, and Isaac Sim (with its camera) and nav.launch.py running unless --no-robot.
Run it instead of `gui`, not with it: both would drive Nav2. Ctrl+C stops the dog and ends everything.
"""
import argparse
import os
import signal
import socket
import sys
import threading

from robot_dog_agent import chat
from robot_dog_agent.phone_code import Guard, load_or_create
from robot_dog_agent.phone_server import PhoneServer, Sources
from robot_dog_agent.robot import FakeRobot
from robot_dog_agent.session import Session

PORT = 8765
PREFIX = {'user': '你：', 'reply': '狗：', 'error': '[錯誤] ', 'log': ''}
PORT_IN_USE = '無法使用 port %d（%s）。是不是已經開了一個 phone？可以用 --port 換一個。'


def lan_address():
    """This computer's address on the home network (the one phones connect to), or '0.0.0.0'."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('192.0.2.1', 9))  # a documentation address: UDP connect sends nothing, it only picks a route
            return s.getsockname()[0]
    except OSError:
        return '0.0.0.0'


def create(args, api_key, code, echo=None):
    """(server, session, feeds to close at the end), serving and starting; the same parts as gui.create.
    echo(kind, text), if given, also gets every session event."""
    session, feeds, latest_frame, latest_path, memory, make_eyes = Session(), [], None, None, None, None
    if args.no_robot:
        from robot_dog_agent.map_view import load_saved_map
        make_robot = FakeRobot
        saved = load_saved_map() + (1,)
        latest_map = lambda: saved  # noqa: E731
    else:
        import rclpy  # only here: --no-robot needs no ROS
        from rclpy.signals import SignalHandlerOptions

        from robot_dog_agent.camera_feed import CameraFeed
        from robot_dog_agent.eyes import Eyes
        from robot_dog_agent.gemini import GeminiClient
        from robot_dog_agent.map_feed import MapFeed
        from robot_dog_agent.nav2_robot import Nav2Robot
        from robot_dog_agent.objects import ObjectMemory
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        camera, maps = CameraFeed(), MapFeed()
        feeds, make_robot = [camera, maps], Nav2Robot
        latest_frame, latest_map, latest_path = camera.latest, maps.latest_map, maps.latest_path
        memory = ObjectMemory()
        vision = GeminiClient(api_key, getattr(args, 'vision_model', chat.VISION_MODEL), '', [],
                              say=lambda text: session.on_event('log', text))
        make_eyes = lambda robot: Eyes(lambda: camera.snapshot(robot.tf_buffer), vision.ask_image, memory)  # noqa: E731
    objects = None if memory is None else (lambda: [(i.name, i.x, i.y) for i in memory.items])
    sources = Sources(latest_frame=latest_frame, latest_map=latest_map, latest_path=latest_path, objects=objects,
                      can_survey=memory is not None)
    try:
        server = PhoneServer(session, Guard(code), sources, port=args.port)
    except OSError:
        for feed in feeds:
            feed.close()
        raise

    def on_event(kind, text):
        server.on_event(kind, text)
        if echo:
            echo(kind, text)
    session.on_event = on_event
    server.start()
    session.start(make_robot, lambda robot, **callbacks: chat.build_agent(
        robot, api_key, args.model, eyes=make_eyes(robot) if make_eyes else None, **callbacks))
    return server, session, feeds


def print_event(kind, text):
    if kind != 'state':
        print(PREFIX.get(kind, '') + text, flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--port', type=int, default=PORT, help='TCP port the phone connects to')
    parser.add_argument('--model', default=chat.DEFAULT_MODEL, help='Gemini model id')
    parser.add_argument('--vision-model', default=chat.VISION_MODEL, help='Gemini model id for the camera pictures')
    parser.add_argument('--no-robot', action='store_true', help='a stand-in robot: no ROS, no simulator, no camera')
    parser.add_argument('--new-code', action='store_true', help='make a new pairing code (phones must enter it again)')
    args, _ = parser.parse_known_args(argv)
    api_key = chat.usable_key(os.environ.get('GEMINI_API_KEY'))
    if not api_key:
        print(chat.BAD_KEY if os.environ.get('GEMINI_API_KEY', '').strip() else chat.NO_KEY)
        return 2
    code = load_or_create(new=args.new_code)
    try:
        server, session, feeds = create(args, api_key, code, echo=print_event)
    except OSError as e:
        print(PORT_IN_USE % (args.port, e.strerror or type(e).__name__))
        return 1
    print('手機連線：http://%s:%d　配對碼 %s' % (lan_address(), server.port, code), flush=True)
    print('（和 gui 二選一；按 Ctrl+C 結束）', flush=True)
    done = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    while not done.wait(0.5):
        pass
    clean = session.shutdown()
    server.close()
    if not clean:
        # a background thread would not end: the start-up still waiting for Nav2, or a command waiting for the model
        os._exit(1)
    for feed in feeds:
        feed.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
