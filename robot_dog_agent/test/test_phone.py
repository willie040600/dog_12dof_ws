import argparse
import ipaddress
import json
import os
import subprocess
import sys
import time
import urllib.request

from robot_dog_agent import phone

HERE = os.path.dirname(os.path.abspath(__file__))


def test_lan_address_is_an_ip():
    ipaddress.IPv4Address(phone.lan_address())


def test_no_key_exits_2(capsys, monkeypatch):
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    assert phone.main([]) == 2
    assert 'GEMINI_API_KEY' in capsys.readouterr().out


def get(port, path, code='123456'):
    request = urllib.request.Request('http://127.0.0.1:%d%s' % (port, path),
                                     headers={'Authorization': 'Bearer ' + code})
    with urllib.request.urlopen(request, timeout=5) as r:
        return r.status, r.read()


def test_no_robot_server_works():
    args = argparse.Namespace(model='m', vision_model='m', no_robot=True, port=0)
    server, session, feeds = phone.create(args, 'k', '123456')
    try:
        for _ in range(50):
            if session.state == 'idle':
                break
            time.sleep(0.05)
        assert session.state == 'idle' and feeds == []
        status, data = get(server.port, '/api/hello')
        hello = json.loads(data)
        assert status == 200 and hello['camera'] is False and hello['can_survey'] is False
        assert get(server.port, '/api/map.png')[0] == 200
        state = json.loads(get(server.port, '/api/state')[1])
        assert state['pose'] == [0.0, 0.0, 0.0] and state['map']['version'] == 1
        events = json.loads(get(server.port, '/api/events?after=0')[1])['events']
        assert [e['kind'] for e in events] == ['state', 'log']  # idle, then "可以開始了"
    finally:
        assert session.shutdown() is True
        server.close()


def test_module_does_not_import_ros_or_qt():
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([os.path.join(HERE, '..'),
                                                       os.path.join(HERE, '..', '..', 'robot_dog_navigation'),
                                                       os.environ.get('PYTHONPATH', '')]))
    out = subprocess.run([sys.executable, '-c', 'import sys, robot_dog_agent.phone; '
                          'print(sorted(m for m in ("rclpy", "PyQt5", "robot_dog_agent.gui") if m in sys.modules))'],
                         env=env, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == '[]'
