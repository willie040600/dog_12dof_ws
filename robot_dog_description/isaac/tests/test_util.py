import pytest

from robot_dog_sim.util import require_ros_jazzy, try_step


def test_require_ros_jazzy_exits_when_unsourced(monkeypatch, capsys):
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    with pytest.raises(SystemExit) as e:
        require_ros_jazzy()
    assert e.value.code == 2
    assert "source /opt/ros/jazzy/setup.bash" in capsys.readouterr().out


def test_require_ros_jazzy_rejects_other_distro(monkeypatch):
    monkeypatch.setenv("ROS_DISTRO", "humble")
    with pytest.raises(SystemExit):
        require_ros_jazzy()


def test_require_ros_jazzy_ok(monkeypatch):
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    require_ros_jazzy()  # no exception


def test_try_step_reports_ok_and_failed(capsys):
    assert try_step("good", lambda: None) is True
    assert try_step("bad", lambda: 1 / 0) is False
    out = capsys.readouterr().out
    assert "good" in out and "ok" in out
    assert "bad" in out and "FAILED" in out
