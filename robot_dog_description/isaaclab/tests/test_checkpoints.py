import os

import pytest

from robot_dog_lab.checkpoints import check_out_dir, latest_checkpoint, staged_dir


def test_latest_checkpoint_is_numeric(tmp_path):
    for n in (50, 999, 1000):
        (tmp_path / f"model_{n}.pt").write_bytes(b"")
    assert latest_checkpoint(str(tmp_path)) == "model_1000.pt"


def test_latest_checkpoint_ignores_other_files(tmp_path):
    (tmp_path / "model_7.pt").write_bytes(b"")
    (tmp_path / "model_best.pt").write_bytes(b"")
    (tmp_path / "events.out").write_bytes(b"")
    assert latest_checkpoint(str(tmp_path)) == "model_7.pt"


def test_latest_checkpoint_empty_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        latest_checkpoint(str(tmp_path))


def test_check_out_dir_refuses_existing(tmp_path):
    (tmp_path / "policy.json").write_text("{}")
    with pytest.raises(FileExistsError, match="--force"):
        check_out_dir(str(tmp_path), force=False)


def test_check_out_dir_force(tmp_path):
    (tmp_path / "policy.json").write_text("{}")
    check_out_dir(str(tmp_path), force=True)


def test_check_out_dir_does_not_create(tmp_path):
    check_out_dir(str(tmp_path / "new"), force=False)
    assert not (tmp_path / "new").exists()  # a failed export must not leave an empty folder


def _leftovers(tmp_path):
    return sorted(p for p in os.listdir(tmp_path) if p != "out")


def test_staged_dir_publishes_on_success(tmp_path):
    out = tmp_path / "out"
    with staged_dir(str(out), force=False) as tmp:
        assert not out.exists()
        open(os.path.join(tmp, "policy.json"), "w").write("{}")
    assert (out / "policy.json").read_text() == "{}"
    assert _leftovers(tmp_path) == []


def test_staged_dir_leaves_nothing_on_failure(tmp_path):
    out = tmp_path / "out"
    with pytest.raises(RuntimeError):
        with staged_dir(str(out), force=False) as tmp:
            open(os.path.join(tmp, "policy.pt"), "w").write("weights")
            raise RuntimeError("parity check failed")
    assert not out.exists()
    assert _leftovers(tmp_path) == []


def test_staged_dir_cleans_up_on_system_exit(tmp_path):
    out = tmp_path / "out"
    with pytest.raises(SystemExit):
        with staged_dir(str(out), force=False) as tmp:
            open(os.path.join(tmp, "policy.pt"), "w").write("weights")
            raise SystemExit(1)
    assert not out.exists()
    assert _leftovers(tmp_path) == []


def test_staged_dir_keeps_old_folder_on_failure(tmp_path):
    out = tmp_path / "out"
    out.mkdir()
    (out / "policy.json").write_text("old")
    (out / "policy.pt").write_text("old weights")
    with pytest.raises(RuntimeError):
        with staged_dir(str(out), force=True) as tmp:
            open(os.path.join(tmp, "policy.pt"), "w").write("new weights")
            raise RuntimeError("validation failed")
    assert (out / "policy.json").read_text() == "old"
    assert (out / "policy.pt").read_text() == "old weights"
    assert _leftovers(tmp_path) == []


def test_staged_dir_replaces_old_folder_with_force(tmp_path):
    out = tmp_path / "out"
    out.mkdir()
    (out / "stale.txt").write_text("old")
    with staged_dir(str(out), force=True) as tmp:
        open(os.path.join(tmp, "policy.json"), "w").write("new")
    assert sorted(os.listdir(out)) == ["policy.json"]
    assert _leftovers(tmp_path) == []


def test_staged_dir_refuses_existing_without_force(tmp_path):
    out = tmp_path / "out"
    out.mkdir()
    (out / "policy.json").write_text("old")
    with pytest.raises(FileExistsError, match="--force"):
        with staged_dir(str(out), force=False):
            pass
    assert (out / "policy.json").read_text() == "old"
    assert _leftovers(tmp_path) == []
