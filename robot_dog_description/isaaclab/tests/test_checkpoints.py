import pytest

from robot_dog_lab.checkpoints import latest_checkpoint, prepare_out_dir


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


def test_prepare_out_dir_creates(tmp_path):
    prepare_out_dir(str(tmp_path / "a" / "b"), force=False)
    assert (tmp_path / "a" / "b").is_dir()


def test_prepare_out_dir_refuses_existing(tmp_path):
    (tmp_path / "policy.json").write_text("{}")
    with pytest.raises(FileExistsError, match="--force"):
        prepare_out_dir(str(tmp_path), force=False)


def test_prepare_out_dir_force(tmp_path):
    (tmp_path / "policy.json").write_text("{}")
    prepare_out_dir(str(tmp_path), force=True)
