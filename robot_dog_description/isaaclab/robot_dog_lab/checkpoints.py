"""Checkpoint and output-folder helpers for export_policy.py (plain Python)."""
import contextlib
import os
import re
import shutil

_MODEL = re.compile(r"^model_(\d+)\.pt$")


def latest_checkpoint(run_dir):
    """File name of the highest-numbered model_<N>.pt in run_dir (numeric, so 1000 beats 999)."""
    found = [(int(m.group(1)), name) for name in os.listdir(run_dir) if (m := _MODEL.match(name))]
    if not found:
        raise FileNotFoundError("no model_<N>.pt in %s" % run_dir)
    return max(found)[1]


def check_out_dir(path, force):
    """Refuse a non-empty output folder unless force, so committed policies are not overwritten."""
    if os.path.isdir(path) and os.listdir(path) and not force:
        raise FileExistsError("%s already has files; pass --force to overwrite" % path)


@contextlib.contextmanager
def staged_dir(path, force):
    """Yield an empty sibling folder; publish it as path only if the block finishes.

    On any failure (also SystemExit) the staging folder is removed and an existing path is left exactly
    as it was, so policy.pt and policy.json are only ever replaced together.
    """
    check_out_dir(path, force)
    path = os.path.abspath(path)
    staging = "%s.staging-%d" % (path, os.getpid())
    shutil.rmtree(staging, ignore_errors=True)
    os.makedirs(staging)
    try:
        yield staging
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    old = None
    if os.path.exists(path):
        old = "%s.old-%d" % (path, os.getpid())
        os.rename(path, old)
    os.rename(staging, path)
    if old:
        shutil.rmtree(old)
