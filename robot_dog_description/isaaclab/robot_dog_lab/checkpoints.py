"""Checkpoint and output-folder helpers for export_policy.py (plain Python)."""
import os
import re

_MODEL = re.compile(r"^model_(\d+)\.pt$")


def latest_checkpoint(run_dir):
    """File name of the highest-numbered model_<N>.pt in run_dir (numeric, so 1000 beats 999)."""
    found = [(int(m.group(1)), name) for name in os.listdir(run_dir) if (m := _MODEL.match(name))]
    if not found:
        raise FileNotFoundError("no model_<N>.pt in %s" % run_dir)
    return max(found)[1]


def prepare_out_dir(path, force):
    """Create path; refuse to reuse a non-empty one unless force, so committed policies are not overwritten."""
    if os.path.isdir(path) and os.listdir(path) and not force:
        raise FileExistsError("%s already has files; pass --force to overwrite" % path)
    os.makedirs(path, exist_ok=True)
