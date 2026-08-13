"""Default experiments/dataset roots, shared by every script here. Inside
ros_container the volume mounts land at /root/experiments and /root/dataset;
outside it (plain host python3) everything is resolved relative to this
file's location instead."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def _is_dir(path: Path) -> bool:
    try:
        return path.is_dir()
    except PermissionError:
        return False


def default_experiments_root() -> Path:
    p = Path("/root/experiments")
    return p if _is_dir(p) else REPO_ROOT / "experiments"


def default_out_root() -> Path:
    p = Path("/root/dataset")
    if _is_dir(p):
        return p / "unlabeled" / "ros_extractor"
    return REPO_ROOT / "dataset" / "unlabeled" / "ros_extractor"
