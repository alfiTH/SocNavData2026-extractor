"""Resolves the on-disk layout of a scene under experiments/<scene>/,
transparently unzipping the *_annotated.zip / *_poses.zip / *_robot.zip
archives the first time a given piece is needed."""

import zipfile
from pathlib import Path


def scene_tag(scene: int) -> str:
    return f"{scene:02d}"


class ExperimentPaths:
    def __init__(self, scene: int, experiments_root: Path):
        self.scene = scene
        self.tag = scene_tag(scene)
        self.root = Path(experiments_root) / str(scene)
        if not self.root.is_dir():
            raise FileNotFoundError(f"No such experiment directory: {self.root}")

    def _ensure_dir(self, dirname: str) -> Path:
        target = self.root / dirname
        if target.is_dir():
            return target
        zip_path = self.root / f"{dirname}.zip"
        if not zip_path.is_file():
            raise FileNotFoundError(f"Missing both {target} and {zip_path}")
        print(f"Extracting {zip_path} -> {self.root}")
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(self.root)
        return target

    def _ensure_file(self, filename: str, zip_stem: str) -> Path:
        target = self.root / filename
        if target.is_file():
            return target
        zip_path = self.root / f"{zip_stem}.zip"
        if not zip_path.is_file():
            raise FileNotFoundError(f"Missing both {target} and {zip_path}")
        print(f"Extracting {zip_path} -> {self.root}")
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(self.root)
        if not target.is_file():
            raise FileNotFoundError(f"{zip_path} did not contain {filename}")
        return target

    @property
    def annotated_dir(self) -> Path:
        return self._ensure_dir(f"{self.tag}_annotated")

    @property
    def poses_dir(self) -> Path:
        return self._ensure_dir(f"{self.tag}_poses")

    @property
    def robot_bag(self) -> Path:
        return self._ensure_file(f"{self.scene}_robot.bag", f"{self.tag}_robot")

    @property
    def static_map_json(self) -> Path:
        return self.root / f"{self.scene}_static_map.json"

    @property
    def walls_override_json(self) -> Path:
        return self.root / f"{self.scene}_walls_override.json"

    @property
    def skipped_frames_json(self) -> Path:
        return self.root / f"{self.scene}_skipped_frames.json"

    @property
    def robot_pose_csv(self) -> Path:
        return self.annotated_dir / f"{self.scene}_robot_pose.csv"

    @property
    def grs_to_bot_offset_json(self) -> Path:
        return self.annotated_dir / f"{self.scene}_grs_to_bot_offset.json"

    @property
    def key_id_map_json(self) -> Path:
        return self.annotated_dir / "key_id_map.json"

    @property
    def ann_dir(self) -> Path:
        return self.annotated_dir / f"scene_{self.scene}" / "ann"
