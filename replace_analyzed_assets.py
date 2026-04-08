#!/usr/bin/env python3
"""Replace Analyzed/files cameraTimes and images with generated output versions."""

from __future__ import annotations

import shutil
from pathlib import Path


def replace_dir(src: Path, dst: Path) -> None:
    """Replace destination directory with source directory contents."""
    if not src.exists() or not src.is_dir():
        raise FileNotFoundError(f"Source directory not found: {src}")

    if dst.exists():
        if dst.is_dir():
            shutil.rmtree(dst)
        else:
            dst.unlink()

    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src, dst)


def main() -> None:
    base_dir = Path(__file__).resolve().parent

    src_camera_times = base_dir / "output" / "cameraTimes"
    src_images = base_dir / "output" / "images"

    dst_camera_times = base_dir / "Analyzed" / "files" / "cameraTimes"
    dst_images = base_dir / "Analyzed" / "files" / "images"

    replace_dir(src_camera_times, dst_camera_times)
    print(f"Replaced: {dst_camera_times}")

    replace_dir(src_images, dst_images)
    print(f"Replaced: {dst_images}")

    print("Done.")


if __name__ == "__main__":
    main()
