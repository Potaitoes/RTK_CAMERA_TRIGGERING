#!/usr/bin/env python3
"""
Complete pipeline to filter camera frames and tm2 edges, match them, and prepare dataset.

Steps:
1. Filter frames.csv by delta_t_prev_us (25-35 ms)
2. Convert tm2.csv rising edges to UTC timestamps (no leap seconds)
3. Match filtered frames to tm2 rising edges by receiver timebase
4. Copy matched frames to a new directory
5. Create cameraTimes.csv with UTC timestamps of matched frames
"""

from __future__ import annotations

import csv
import shutil
from pathlib import Path

GPS_UNIX_EPOCH = 315964800  # 1980-01-06 00:00:00 UTC
GPS_WEEK_SECONDS = 604800
NS_TO_S = 1e-9


def gps_tow_to_unix_seconds(wn: int, tow_ms: int, tow_sub_ms_ns: int) -> float:
    """Convert GPS week + TOW fields to UTC-like seconds (no leap second adjustment)."""
    return (
        GPS_UNIX_EPOCH
        + wn * GPS_WEEK_SECONDS
        + tow_ms / 1000.0
        + tow_sub_ms_ns * NS_TO_S
    )


def filter_frames_by_delta(
    frames_csv: Path,
    min_delta_us: int,
    max_delta_us: int,
) -> list[dict[str, str]]:
    """Filter frames by delta_t_prev_us between min and max."""
    filtered = []
    with frames_csv.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                delta_us = int(row.get("delta_t_prev_us", "0") or 0)
                if min_delta_us <= delta_us <= max_delta_us:
                    filtered.append(row)
            except ValueError:
                continue
    return filtered


def convert_tm2_rising_edges(tm2_csv: Path) -> list[dict[str, str | float]]:
    """Convert tm2 rising edges to UTC timestamps."""
    edges = []
    with tm2_csv.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                wn_r = int(row["wnR"])
                tow_ms_r = int(row["towMsR"])
                tow_sub_ms_r = int((row.get("towSubMsR") or "0").strip())
                arrival_us = int(row["arrival_us"])
                
                rising_utc_s = gps_tow_to_unix_seconds(wn_r, tow_ms_r, tow_sub_ms_r)
                
                edges.append({
                    "tm2_index": row["tm2_index"],
                    "arrival_us": arrival_us,
                    "rising_edge_utc_s": rising_utc_s,
                })
            except (ValueError, KeyError):
                continue
    return edges


def match_frames_to_tm2(
    filtered_frames: list[dict[str, str]],
    tm2_edges: list[dict[str, str | float]],
) -> list[dict[str, str]]:
    """Match filtered frames to TM2 edges by closest receiver time."""
    matched = []
    
    for tm2 in tm2_edges:
        tm2_time_us = tm2["arrival_us"]
        
        best_frame = None
        best_distance_us = float("inf")
        
        for frame in filtered_frames:
            frame_time_us = int(frame["recv_time_us"])
            distance_us = abs(tm2_time_us - frame_time_us)
            
            if distance_us < best_distance_us:
                best_distance_us = distance_us
                best_frame = frame
        
        if best_frame is not None:
            matched.append({
                "tm2_index": str(tm2["tm2_index"]),
                "tm2_arrival_us": str(tm2["arrival_us"]),
                "rising_edge_utc_s": f"{tm2['rising_edge_utc_s']:.16e}",
                "frame_index": best_frame["frame_index"],
                "frame_filename": best_frame["filename"],
                "frame_recv_time_us": best_frame["recv_time_us"],
                "time_diff_us": f"{best_distance_us}",
                "brightness": best_frame.get("brightness", ""),
            })
    
    return matched


def copy_matched_frames(
    matched: list[dict[str, str]],
    src_image_dir: Path,
    dst_image_dir: Path,
) -> int:
    """Copy matched frame files to destination directory."""
    dst_image_dir.mkdir(parents=True, exist_ok=True)
    
    unique_frames = set(row["frame_filename"] for row in matched)
    copied = 0
    
    for filename in unique_frames:
        src_file = src_image_dir / filename
        dst_file = dst_image_dir / filename
        if src_file.exists():
            shutil.copy2(src_file, dst_file)
            copied += 1
    
    return copied


def write_camera_times(
    matched: list[dict[str, str]],
    output_csv: Path,
) -> int:
    """Write cameraTimes.csv with just the UTC timestamps in order."""
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["rising_edge_utc_s"])
        writer.writeheader()
        for row in matched:
            writer.writerow({"rising_edge_utc_s": row["rising_edge_utc_s"]})
    
    return len(matched)


def main() -> None:
    # Hardcoded paths
    frames_csv = Path("/home/vitom/recordSample/parrot_ts7/frames.csv")
    tm2_csv = Path("/home/vitom/recordSample/parrot_ts7/tm2.csv")
    src_image_dir = Path("/home/vitom/recordSample/parrot_ts7/images/camera1")
    output_dir = Path("/home/vitom/recordSample/parrot_ts7/output")
    
    # Delta time filters (in microseconds)
    min_delta_us = 25000
    max_delta_us = 35000
    
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Step 1: Filtering frames by delta_t_prev_us...")
    filtered_frames = filter_frames_by_delta(
        frames_csv,
        min_delta_us,
        max_delta_us,
    )
    print(f"  Filtered to {len(filtered_frames)} frames")

    print("Step 2: Converting TM2 rising edges to UTC...")
    tm2_edges = convert_tm2_rising_edges(tm2_csv)
    print(f"  Converted {len(tm2_edges)} TM2 edges")

    print("Step 3: Matching frames to TM2 edges...")
    matched = match_frames_to_tm2(filtered_frames, tm2_edges)
    print(f"  Matched {len(matched)} pairs")

    print("Step 4: Copying matched frame images...")
    dst_image_dir = output_dir / "camera1_matched"
    copied = copy_matched_frames(matched, src_image_dir, dst_image_dir)
    print(f"  Copied {copied} unique frames to {dst_image_dir}")

    print("Step 5: Writing cameraTimes.csv...")
    camera_times_csv = output_dir / "cameraTimes.csv"
    wrote = write_camera_times(matched, camera_times_csv)
    print(f"  Wrote {wrote} timestamps to {camera_times_csv}")

    # Also write the full matched data for reference
    matched_csv = output_dir / "frames_tm2_matched.csv"
    with matched_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "tm2_index",
                "tm2_arrival_us",
                "rising_edge_utc_s",
                "frame_index",
                "frame_filename",
                "frame_recv_time_us",
                "time_diff_us",
                "brightness",
            ],
        )
        writer.writeheader()
        writer.writerows(matched)
    print(f"  Wrote full match data to {matched_csv}")

    print("\nPipeline complete!")
    print(f"  Output directory: {output_dir}")


if __name__ == "__main__":
    main()
