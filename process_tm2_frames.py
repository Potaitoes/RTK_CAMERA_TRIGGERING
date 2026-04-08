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
from bisect import bisect_left
from pathlib import Path

GPS_UNIX_EPOCH = 315964800  # 1980-01-06 00:00:00 UTC
GPS_WEEK_SECONDS = 604800
NS_TO_S = 1e-9
EXPOSURE_CENTER_OFFSET_S = 0.030  # subtract 30 ms from each camera timestamp


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
    """Filter frames by frame-to-frame delta from consecutive `recv_time_us`."""
    filtered = []
    prev_recv_time_us: int | None = None
    with frames_csv.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                recv_time_us = int(row["recv_time_us"])
                if prev_recv_time_us is None:
                    prev_recv_time_us = recv_time_us
                    continue

                delta_us = recv_time_us - prev_recv_time_us
                prev_recv_time_us = recv_time_us

                if min_delta_us <= delta_us <= max_delta_us:
                    row_with_delta = dict(row)
                    row_with_delta["actual_delta_us"] = str(delta_us)
                    filtered.append(row_with_delta)
            except ValueError:
                continue
            except KeyError:
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
    max_time_diff_us: int,
) -> list[dict[str, str]]:
    """Match filtered frames to TM2 edges by closest receiver time without reuse."""
    matched = []
    frame_times_us = [int(frame["recv_time_us"]) for frame in filtered_frames]
    next_frame_start = 0

    for tm2 in tm2_edges:
        if next_frame_start >= len(filtered_frames):
            break

        tm2_time_us = int(tm2["arrival_us"])
        insertion_index = bisect_left(frame_times_us, tm2_time_us, lo=next_frame_start)

        candidate_indexes = []
        # Consider both neighbors around insertion index and pick the closest,
        # regardless of being before or after TM2 arrival.
        if insertion_index < len(filtered_frames):
            candidate_indexes.append(insertion_index)
        if insertion_index - 1 >= next_frame_start:
            candidate_indexes.append(insertion_index - 1)

        if not candidate_indexes:
            continue

        best_index = min(
            candidate_indexes,
            key=lambda index: abs(frame_times_us[index] - tm2_time_us),
        )
        best_distance_us = abs(frame_times_us[best_index] - tm2_time_us)

        if best_distance_us > max_time_diff_us:
            if frame_times_us[best_index] < tm2_time_us:
                next_frame_start = best_index + 1
            continue

        best_frame = filtered_frames[best_index]
        next_frame_start = best_index + 1

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
        file_name_only = Path(filename).name
        src_file = src_image_dir / filename
        if not src_file.exists():
            src_file = src_image_dir / file_name_only

        dst_file = dst_image_dir / file_name_only
        if src_file.exists():
            shutil.copy2(src_file, dst_file)
            copied += 1
    
    return copied


def write_camera_times(
    matched: list[dict[str, str]],
    output_csv: Path,
    exposure_center_offset_s: float,
) -> int:
    """Write cameraTimes.csv timestamps corrected from end-of-exposure to center."""
    with output_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        for row in matched:
            corrected_utc_s = float(row["rising_edge_utc_s"]) - exposure_center_offset_s
            writer.writerow([f"{corrected_utc_s:.16e}"])
    
    return len(matched)


def write_filtered_frames_by_delta_csv(
    filtered_frames: list[dict[str, str]],
    output_csv: Path,
) -> int:
    """Write filtered frames (post-delta filter) to CSV with header."""
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = list(filtered_frames[0].keys()) if filtered_frames else []
    with output_csv.open("w", newline="") as f:
        if fieldnames:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(filtered_frames)

    return len(filtered_frames)


def main() -> None:
    # Resolve all paths relative to this script's directory
    base_dir = Path(__file__).resolve().parent
    frames_csv = base_dir / "frames.csv"
    tm2_csv = base_dir / "tm2.csv"
    src_image_dir = base_dir / "images_raw" / "camera1"
    output_dir = base_dir / "output"
    
    # Delta time filters (in microseconds)
    min_delta_us = 25000 #exposures of 25000 us or more (25 ms) are expected, so this filters out any frames that are too close together to be valid
    max_delta_us = 36000 #exposures of 36000 us or less (36 ms) are expected, so this filters out any frames that are too far apart to be valid (e.g. dropped frames or long gaps)
    max_match_diff_us = 55000 # allow matches within 55 ms of TM2 arrival time, which should be sufficient given the expected frame rate and timing variability. This is a more lenient threshold to account for any potential timing discrepancies while still aiming to match the correct frames.
    
    # Validate required inputs
    missing = []
    if not frames_csv.exists():
        missing.append(str(frames_csv))
    if not tm2_csv.exists():
        missing.append(str(tm2_csv))
    if not src_image_dir.exists():
        missing.append(str(src_image_dir))

    if missing:
        print("Missing required input paths:")
        for path in missing:
            print(f"  - {path}")
        return

    output_dir.mkdir(parents=True, exist_ok=True)

    print("Step 1: Filtering frames by delta_t_prev_us...")
    filtered_frames = filter_frames_by_delta(
        frames_csv,
        min_delta_us,
        max_delta_us,
    )
    print(f"  Filtered to {len(filtered_frames)} frames")

    filtered_frames_csv = output_dir / "files" / "filtered_frames_by_delta.csv"
    wrote_filtered = write_filtered_frames_by_delta_csv(
        filtered_frames,
        filtered_frames_csv,
    )
    print(f"  Wrote {wrote_filtered} filtered frames to {filtered_frames_csv}")

    print("Step 2: Converting TM2 rising edges to UTC...")
    tm2_edges = convert_tm2_rising_edges(tm2_csv)
    print(f"  Converted {len(tm2_edges)} TM2 edges")

    print("Step 3: Matching frames to TM2 edges...")
    matched = match_frames_to_tm2(filtered_frames, tm2_edges, max_match_diff_us)
    print(f"  Matched {len(matched)} pairs")

    print("Step 4: Copying matched frame images...")
    dst_image_dir = output_dir / "images" / "camera1"
    copied = copy_matched_frames(matched, src_image_dir, dst_image_dir)
    print(f"  Copied {copied} unique frames to {dst_image_dir}")

    print("Step 5: Writing cameraTimes/camera1.csv...")
    camera_times_csv = output_dir / "cameraTimes" / "camera1.csv"
    camera_times_csv.parent.mkdir(parents=True, exist_ok=True)
    wrote = write_camera_times(
        matched,
        camera_times_csv,
        EXPOSURE_CENTER_OFFSET_S,
    )
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
