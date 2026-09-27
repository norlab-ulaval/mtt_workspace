#!/usr/bin/env python3
"""Export a lightweight inspection MP4 directly from a ROS 2 MCAP bag.

The script never starts a ROS graph and never calls ``ros2 bag play``. It
reads one recorded ``sensor_msgs/msg/CompressedImage`` topic through
``rosbag2_py`` and pipes decoded frames directly to ffmpeg.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
except ImportError:
    rosbag2_py = None
    deserialize_message = None
    CompressedImage = None

DEFAULT_TOPIC = "/zed/zed_node/rgb/color/rect/image/compressed"
TRASH_DIR_NAME = ".trash_backup_2026-07-29"

PRIORITY_TOPICS = [
    "/zed/zed_node/rgb/color/rect/image/compressed",
    "/zed/zed_node/rgb/image_rect_color/compressed",
    "/zed2i/zed_node/rgb/color/rect/image/compressed",
    "/oak/rgb/image_rect/compressed",
    "/oak/rgb/image_raw/compressed",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "session",
        type=Path,
        nargs="?",
        help="Session directory or bag directory",
    )
    parser.add_argument(
        "output",
        type=Path,
        nargs="?",
        help="Destination MP4 (defaults to <session>/dataset/preview.mp4)",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Explicit ROS 2 CompressedImage topic to export",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force re-generation even if preview.mp4 exists and is valid",
    )
    parser.add_argument(
        "--all-sessions",
        type=Path,
        help="Process all sessions inside the specified bags root directory",
    )
    parser.add_argument("--crf", type=int, default=24)
    parser.add_argument("--preset", default="veryfast")
    parser.add_argument(
        "--max-width",
        type=int,
        default=1280,
        help="Maximum output video width (default: 1280)",
    )
    parser.add_argument(
        "--max-height",
        type=int,
        default=720,
        help="Maximum output video height (default: 720)",
    )
    return parser.parse_args()


def resolve_bag_dir(path: Path) -> tuple[Path, Path]:
    """Return (session_dir, bag_dir)."""
    path = path.resolve()
    if (path / "metadata.yaml").is_file():
        session_dir = path.parent if path.name == "bag" else path
        return session_dir, path
    elif (path / "bag" / "metadata.yaml").is_file():
        return path, path / "bag"
    else:
        raise FileNotFoundError(f"No metadata.yaml found in {path} or {path / 'bag'}")


def verify_video(file_path: Path) -> dict | None:
    """Verify video file with ffprobe."""
    if not file_path.is_file() or file_path.stat().st_size == 0:
        return None
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=codec_name,pix_fmt,width,height,nb_frames,duration,avg_frame_rate",
        "-show_entries",
        "format=duration,size",
        "-of",
        "json",
        str(file_path),
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        data = json.loads(res.stdout)
        streams = data.get("streams", [])
        if not streams:
            return None
        stream = streams[0]
        fmt = data.get("format", {})

        codec = stream.get("codec_name")
        pix_fmt = stream.get("pix_fmt")
        width = int(stream.get("width", 0))
        height = int(stream.get("height", 0))

        duration_val = fmt.get("duration") or stream.get("duration") or "0"
        duration = float(duration_val)
        nb_frames_val = stream.get("nb_frames")
        nb_frames = int(nb_frames_val) if nb_frames_val and nb_frames_val != "N/A" else 0

        size_bytes = int(fmt.get("size", file_path.stat().st_size))
        size_mb = size_bytes / (1024 * 1024)

        if (
            codec == "h264"
            and pix_fmt == "yuv420p"
            and width > 0
            and height > 0
            and width <= 1280
            and height <= 720
            and duration > 0
        ):
            return {
                "codec": codec,
                "pix_fmt": pix_fmt,
                "width": width,
                "height": height,
                "resolution": f"{width}x{height}",
                "duration": duration,
                "nb_frames": nb_frames,
                "size_mb": size_mb,
                "size_str": f"{size_mb:.2f} MB",
            }
    except Exception:
        return None
    return None


def select_topic(
    bag_dir: Path, requested_topic: str | None
) -> tuple[dict, str | None, int, float, int]:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"Missing metadata.yaml at {metadata_path}")
    with open(metadata_path, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f)

    info = root.get("rosbag2_bagfile_information", {})
    topics = info.get("topics_with_message_count", [])
    duration_ns = info.get("duration", {}).get("nanoseconds", 0)
    duration_s = float(duration_ns) / 1e9 if duration_ns > 0 else 0.0
    start_ns = int(info.get("starting_time", {}).get("nanoseconds_since_epoch", 0))

    topic_map: dict[str, tuple[str, int]] = {}
    for entry in topics:
        tm = entry.get("topic_metadata", {})
        tname = tm.get("name")
        ttype = tm.get("type")
        count = int(entry.get("message_count", 0))
        if tname and ttype:
            topic_map[tname] = (ttype, count)

    selected_topic = None
    selected_count = 0

    if requested_topic and requested_topic in topic_map:
        ttype, count = topic_map[requested_topic]
        if count > 0 and ttype == "sensor_msgs/msg/CompressedImage":
            selected_topic = requested_topic
            selected_count = count

    if not selected_topic:
        for candidate in PRIORITY_TOPICS:
            if candidate in topic_map:
                ttype, count = topic_map[candidate]
                if count > 0 and ttype == "sensor_msgs/msg/CompressedImage":
                    selected_topic = candidate
                    selected_count = count
                    break

    if not selected_topic:
        for tname, (ttype, count) in topic_map.items():
            if ttype == "sensor_msgs/msg/CompressedImage" and count > 0:
                tname_lower = tname.lower()
                if ("rgb" in tname_lower or "color" in tname_lower) and "depth" not in tname_lower:
                    selected_topic = tname
                    selected_count = count
                    break

    if not selected_topic or selected_count <= 0 or duration_s <= 0:
        return info, None, 0, duration_s, start_ns

    return info, selected_topic, selected_count, duration_s, start_ns


def scaled_size(width: int, height: int, max_w: int = 1280, max_h: int = 720) -> tuple[int, int]:
    scale = min(1.0, max_w / width, max_h / height)
    out_w = max(2, int(round(width * scale / 2.0) * 2))
    out_h = max(2, int(round(height * scale / 2.0) * 2))
    return out_w, out_h


def start_ffmpeg(
    output_tmp: Path,
    width: int,
    height: int,
    fps: float,
    crf: int,
    preset: str,
) -> subprocess.Popen:
    command = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        f"{fps:.9f}",
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        preset,
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_tmp),
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)


def open_reader(
    bag_dir: Path, storage_id: str, topic: str, info: dict, session_name: str
) -> tuple[rosbag2_py.SequentialReader, Path | None]:
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
            rosbag2_py.ConverterOptions("", ""),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        return reader, None
    except Exception as primary_err:
        mcap_files = list(bag_dir.glob("*.mcap"))
        expected_files = info.get("relative_file_paths", []) or [
            f.get("path") for f in info.get("files", []) if f.get("path")
        ]
        if mcap_files and expected_files:
            import shutil
            import tempfile

            tmp_dir = Path(tempfile.mkdtemp(prefix=f"mcap_symlink_{session_name}_"))
            try:
                os.symlink(bag_dir / "metadata.yaml", tmp_dir / "metadata.yaml")
                if len(mcap_files) == 1 and len(expected_files) == 1:
                    os.symlink(mcap_files[0], tmp_dir / expected_files[0])
                else:
                    for exp in expected_files:
                        exp_name = Path(exp).name
                        if (bag_dir / exp_name).exists():
                            os.symlink(bag_dir / exp_name, tmp_dir / exp_name)
                        elif mcap_files:
                            os.symlink(mcap_files[0], tmp_dir / exp_name)

                reader_tmp = rosbag2_py.SequentialReader()
                reader_tmp.open(
                    rosbag2_py.StorageOptions(uri=str(tmp_dir), storage_id=storage_id),
                    rosbag2_py.ConverterOptions("", ""),
                )
                reader_tmp.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
                print(
                    f"[{session_name}] Opened reader using temporary symlink wrapper in {tmp_dir}",
                    file=sys.stderr,
                )
                return reader_tmp, tmp_dir
            except Exception:
                shutil.rmtree(tmp_dir, ignore_errors=True)
                raise primary_err
        raise primary_err


def export_single_session(
    session_path: Path,
    output_path: Path | None = None,
    requested_topic: str | None = None,
    force: bool = False,
    crf: int = 24,
    preset: str = "veryfast",
    max_w: int = 1280,
    max_h: int = 720,
) -> dict:
    if rosbag2_py is None:
        raise RuntimeError("rosbag2_py is missing. Ensure ROS 2 environment is sourced.")

    session_dir, bag_dir = resolve_bag_dir(session_path)
    session_name = session_dir.name

    if output_path is None:
        output_path = session_dir / "dataset" / "preview.mp4"
    else:
        output_path = output_path.resolve()

    # Check idempotency
    if not force:
        v_info = verify_video(output_path)
        if v_info is not None:
            actual_fps = v_info["nb_frames"] / max(0.1, v_info["duration"]) if v_info["nb_frames"] > 0 else 0
            return {
                "session": session_name,
                "topic": requested_topic or DEFAULT_TOPIC,
                "frames": v_info["nb_frames"],
                "fps": f"{actual_fps:.2f}",
                "duration": f"{v_info['duration']:.2f} s",
                "resolution": v_info["resolution"],
                "size": v_info["size_str"],
                "statut": "ok (existant)",
            }

    info, topic, expected_count, duration_s, bag_start_ns = select_topic(
        bag_dir, requested_topic
    )

    if not topic or expected_count <= 0:
        return {
            "session": session_name,
            "topic": requested_topic or "-",
            "frames": 0,
            "fps": "-",
            "duration": f"{duration_s:.2f} s" if duration_s > 0 else "-",
            "resolution": "-",
            "size": "-",
            "statut": "no RGB stream",
        }

    storage_id = str(info.get("storage_identifier", "mcap"))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_tmp = output_path.with_name(f".{output_path.name}.tmp.mp4")
    if output_tmp.exists():
        output_tmp.unlink()

    fps = max(0.1, expected_count / duration_s)
    tmp_symlink_dir = None
    try:
        reader, tmp_symlink_dir = open_reader(bag_dir, storage_id, topic, info, session_name)
    except Exception as err:
        print(f"[{session_name}] Failed to open bag reader: {err}", file=sys.stderr, flush=True)
        return {
            "session": session_name,
            "topic": topic,
            "frames": 0,
            "fps": "-",
            "duration": f"{duration_s:.2f} s" if duration_s > 0 else "-",
            "resolution": "-",
            "size": "-",
            "statut": "error (missing/corrupted mcap)",
        }

    encoder: subprocess.Popen | None = None
    out_w, out_h = 0, 0
    written = 0

    try:
        try:
            while reader.has_next():
                _, serialized, _ = reader.read_next()
                message = deserialize_message(serialized, CompressedImage)
                frame = cv2.imdecode(
                    np.frombuffer(message.data, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if frame is None:
                    continue

                h, w = frame.shape[:2]
                if encoder is None:
                    out_w, out_h = scaled_size(w, h, max_w, max_h)
                    encoder = start_ffmpeg(
                        output_tmp, out_w, out_h, fps, crf, preset
                    )

                if (w, h) != (out_w, out_h):
                    frame = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)

                assert encoder is not None and encoder.stdin is not None
                encoder.stdin.write(frame.tobytes())
                written += 1
                if written % 1000 == 0:
                    print(f"[{session_name}] Processed {written}/{expected_count} frames...", file=sys.stderr, flush=True)
        except Exception as e:
            if encoder is not None:
                if encoder.stdin is not None:
                    encoder.stdin.close()
                encoder.kill()
                encoder.wait()
            if output_tmp.exists():
                output_tmp.unlink()
            raise e

        if encoder is None or encoder.stdin is None:
            return {
                "session": session_name,
                "topic": topic,
                "frames": 0,
                "fps": "-",
                "duration": f"{duration_s:.2f} s",
                "resolution": "-",
                "size": "-",
                "statut": "no RGB stream",
            }

        encoder.stdin.close()
        return_code = encoder.wait()
        if return_code != 0:
            if output_tmp.exists():
                output_tmp.unlink()
            raise RuntimeError(f"ffmpeg failed with exit code {return_code}")

        output_tmp.replace(output_path)

        v_info = verify_video(output_path)
        if v_info is None:
            raise RuntimeError(f"ffprobe verification failed for {output_path}")

        actual_fps = written / duration_s if duration_s > 0 else fps
        return {
            "session": session_name,
            "topic": topic,
            "frames": written,
            "fps": f"{actual_fps:.2f}",
            "duration": f"{v_info['duration']:.2f} s",
            "resolution": v_info["resolution"],
            "size": v_info["size_str"],
            "statut": "ok",
        }
    finally:
        if tmp_symlink_dir and tmp_symlink_dir.exists():
            import shutil
            shutil.rmtree(tmp_symlink_dir, ignore_errors=True)


def main() -> int:
    args = parse_args()

    if args.all_sessions:
        bags_root = args.all_sessions.resolve()
        results = []
        for entry in sorted(os.listdir(bags_root)):
            if entry == TRASH_DIR_NAME or entry.startswith("."):
                continue
            session_path = bags_root / entry
            if not session_path.is_dir():
                continue
            try:
                resolve_bag_dir(session_path)
            except FileNotFoundError:
                continue

            print(f"Processing session: {entry} ...", file=sys.stderr, flush=True)
            res = export_single_session(
                session_path,
                requested_topic=args.topic,
                force=args.force,
                crf=args.crf,
                preset=args.preset,
                max_w=args.max_width,
                max_h=args.max_height,
            )
            results.append(res)

        print("\n### Summary Table\n")
        print("| session | topic | frames | fps vidéo | durée | résolution | taille | statut |")
        print("| --- | --- | --- | --- | --- | --- | --- | --- |")
        for r in results:
            print(
                f"| {r['session']} | `{r['topic']}` | {r['frames']} | {r['fps']} | "
                f"{r['duration']} | {r['resolution']} | {r['size']} | {r['statut']} |"
            )
        return 0

    if not args.session:
        print("Error: session path required unless --all-sessions is specified.", file=sys.stderr)
        return 1

    res = export_single_session(
        args.session,
        args.output,
        requested_topic=args.topic,
        force=args.force,
        crf=args.crf,
        preset=args.preset,
        max_w=args.max_width,
        max_h=args.max_height,
    )

    print("\nResult:")
    print(json.dumps(res, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
