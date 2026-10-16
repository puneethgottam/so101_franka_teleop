#!/usr/bin/env python3
"""Convert raw episodes from recorder_node.py into a LeRobot dataset (parquet + mp4).

Runs in .venv-lerobot (lerobot needs numpy 2, ROS Jazzy needs numpy 1.26 - keep them apart):

    tools/to_lerobot.sh data/raw --repo-id <hf_user>/franka_pick --root data/lerobot/franka_pick
    # add --push to upload to the Hugging Face hub (huggingface-cli login first)

The dataset is rebuilt from the raw episodes each time (raw = source of truth).
Paused frames at the start/end of each episode are trimmed unless --keep-paused.
LeRobot assumes timestamp = frame_index / fps, so each episode is resampled onto that grid
(nearest recorded frame) to absorb the recorder's timer jitter.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset

VECTORS = ("observation.state", "observation.ee_pose", "action", "action.ee_pose")


def load_episodes(raw):
    eps = sorted(p for p in raw.glob("episode_*") if (p / "meta.json").exists() and (p / "frames.npz").exists())
    return [(p, json.loads((p / "meta.json").read_text())) for p in eps]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw", type=Path, help="raw episode directory (recorder output_dir)")
    ap.add_argument("--repo-id", required=True, help="dataset name, e.g. <hf_user>/franka_pick")
    ap.add_argument("--root", type=Path, required=True, help="output directory of the LeRobot dataset")
    ap.add_argument("--keep-paused", action="store_true", help="keep paused frames at episode start/end")
    ap.add_argument("--min-frames", type=int, default=10, help="skip episodes shorter than this")
    ap.add_argument("--images", action="store_true", help="store PNG images instead of mp4 video")
    ap.add_argument("--overwrite", action="store_true", help="delete --root first if it exists")
    ap.add_argument("--push", action="store_true", help="push to the Hugging Face hub when done")
    args = ap.parse_args()

    episodes = load_episodes(args.raw)
    if not episodes:
        sys.exit(f"no episodes in {args.raw}")
    if args.root.exists():
        if not args.overwrite:
            sys.exit(f"{args.root} exists - pass --overwrite to rebuild it")
        shutil.rmtree(args.root)

    ref = episodes[0][1]
    for p, m in episodes:
        if m["fps"] != ref["fps"] or m["names"] != ref["names"] or set(m["cameras"]) != set(ref["cameras"]):
            sys.exit(f"{p.name}: fps / features / cameras differ from {episodes[0][0].name}")

    features = {k: {"dtype": "float32", "shape": (len(ref["names"][k]),), "names": ref["names"][k]}
                for k in VECTORS}
    for cam in ref["cameras"]:
        shapes = {tuple(m["cameras"][cam]) for _, m in episodes}
        if len(shapes) != 1:
            sys.exit(f"camera {cam} has different image sizes across episodes: {shapes}")
        features[f"observation.images.{cam}"] = {
            "dtype": "image" if args.images else "video",
            "shape": shapes.pop(), "names": ["height", "width", "channels"]}

    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=ref["fps"], features=features, root=args.root,
                               robot_type=ref["robot_type"], use_videos=not args.images)
    n_saved = 0
    for path, meta in episodes:
        f = np.load(path / "frames.npz")
        idx = np.arange(meta["num_frames"])
        if not args.keep_paused:
            eng = np.flatnonzero(f["engaged"])
            idx = idx[eng[0]:eng[-1] + 1] if len(eng) else idx[:0]
        if len(idx):
            t = f["timestamp"][idx]
            grid = t[0] + np.arange(int(round((t[-1] - t[0]) * meta["fps"])) + 1) / meta["fps"]
            j = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
            idx = idx[np.where(grid - t[j - 1] < t[j] - grid, j - 1, j)]
        if len(idx) < args.min_frames:
            print(f"skip {path.name}: {len(idx)} frames")
            continue
        for i in idx:
            frame = {k: f[k][i].astype(np.float32) for k in VECTORS}
            for cam in ref["cameras"]:
                frame[f"observation.images.{cam}"] = np.asarray(Image.open(path / cam / f"{i:06d}.jpg").convert("RGB"))
            frame["task"] = meta["task"]
            ds.add_frame(frame)
        ds.save_episode()
        n_saved += 1
        print(f"{path.name}: {len(idx)} frames  \"{meta['task']}\"")
    ds.finalize()
    print(f"\n{n_saved} episodes -> {args.root}")
    if args.push:
        ds.push_to_hub()


if __name__ == "__main__":
    main()
