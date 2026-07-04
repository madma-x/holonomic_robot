#!/usr/bin/env python3
"""Display map_tool.png in a 3 m x 2 m coordinate frame with live mouse position."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Open a map image and display mouse coordinates in meters "
            "with origin at bottom-left."
        )
    )
    parser.add_argument(
        "image",
        nargs="?",
        default="map_tool.png",
        help="Path to map image (default: map_tool.png)",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=2,
        help="Display every Nth pixel in x/y (default: 2)",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    image_path = Path(args.image)

    if not image_path.exists():
        raise FileNotFoundError(f"Image not found: {image_path}")

    downsample = max(1, args.downsample)

    img = mpimg.imread(image_path)
    img = np.rot90(img, 2)
    if downsample > 1:
        img = img[::downsample, ::downsample]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(
        img,
        extent=[0.0, 3.0, 0.0, 2.0],
        origin="lower",
        interpolation="nearest",
    )
    ax.set_title(f"Map viewer: {image_path.name}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xlim(0.0, 3.0)
    ax.set_ylim(0.0, 2.0)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.5)

    # Overlay text for current pointer position in meters.
    coord_text = ax.text(
        0.02,
        0.98,
        "x = -, y = -",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=11,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "black"},
    )

    ax.format_coord = lambda x, y: f"x = {x:.3f} m, y = {y:.3f} m"

    blit_cache = {"background": None}

    def set_coord_text(new_text: str) -> None:
        if coord_text.get_text() == new_text:
            return
        coord_text.set_text(new_text)
        if fig.canvas.supports_blit and blit_cache["background"] is not None:
            fig.canvas.restore_region(blit_cache["background"])
            ax.draw_artist(coord_text)
            fig.canvas.blit(ax.bbox)
            return
        fig.canvas.draw_idle()

    def on_draw(_event) -> None:
        if not fig.canvas.supports_blit:
            return
        blit_cache["background"] = fig.canvas.copy_from_bbox(ax.bbox)
        ax.draw_artist(coord_text)
        fig.canvas.blit(ax.bbox)

    def on_mouse_move(event) -> None:
        if event.x is None or event.y is None or not ax.bbox.contains(event.x, event.y):
            set_coord_text("x = -, y = -")
            return

        x, y = ax.transData.inverted().transform((event.x, event.y))
        x = max(0.0, min(3.0, x))
        y = max(0.0, min(2.0, y))
        set_coord_text(f"x = {x:.3f} m, y = {y:.3f} m")

    fig.canvas.mpl_connect("draw_event", on_draw)
    fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
