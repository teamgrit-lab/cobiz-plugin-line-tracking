"""Small real-layout ROS sensor packets for recorded point-cloud decoding tests."""

from types import SimpleNamespace as NS
import numpy as np


def header(stamp=100.0, frame="livox_frame"):
    ns = round(stamp * 1e9)
    sec, nanosec = divmod(ns, 1_000_000_000)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame)


def cloud(points, stamp=100.0, frame="livox_frame", endian="<", padding=0):
    points = np.asarray(points, float).reshape(-1, 3)
    dtype = np.dtype(
        dict(
            names=["x", "y", "z"],
            formats=[endian + "f4"] * 3,
            offsets=[0, 4, 8],
            itemsize=26,
        )
    )
    raw = np.zeros(len(points), dtype=dtype)
    for i, name in enumerate(("x", "y", "z")):
        raw[name] = points[:, i]
    return NS(
        header=header(stamp, frame),
        width=len(points),
        height=1,
        point_step=26,
        row_step=len(points) * 26 + padding,
        is_bigendian=endian == ">",
        data=raw.tobytes() + bytes(padding),
        fields=[
            NS(name=n, offset=i * 4, datatype=7, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ],
    )
