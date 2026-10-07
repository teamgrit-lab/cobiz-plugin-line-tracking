"""Small real-layout ROS sensor packets for obstacle and live-control tests."""

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


def imu(stamp=100.0, up=(0.0, 0.0, 1.0), frame="livox_frame"):
    return NS(
        header=header(stamp, frame),
        linear_acceleration=NS(**dict(zip(("x", "y", "z"), up))),
    )


def camera_info(stamp=100.0, width=640, height=360):
    # Live A2 1280x720 calibration, scaled with the camera relay resolution.
    sx, sy = width / 1280, height / 720
    return NS(
        header=header(stamp, "camera_optical_frame"),
        width=width,
        height=height,
        k=[
            535.088326 * sx,
            0.0,
            643.406605 * sx,
            0.0,
            532.843368 * sy,
            355.973065 * sy,
            0.0,
            0.0,
            1.0,
        ],
        d=[
            -2.13869731,
            0.77162026,
            0.00048845,
            -0.001401,
            0.42753978,
            -1.85354262,
            0.07940282,
            0.84793409,
        ],
        distortion_model="rational_polynomial",
    )
