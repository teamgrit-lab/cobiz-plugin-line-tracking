"""Timestamped latest-frame capture shared by stitched 360 camera consumers."""
import cv2
import numpy as np


class Camera:
    """Pull the latest timestamped MJPEG frame; inference never queues old captures."""
    def __init__(self, device):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        self.gst = Gst
        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            "v4l2src name=camera ! image/jpeg,width=2880,height=1440 ! "
            "appsink name=latest max-buffers=1 drop=true sync=false")
        self.pipeline.get_by_name("camera").set_property("device", device)
        self.sink = self.pipeline.get_by_name("latest")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.close()
            raise RuntimeError(f"Cannot open 2880x1440 MJPEG capture on {device}")

    def read(self, clock):
        gst = self.gst
        sample = self.sink.emit("try-pull-sample", 200 * gst.MSECOND)
        if sample is None:
            error = self.pipeline.get_bus().pop_filtered(gst.MessageType.ERROR | gst.MessageType.EOS)
            if error is not None:
                raise RuntimeError(error.parse_error() if error.type == gst.MessageType.ERROR else "camera EOS")
            return None
        buffer = sample.get_buffer()
        pts = sample.get_segment().to_running_time(gst.Format.TIME, buffer.pts)
        if pts == gst.CLOCK_TIME_NONE:
            return None
        running = self.pipeline.get_clock().get_time() - self.pipeline.get_base_time()
        age = running - pts
        if not 0 <= age <= gst.SECOND:
            return None
        from rclpy.duration import Duration
        stamp = (clock.now() - Duration(nanoseconds=int(age))).to_msg()
        encoded = np.frombuffer(buffer.extract_dup(0, buffer.get_size()), np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Cannot decode 360 camera JPEG")
        return image, stamp

    def close(self):
        self.pipeline.set_state(self.gst.State.NULL)
