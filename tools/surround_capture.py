"""Timestamped latest-frame capture shared by stitched 360 camera consumers."""
import cv2
import numpy as np
import time


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

    def read(self, clock=None):
        capture = self._sample(clock)
        if capture is None:
            return None
        sample, stamp = capture
        buffer = sample.get_buffer()
        encoded = np.frombuffer(buffer.extract_dup(0, buffer.get_size()), np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Cannot decode 360 camera JPEG")
        return image, stamp

    def _sample(self, clock=None):
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
        # Direct-camera inference does not need ROS. Preserve the original
        # capture time, rather than stamping a completed inference as new input.
        if clock is None:
            stamp = time.time_ns() - int(age)
        else:
            from rclpy.duration import Duration
            stamp = (clock.now() - Duration(nanoseconds=int(age))).to_msg()
        return sample, stamp

    def close(self):
        self.pipeline.set_state(self.gst.State.NULL)


class HardwareCamera(Camera):
    """Jetson MJPEG hardware decode -> VIC BGRx -> CUDA uint8 panorama.

    The GStreamer/Python boundary is a system-memory staging buffer, NOT
    zero-copy NVMM/EGL interop. No JPEG decoding or projection runs on CPU.
    CUDA upload completes before unmapping/releasing the GStreamer buffer.
    """
    PIPELINE = (
        "v4l2src name=camera ! image/jpeg,width=2880,height=1440 ! "
        "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
        "jpegparse ! nvv4l2decoder mjpeg=true ! nvvidconv ! "
        "video/x-raw,format=BGRx,width=2880,height=1440 ! "
        "appsink name=latest max-buffers=1 drop=true sync=false"
    )

    @staticmethod
    def check_runtime():
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError('360 hardware capture requires CUDA; no software fallback')
        import gi
        gi.require_version('Gst', '1.0')
        gi.require_version('GstVideo', '1.0')
        from gi.repository import Gst, GstVideo
        Gst.init(None)
        missing = [name for name in ('v4l2src', 'jpegparse', 'nvv4l2decoder', 'nvvidconv', 'appsink')
                   if Gst.ElementFactory.find(name) is None]
        if missing:
            raise RuntimeError('Missing Jetson capture plugins: ' + ', '.join(missing))
        return torch, Gst, GstVideo

    def __init__(self, device):
        self.torch, self.gst, self.video = self.check_runtime()
        self.pipeline = self.gst.parse_launch(self.PIPELINE)
        self.pipeline.get_by_name('camera').set_property('device', device)
        self.sink = self.pipeline.get_by_name('latest')
        try:
            if self.pipeline.set_state(self.gst.State.PLAYING) == self.gst.StateChangeReturn.FAILURE:
                raise RuntimeError(f'Cannot open Jetson hardware MJPEG capture on {device}')
        except BaseException:
            self.close()
            raise

    def read(self, clock=None):
        capture = self._sample(clock)
        if capture is None:
            return None
        sample, stamp = capture
        info = self.video.VideoInfo.new_from_caps(sample.get_caps())
        if info is None or (info.width, info.height, info.finfo.name) != (2880, 1440, 'BGRx'):
            raise ValueError('Hardware decoder must produce 2880x1440 BGRx')
        buffer = sample.get_buffer()
        # A producer may advertise row padding through VideoMeta rather than caps.
        meta = self.video.buffer_get_video_meta(buffer)
        stride, offset = (meta.stride[0], meta.offset[0]) if meta else (info.stride[0], info.offset[0])
        ok, mapped = buffer.map(self.gst.MapFlags.READ)
        if not ok:
            raise RuntimeError('Cannot map decoded 360 frame')
        try:
            frame = upload_bgrx(mapped.data, info.width, info.height, stride, offset)
        finally:
            buffer.unmap(mapped)
        return frame, stamp


def upload_bgrx(data, width, height, stride, offset=0, *, device='cuda'):
    """Respect row pitch and finish the upload while the GstBuffer is mapped."""
    import torch
    if stride < width * 4 or offset < 0 or len(data) < offset + (height-1)*stride + width*4:
        raise ValueError('Invalid decoded BGRx buffer layout')
    # GI exposes a read-only memoryview on some versions. The owning bytearray
    # is an explicit bounded staging copy; PyTorch never writes into Gst memory.
    storage = torch.frombuffer(bytearray(data), dtype=torch.uint8)
    image = storage.as_strided((height, width, 4), (stride, 4, 1), offset)
    return image.to(device=device, non_blocking=False).contiguous()
