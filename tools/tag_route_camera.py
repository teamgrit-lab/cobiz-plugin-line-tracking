"""One task-scoped 360 camera owner; detection never blocks the ROS executor."""
import threading
import time

from panorama_tag_alignment import PanoramaDetector


class TagRouteCamera:
    def __init__(self, device, front_x=720., hz=5., camera_offset_xy=(0., 0.),
                 family='36h11', source_factory=None, detector_factory=None,
                 camera_z_m=0., camera_roll_pitch_rad=(0., 0.)):
        self.device, self.front_x, self.period = device, front_x, 1. / hz
        self.camera_offset_xy, self.family = camera_offset_xy, family
        self.camera_z_m, self.camera_roll_pitch_rad = camera_z_m, camera_roll_pitch_rad
        self.source_factory = source_factory or self._source
        self.detector_factory = detector_factory or PanoramaDetector
        self.lock = threading.Lock()
        self.wake, self.closed, self.idle = threading.Event(), threading.Event(), threading.Event()
        self.idle.set()
        self.session, self.packet = None, None
        self.thread = threading.Thread(target=self._run, name='tag-route-camera', daemon=True)
        self.thread.start()

    def _source(self):
        from surround_capture import Camera
        return Camera(self.device)

    def start(self, session_id, spec):
        with self.lock:
            self.session, self.packet = (session_id, spec), None
            self.idle.clear()
        self.wake.set()

    def stop(self):
        with self.lock:
            self.session, self.packet = None, None
        self.wake.set()

    def latest(self):
        with self.lock:
            return dict(self.packet) if self.packet is not None else None

    def close(self):
        self.closed.set()
        self.wake.set()
        self.thread.join(timeout=5.)

    def _run(self):
        source, detector, active = None, None, None
        try:
            while not self.closed.is_set():
                with self.lock:
                    requested = self.session
                if requested != active:
                    if source is not None:
                        try:
                            source.close()
                        except Exception:
                            pass
                    source, detector, active = None, None, requested
                if requested is None:
                    with self.lock:
                        if self.session is not None:
                            continue
                        self.idle.set()
                    self.wake.wait(.1)
                    self.wake.clear()
                    continue
                started = time.monotonic()
                try:
                    session_id, spec = requested
                    if detector is None:
                        detector = self.detector_factory(2880, 1440, self.front_x,
                            family=self.family, tag_ids={spec.start_tag_id, spec.end_tag_id},
                            tag_size_m=spec.tag_size_m, camera_offset_xy=self.camera_offset_xy,
                            camera_z_m=self.camera_z_m, camera_roll_pitch_rad=self.camera_roll_pitch_rad)
                    if source is None:
                        source = self.source_factory()
                    capture = source.read()
                    if capture is not None:
                        image, stamp_ns = capture
                        detections = detector.detect(image)
                        packet = dict(session_id=session_id, stamp_ns=stamp_ns, detections=detections)
                    else:
                        packet = None
                except Exception as error:
                    # A previous Road owner can keep the USB camera briefly
                    # while its idle lease expires. Retry within admission's
                    # bounded sensor timeout, retaining the actual error.
                    packet = dict(session_id=requested[0], error=str(error))
                    if source is not None:
                        try:
                            source.close()
                        except Exception:
                            pass
                    source = None
                if packet is not None:
                    with self.lock:
                        if self.session == requested:
                            self.packet = packet
                self.wake.wait(max(.01, self.period - (time.monotonic() - started)))
                self.wake.clear()
        finally:
            try:
                if source is not None:
                    source.close()
            finally:
                self.idle.set()
