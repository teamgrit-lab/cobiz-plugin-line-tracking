"""Select the front Road policy or the original Sidewalk segmenter per task."""


class FrontRoadInferenceError(RuntimeError):
    """A failed Road request must not take the Sidewalk worker down."""


class SurfaceSegmenter:
    def __init__(self, sidewalk, road_factory):
        self.sidewalk = sidewalk
        self.road_factory = road_factory
        self.road = None
        self.mode = None
        self.device = sidewalk.device

    def reset(self):
        # Called by the inference worker, never concurrently with forward().
        self.sidewalk.reset()
        if self.road is not None:
            self.road.reset()
        self.mode = None

    def segment(self, frame, *, path_mask_class, source_stamp=None, **kwargs):
        if path_mask_class not in (1, 2):
            raise ValueError('front/sidewalk routing requires explicit mode 1 or 2')
        if self.mode is not None and self.mode != path_mask_class:
            self.reset()
        self.mode = path_mask_class
        if path_mask_class == 2:
            return self.sidewalk.segment(frame, path_mask_class=2, **kwargs)
        try:
            # Do not contact the Road service when starting/using Sidewalk.
            if self.road is None:
                road = self.road_factory()
                if road.device.type != self.device.type:
                    raise ValueError('Road and Sidewalk model devices must match')
                self.road = road
            return self.road.segment(frame, path_mask_class=1,
                                     source_stamp=source_stamp, **kwargs)
        except Exception as error:
            # Recheck service identity on the next task; never fall back to R50
            # for Road, and never reuse the failed request's learned command.
            self.road = None
            raise FrontRoadInferenceError(str(error)) from error

    def metadata(self):
        return {'backend': 'task-surface-routing',
                'road': self.road.metadata() if self.road is not None else {'ready': False},
                'sidewalk': self.sidewalk.metadata()}

    def render_overlay(self, *args, **kwargs):
        selected = self.road if self.mode == 1 else self.sidewalk
        return selected.render_overlay(*args, **kwargs)
