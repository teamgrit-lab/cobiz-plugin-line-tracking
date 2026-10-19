"""Isolate CUDA/HTTP/geometry Python work from live ROS sensor/control callbacks.

This child imports no ROS interfaces and cannot publish robot commands. One
synchronous request is in flight; the parent retains task/epoch/source gates.
"""
import multiprocessing as mp
import time


def perception_worker(pipe, config, planner_values):
    try:
        import cv2
        import numpy as np
        from best_so_far_runtime import BestSoFarSegmenter
        from connectome_client import ConnectomeClient
        cv2.setNumThreads(2)
        model = BestSoFarSegmenter(config)
        planner = ConnectomeClient(**planner_values)
        # Capture the fixed-shape model before announcing readiness. Reset
        # temporal state so the synthetic warmup never supports a live path.
        if config.cuda_graph:
            model.segment(np.zeros((model.profile.input_height, model.profile.input_width, 3), np.uint8),
                          color_order='rgb', detect_road_hazards=True)
            model.reset()
        pipe.send({'ready': True, 'device': str(model.device)})
        while True:
            request = pipe.recv()
            if request is None: break
            if request['reset']: model.reset()
            started = time.monotonic()
            result = model.segment(request['rgb'], color_order='rgb', detect_road_hazards=True)
            segmented = time.monotonic()
            estimates = planner.plan(cv2.cvtColor(request['rgb'], cv2.COLOR_RGB2BGR),
                result.selected_mask, result.road_hazard_mask,
                request['stamp'], request['generation'], request['sequence'])
            memory = {}
            if model.device.type == 'cuda':
                import torch
                memory = {key+'_mib': fn(model.device)/1024**2 for key, fn in (
                    ('allocated', torch.cuda.memory_allocated), ('reserved', torch.cuda.memory_reserved),
                    ('max_allocated', torch.cuda.max_memory_allocated), ('max_reserved', torch.cuda.max_memory_reserved))}
            pipe.send({'sequence': request['sequence'], 'generation': request['generation'],
                'result': result, 'estimates': estimates, 'planner': planner.metrics,
                'segmentation_finished_at': segmented, 'memory': memory})
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as error:
        try: pipe.send({'error': type(error).__name__+': '+str(error)[:600]})
        except (EOFError, BrokenPipeError, OSError): pass
    finally:
        pipe.close()


class PerceptionProcess:
    def __init__(self, config, planner_values, *, target=perception_worker, startup_timeout=120., timeout=3.):
        import torch
        context = mp.get_context('spawn')
        self.pipe, child = context.Pipe()
        self.process = context.Process(target=target, args=(child, config, planner_values),
                                       name='line-tracking-perception', daemon=True)
        self.timeout, self.needs_reset, self.cuda_memory = timeout, True, {}
        self.process.start(); child.close()
        try:
            ready = self._receive(startup_timeout)
            if ready.get('ready') is not True:
                raise RuntimeError('perception child did not become ready')
            self.device = torch.device(ready['device'])
        except BaseException:
            self.close()
            raise

    def _receive(self, timeout):
        if not self.pipe.poll(timeout):
            raise TimeoutError('perception child deadline exceeded')
        try: reply = self.pipe.recv()
        except EOFError as error: raise RuntimeError('perception child exited') from error
        if reply.get('error'): raise RuntimeError('perception child: '+reply['error'])
        return reply

    def reset(self):
        self.needs_reset = True

    def infer(self, rgb, stamp, generation, sequence):
        self.pipe.send({'rgb': rgb, 'stamp': stamp, 'generation': generation,
                        'sequence': sequence, 'reset': self.needs_reset})
        self.needs_reset = False
        reply = self._receive(self.timeout)
        if reply.get('sequence') != sequence or reply.get('generation') != generation:
            raise RuntimeError('perception reply does not match task/frame')
        self.cuda_memory = reply['memory']
        return reply

    def close(self):
        if self.process.is_alive():
            try: self.pipe.send(None)
            except (BrokenPipeError, EOFError, OSError): pass
            self.process.join(2.)
        if self.process.is_alive():
            self.process.terminate(); self.process.join(2.)
        self.pipe.close()
