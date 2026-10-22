"""Lease client for legacy LINE_TRACKING control in navigation-worker mode."""
import math
import uuid


class StandaloneControl:
    def __init__(self):
        self.context = None
        self.granted = False

    def begin(self, task_id, now):
        self.context = {"session_id": uuid.uuid4().hex, "task_id": str(task_id)}
        self.started = now
        self.granted = False
        self.grant_at = None
        self.grant_stamp = float("-inf")

    def request(self, stamp, command=None):
        return dict(self.context, protocol=1, stamp_sec=stamp,
                    command=command or ("HEARTBEAT" if self.granted else "ACQUIRE"))

    def receive(self, body, now, ros_now):
        if (self.context is None or not isinstance(body, dict)
                or type(body.get("protocol")) is not int or body["protocol"] != 1
                or any(body.get(k) != v for k, v in self.context.items())):
            return None
        stamp = body.get("stamp_sec")
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or not -.05 <= ros_now - stamp <= .4 or stamp < self.grant_stamp):
            return None
        if self.granted and not self.ready(now):
            return "LOST"  # A late callback must not revive an expired lease.
        if (body.get("state") == "GRANTED" and type(body.get("lease_sec")) in (int, float)
                and body["lease_sec"] == 1.0):
            first = not self.granted
            self.granted, self.grant_at, self.grant_stamp = True, now, stamp
            return "START" if first else "HEARTBEAT"
        if body.get("state") in ("REJECTED", "STOPPED", "ERROR"):
            return "REJECTED" if not self.granted else "LOST"
        return None

    def ready(self, now):
        return self.context is not None and self.granted and now - self.grant_at <= 1.0

    def failure(self, now):
        if self.context is None:
            return None
        if self.granted and not self.ready(now):
            return "standalone_control_lease_expired"
        if not self.granted and now - self.started > 5.0:
            return "standalone_control_unavailable"
        return None

    def close(self):
        self.context = None
        self.granted = False
