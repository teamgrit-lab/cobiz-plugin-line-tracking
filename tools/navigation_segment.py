"""ROS-free leased internal segment protocol; no Cobiz or Sport ownership."""
from collections import OrderedDict
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class NavigationSegment:
    def __init__(self, max_age_sec=0.4, max_lease_sec=1.0):
        self.max_age_sec, self.max_lease_sec = max_age_sec, max_lease_sec
        self.active = None
        self.recent = OrderedDict()
        self.sequence = 0

    def remember(self, context, state="STOPPED", reason=""):
        key = (context["session_id"], context["segment_id"])
        self.recent[key] = (state, reason)
        self.recent.move_to_end(key)
        while len(self.recent) > 256:
            self.recent.popitem(last=False)

    @staticmethod
    def identity(body):
        return {k: body[k] for k in ("session_id", "segment_id")}

    def handle(self, body, *, now, ros_now):
        if (not isinstance(body, dict) or type(body.get("protocol")) is not int
                or body["protocol"] != 1
                or any(not isinstance(body.get(k), str) or not 0 < len(body[k]) <= 128
                       for k in ("session_id", "segment_id"))
                or not finite(body.get("stamp_sec"))
                or not -0.05 <= ros_now - body["stamp_sec"] <= self.max_age_sec
                or body.get("command") not in ("START", "HEARTBEAT", "STOP")):
            return None
        identity = self.identity(body)
        key = tuple(identity.values())
        matches = self.active is not None and self.identity(self.active) == identity
        command = body["command"]
        if command == "STOP":
            # STOP before START is a tombstone: a delayed START can never revive
            # an already cancelled leg. A foreign stop never clears this owner.
            self.remember(identity)
            if matches:
                self.active = None
            return dict(identity, effect="STOP" if matches else "STATUS", state="STOPPED")
        if key in self.recent:
            state, reason = self.recent[key]
            return dict(identity, effect="STATUS", state=state, reason=reason)
        lease = body.get("lease_sec")
        if not finite(lease) or not 0 < lease <= self.max_lease_sec:
            return dict(identity, effect="STATUS", state="REJECTED", reason="invalid_lease")
        if command == "START":
            mask = body.get("selected_mask")
            if type(mask) is not int or mask not in (1, 2):
                return dict(identity, effect="STATUS", state="REJECTED", reason="invalid_selected_mask")
            if self.active is not None:
                if not matches or mask != self.active["selected_mask"]:
                    return dict(identity, effect="STATUS", state="REJECTED", reason="segment_busy")
            else:
                self.active = dict(identity, selected_mask=mask, lease_at=now,
                                   lease_sec=lease, control_stamp=body["stamp_sec"])
                self.sequence = 0
                return dict(identity, effect="START", state="ACCEPTED")
        if matches and body["stamp_sec"] >= self.active["control_stamp"]:
            self.active.update(lease_at=now, lease_sec=lease, control_stamp=body["stamp_sec"])
        return None

    def expired(self, now):
        return self.active is not None and now - self.active["lease_at"] > self.active["lease_sec"]

    def close(self, reason="", *, error=False):
        if self.active is None:
            return None
        identity = self.identity(self.active)
        self.remember(identity, "ERROR" if error else "STOPPED", reason)
        self.active = None
        return identity

    def velocity(self, move, *, stamp, context=None):
        context = context or self.active
        if context is None:
            return None
        self.sequence += 1
        return dict(self.identity(context), protocol=1, stamp_sec=stamp,
                    sequence=self.sequence, vx=move.x, vy=move.y, yaw_rate=move.z)
