import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from standalone_control import StandaloneControl


def grant(client, stamp=100., **extra):
    return dict(client.context, protocol=1, state="GRANTED", stamp_sec=stamp,
                lease_sec=1., **extra)


def test_grant_loss_stops_within_one_second_and_old_session_cannot_restart():
    client = StandaloneControl()
    client.begin("task", 0.)
    old = grant(client)
    assert client.receive(old, 0., 100.) == "START"
    assert client.ready(.9)
    assert client.failure(1.01) == "standalone_control_lease_expired"
    assert client.receive(grant(client, 101.01), 1.01, 101.01) == "LOST"
    client.close()
    client.begin("next", 2.)
    assert client.receive(old, 2., 102.) is None
    assert not client.ready(2.)
    assert client.failure(7.1) == "standalone_control_unavailable"


def test_stale_future_malformed_and_reordered_grants_do_not_refresh_control():
    client = StandaloneControl()
    client.begin("task", 0.)
    for body in [None, [], grant(client, 99.), grant(client, 101.),
                 dict(grant(client), protocol=True), dict(grant(client), lease_sec=True)]:
        assert client.receive(body, 0., 100.) is None
    assert client.receive(grant(client, 100.1), .1, 100.1) == "START"
    assert client.receive(grant(client, 100.), .2, 100.2) is None
    assert client.grant_at == .1


def test_control_rejection_before_start_and_loss_after_start_are_distinct():
    client = StandaloneControl()
    client.begin("task", 0.)
    rejected = dict(grant(client), state="REJECTED")
    assert client.receive(rejected, 0., 100.) == "REJECTED"
    assert client.receive(grant(client), 0., 100.) == "START"
    assert client.receive(rejected, 0., 100.) == "LOST"
