import json
import math

import pytest

from px4_orbit_inspection.area_patrol import lawnmower_waypoints, route_length
from px4_orbit_inspection.ground_link import GroundLink, classify, make_event


def test_lawnmower_covers_rectangle_and_alternates():
    pts = lawnmower_waypoints(0.0, 20.0, 0.0, 10.0, 10.0, 6.0)
    assert pts == [(0.0, 0.0, 6.0), (20.0, 0.0, 6.0), (20.0, 10.0, 6.0), (0.0, 10.0, 6.0)]


def test_last_lane_lands_on_east_max_when_spacing_does_not_divide_evenly():
    pts = lawnmower_waypoints(0.0, 1.0, 0.0, 10.0, 4.0, 2.0)
    easts = sorted({p[1] for p in pts})
    assert easts == [0.0, 4.0, 8.0, 10.0]


def test_bad_inputs_raise():
    with pytest.raises(ValueError):
        lawnmower_waypoints(0.0, 1.0, 0.0, 1.0, 0.0, 2.0)
    with pytest.raises(ValueError):
        lawnmower_waypoints(5.0, 1.0, 0.0, 1.0, 1.0, 2.0)


def test_route_length():
    assert route_length([(0, 0, 0), (3, 4, 0)]) == 5.0
    assert math.isclose(route_length([(0, 0, 0), (0, 10, 0), (10, 10, 0)]), 20.0)


def test_classify_labels():
    assert classify("person") == "emergency"
    assert classify("crack") == "damage"
    assert classify("car") == "object"


def test_ground_link_sends_json_to_address():
    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, data, addr):
            self.sent.append((data, addr))

    fake = FakeSocket()
    GroundLink("10.0.0.5", 9999, sock=fake).send(make_event("progress", waypoint=2))
    data, addr = fake.sent[0]
    assert addr == ("10.0.0.5", 9999)
    payload = json.loads(data)
    assert payload["type"] == "progress" and payload["waypoint"] == 2 and "time_s" in payload
