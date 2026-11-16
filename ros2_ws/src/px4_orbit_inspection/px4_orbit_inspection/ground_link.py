"""
Ground-station link: sends mission and detection events as JSON over UDP.

UDP is one-way and fire-and-forget, which suits live telemetry: a late packet
is worth less than a fresh one. The socket is injectable so tests need no network.
"""

import json
import socket
import time

# YOLO labels that mean a person needs help (an emergency).
EMERGENCY_LABELS = {"person"}

# Labels that a custom-trained model would emit for infrastructure damage.
# COCO (the default weights) has none of these, so they only fire with custom weights.
DAMAGE_LABELS = {"crack", "spalling", "corrosion", "leak", "fire", "smoke"}


def classify(label):
    """Map a detector label to 'emergency', 'damage' or 'object'."""
    if label in EMERGENCY_LABELS:
        return "emergency"
    if label in DAMAGE_LABELS:
        return "damage"
    return "object"


def make_event(kind, **fields):
    """Build a JSON-serialisable event with a timestamp."""
    return {"type": kind, "time_s": round(time.time(), 3), **fields}


class GroundLink:
    def __init__(self, host="127.0.0.1", port=14560, sock=None):
        self.addr = (host, int(port))
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, event):
        self.sock.sendto(json.dumps(event).encode("utf-8"), self.addr)


def _demo():
    """Self-check with a fake socket: no network needed."""

    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, data, addr):
            self.sent.append((data, addr))

    fake = FakeSocket()
    link = GroundLink("10.0.0.5", 9999, sock=fake)
    link.send(make_event("detection", category=classify("person"), label="person"))
    data, addr = fake.sent[0]
    assert addr == ("10.0.0.5", 9999)
    assert json.loads(data)["category"] == "emergency"
    assert classify("car") == "object" and classify("crack") == "damage"


if __name__ == "__main__":
    _demo()
    print("ground_link self-check passed")
