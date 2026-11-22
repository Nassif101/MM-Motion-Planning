"""Tests of medium_diag.py (the medium-planner diagnosis, plan docs/superpowers/plans/2026-11-22-medium-diagnosis.md)."""
import struct
import zlib

import medium_diag as md
import plant_stream as ps


def write_log(path, frames, split=None):
    """A run log of encoded frames, in two chunks when `split` (the index of the second chunk's first frame)."""
    parts = [frames] if split is None else [frames[:split], frames[split:]]
    data = b"MMRUNLG1"
    for part in parts:
        raw = b"".join(part)
        packed = zlib.compress(raw)
        data += struct.pack("<II", len(raw), len(packed)) + packed
    path.write_bytes(data)
    return path


def hello(**extra):
    return ps.encode(ps.HELLO, {"schema": 3, **extra}, [])


def state(t, x=0.0):
    return ps.encode(ps.STATE, {"t": t}, [[x, 0.0, 0.0]])


def test_fingerprint_ignores_the_hello_only(tmp_path):
    body = [state(0.02), state(0.04, 0.1), state(0.06, 0.2)]
    a = write_log(tmp_path / "a.mmlog", [hello(provenance={"git_commit": "aaa"})] + body)
    b = write_log(tmp_path / "b.mmlog", [hello(provenance={"git_commit": "bbb"}, config={"seed_detail": False})] + body,
                  split=2)
    c = write_log(tmp_path / "c.mmlog", [hello()] + body[:2] + [state(0.06, 0.3)])
    assert md.log_fingerprint(a) == md.log_fingerprint(b)  # another Hello, another chunking: the same frames
    assert md.log_fingerprint(a) != md.log_fingerprint(c)  # one frame changed
