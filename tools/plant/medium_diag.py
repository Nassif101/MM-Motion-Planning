"""The medium-planner diagnosis's log reading (plan docs/superpowers/plans/2026-11-22-medium-diagnosis.md; spec
docs/superpowers/specs/2026-11-22-medium-diagnosis-design.md). Standard library only.

log_fingerprint: a run log's frames after the Hello, hashed, so that two builds' logs compare without the Hello's
provenance (the commit) and its config keys.
"""
import hashlib
import struct
import zlib


def _raw_frames(data):
    """The encoded frames of a run log's bytes, in order, across its zlib chunks; a truncated or corrupt last chunk ends
    them after the whole ones (as plant_stream.RunLog)."""
    if data[:8] != b"MMRUNLG1":
        raise ValueError("not a run log")
    at, buffer = 8, b""
    while at + 8 <= len(data):
        _, packed_len = struct.unpack_from("<II", data, at)
        at += 8
        if at + packed_len > len(data):
            return
        try:
            buffer += zlib.decompress(data[at:at + packed_len])
        except zlib.error:
            return
        at += packed_len
        pos = 0
        while pos + 4 <= len(buffer):
            (length,) = struct.unpack_from("<I", buffer, pos)
            if pos + 4 + length > len(buffer):
                break
            yield buffer[pos:pos + 4 + length]
            pos += 4 + length
        buffer = buffer[pos:]


def log_fingerprint(path):
    """sha256 (hex) over the encoded bytes of every frame after the Hello, in order."""
    digest = hashlib.sha256()
    frames = _raw_frames(open(path, "rb").read())
    next(frames, None)  # the Hello
    for frame in frames:
        digest.update(frame)
    return digest.hexdigest()
