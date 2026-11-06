"""The MuJoCo plant's stream protocol in Python (MuJoCo plant plan Task 9; docs/experiments/mujoco-plant/stream-protocol.md).

Frames: [u32 length][u16 type][u32 json_len][json][u32 n_blocks]{[u32 n_floats][float32 ...]}, little-endian; `length`
counts the bytes after itself. The same frames go over the demo server's TCP stream and into run logs ("MMRUNLG1",
then zlib chunks of whole frames, the first frame a Hello with the run's header). Standard library only.
"""
import json
import socket
import struct
import zlib

SCHEMA = 1
HELLO, CONTROL, EVENT, STATE, PLANS, SCAN = 1, 2, 3, 10, 11, 12
# a Scan's class block: the plant's HitClass value per point (lidar.hpp), named here in the enum's order
HIT_CLASSES = ["site", "scenario", "mover", "ground", "robot", "panel", "other"]


def encode(kind, header, blocks):
    text = json.dumps(header, separators=(",", ":"), sort_keys=True).encode()  # nlohmann's compact, key-sorted dump
    body = struct.pack("<HI", kind, len(text)) + text + struct.pack("<I", len(blocks))
    for block in blocks:
        body += struct.pack(f"<I{len(block)}f", len(block), *block)
    return struct.pack("<I", len(body)) + body


def decode(buffer):
    """(kind, header, blocks, consumed) of the buffer's first frame, or None while the buffer holds less than a frame."""
    if len(buffer) < 4:
        return None
    (length,) = struct.unpack_from("<I", buffer, 0)
    if len(buffer) < 4 + length:
        return None
    at = 4
    kind, json_len = struct.unpack_from("<HI", buffer, at)
    at += 6
    header = json.loads(bytes(buffer[at:at + json_len]))
    at += json_len
    (n_blocks,) = struct.unpack_from("<I", buffer, at)
    at += 4
    blocks = []
    for _ in range(n_blocks):
        (n,) = struct.unpack_from("<I", buffer, at)
        at += 4
        blocks.append(list(struct.unpack_from(f"<{n}f", buffer, at)))
        at += 4 * n
    if at != 4 + length:
        raise ValueError("frame: trailing bytes")
    return kind, header, blocks, 4 + length


class RunLog:
    """A run log's header and frames."""

    def __init__(self, path):
        self.data = open(path, "rb").read()
        if self.data[:8] != b"MMRUNLG1":
            raise ValueError(f"{path} is not a run log")
        frames = self._frames()
        kind, self.header, _ = next(frames)
        if kind != HELLO:
            raise ValueError(f"{path} has no header")
        self._rest = frames

    def _frames(self):
        at, buffer = 8, b""
        while at < len(self.data):
            raw_len, packed_len = struct.unpack_from("<II", self.data, at)
            at += 8
            buffer += zlib.decompress(self.data[at:at + packed_len])
            at += packed_len
            while (got := decode(buffer)) is not None:
                kind, header, blocks, used = got
                buffer = buffer[used:]
                yield kind, header, blocks

    def frames(self):
        """(kind, header, blocks) after the header."""
        yield from self._rest


class Client:
    """A demo server connection: hello with the schema, commands out, frames in."""

    def __init__(self, host, port, timeout=10.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buffer = b""
        kind, self.hello, _ = self.next()
        if kind != HELLO or self.hello.get("schema") != SCHEMA:
            raise ConnectionError(f"not a schema {SCHEMA} plant demo server: {self.hello}")
        self.command("hello", schema=SCHEMA)

    def command(self, cmd, **fields):
        self.sock.sendall(encode(CONTROL, {"cmd": cmd, **fields}, []))

    def next(self, timeout=None):
        """The next frame (kind, header, blocks); None on timeout."""
        self.sock.settimeout(timeout)
        while (got := decode(self.buffer)) is None:
            try:
                chunk = self.sock.recv(1 << 16)
            except socket.timeout:
                return None
            if not chunk:
                raise ConnectionError("the demo server closed the connection")
            self.buffer += chunk
        kind, header, blocks, used = got
        self.buffer = self.buffer[used:]
        return kind, header, blocks

    def close(self):
        self.sock.close()
