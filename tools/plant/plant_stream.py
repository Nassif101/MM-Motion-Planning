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


KINDS = {HELLO, CONTROL, EVENT, STATE, PLANS, SCAN}


def decode(buffer, at0=0):
    """(kind, header, blocks, consumed) of the frame at at0, or None while the buffer holds less than a frame; ValueError
    for a malformed one (lengths that disagree, an unknown type, bad JSON)."""
    if len(buffer) < at0 + 4:
        return None
    (length,) = struct.unpack_from("<I", buffer, at0)
    end = at0 + 4 + length
    if len(buffer) < end:
        return None
    def need(at, n):
        if at + n > end:
            raise ValueError("frame: a length runs past the frame")
    at = at0 + 4
    need(at, 6)
    kind, json_len = struct.unpack_from("<HI", buffer, at)
    if kind not in KINDS:
        raise ValueError(f"frame: unknown type {kind}")
    at += 6
    need(at, json_len)
    try:
        header = json.loads(bytes(buffer[at:at + json_len]))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"frame: bad JSON ({error})") from None
    at += json_len
    need(at, 4)
    (n_blocks,) = struct.unpack_from("<I", buffer, at)
    at += 4
    blocks = []
    for _ in range(n_blocks):
        need(at, 4)
        (n,) = struct.unpack_from("<I", buffer, at)
        at += 4
        need(at, 4 * n)
        blocks.append(list(struct.unpack_from(f"<{n}f", buffer, at)))
        at += 4 * n
    if at != end:
        raise ValueError("frame: trailing bytes")
    return kind, header, blocks, end - at0


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
        """Frames chunk by chunk; a truncated last chunk (a writer that died) ends the log after the whole ones."""
        at, buffer = 8, b""
        while at + 8 <= len(self.data):
            raw_len, packed_len = struct.unpack_from("<II", self.data, at)
            at += 8
            if at + packed_len > len(self.data):
                return
            try:
                buffer += zlib.decompress(self.data[at:at + packed_len])
            except zlib.error:
                return
            at += packed_len
            pos = 0  # an offset, not a slice per frame (quadratic over a 1 MiB chunk)
            while (got := decode(buffer, pos)) is not None:
                kind, header, blocks, used = got
                pos += used
                yield kind, header, blocks
            buffer = buffer[pos:]

    def frames(self):
        """(kind, header, blocks) after the header."""
        yield from self._rest


class Client:
    """A demo server connection: hello with the schema, commands out, frames in."""

    def __init__(self, host, port, timeout=10.0):
        self.timeout = timeout  # for connecting and for sending a whole Control frame
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buffer = b""
        kind, self.hello, _ = self.next()
        if kind != HELLO or self.hello.get("schema") != SCHEMA:
            raise ConnectionError(f"not a schema {SCHEMA} plant demo server: {self.hello}")
        self.command("hello", schema=SCHEMA)

    def command(self, cmd, **fields):
        self.sock.settimeout(self.timeout)  # not next()'s short poll timeout: a frame cut midway corrupts the stream
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
