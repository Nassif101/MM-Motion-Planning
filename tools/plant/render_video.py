"""Renders a directed run to an MP4 (spec 4C Task 5): the viewer player in render mode, then ffmpeg (host, macOS).

    python3 tools/plant/render_video.py LOG JOURNAL OUT.mp4 [--speed S] [--from T] [--to T] [--png] [--quality NAME]
                                       [--app PATH] [--keep-frames]

LOG is a run log (a demo session's --log file, or a campaign's run.mmlog). JOURNAL is a journal file directed on it in
the player's replay mode (-replay LOG). Before writing a frame the script estimates the frames' size and refuses when
the disk is too small. The player renders every frame at 1920 x 1080, at a fixed 60 frames per video second, and quits.
The frames are encoded with ffmpeg (H.264, CRF 18) and then deleted.
"""
import argparse
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

FPS = 60
BYTES_PER_FRAME = {False: 0.6e6, True: 3.5e6}  # a 1920 x 1080 frame, JPEG quality 95 or PNG (measured order)
MARGIN = 1.5
DEFAULT_APP = Path(__file__).resolve().parents[2] / "motion-planning-sim/Builds/PlantViewer-macOS/PlantViewer.app"


def finite(text):
    """An argparse type: a finite number (Task 5 review I2: nan or inf would fail in the player)."""
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number") from None
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"{text!r} is not finite")
    return value


def render_timeout(frames):
    """Seconds to wait for the player (Task 5 review I1): start-up, then a generous 0.5 s per frame (measured: 0.06)."""
    return 300 + 0.5 * frames


def player_executable(app):
    """The player's binary inside its .app bundle. The render runs it directly: launched through `open`, a render
    stalled twice with no frame written (2026-11-22), and a direct child can be stopped exactly."""
    found = sorted(Path(app, "Contents", "MacOS").glob("*")) if Path(app, "Contents", "MacOS").is_dir() else []
    if len(found) != 1:
        raise FileNotFoundError(f"no single player binary in {app}/Contents/MacOS (build it first)")
    return found[0]


def frame_count(start_t, end_t, from_t, to_t, speed):
    """Frames for [from, to] clamped to the log's [start, end]; frame n at from + n speed / 60 (RenderPlan.Frames)."""
    if not speed > 0:
        raise ValueError("speed must be positive")
    a = max(start_t, start_t if from_t is None else from_t)
    b = min(end_t, end_t if to_t is None else to_t)
    if not b >= a:
        return 0
    return int(math.floor((b - a) / (speed / FPS) + 1e-9)) + 1


def estimate_bytes(frames, png):
    return frames * BYTES_PER_FRAME[png] * MARGIN


def check_disk(frames, png, free):
    """None when the frames fit in `free` bytes, else the reason to refuse."""
    need = estimate_bytes(frames, png)
    if need > free:
        return (f"not enough disk for {frames} frames: about {need / 1e9:.1f} GB needed, {free / 1e9:.1f} GB free "
                f"(shorten with --from/--to{'' if not png else ', or drop --png'})")
    return None


def ffmpeg_command(frames_dir, out, png):
    ext = "png" if png else "jpg"
    return ["ffmpeg", "-y", "-framerate", str(FPS), "-i", str(Path(frames_dir) / f"frame_%06d.{ext}"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(out)]


def player_args(log, journal, frames_dir, speed, start, end, png, quality):
    args = ["-render", str(log), "-journal", str(journal), "-out", str(frames_dir), "-speed", str(speed),
            "-screen-width", "1920", "-screen-height", "1080", "-screen-fullscreen", "0"]
    if start is not None:
        args += ["-from", str(start)]
    if end is not None:
        args += ["-to", str(end)]
    if png:
        args += ["-png"]
    if quality:
        args += ["-quality", quality]
    return args


def log_span(log):
    """The first and last State times of a run log."""
    sys.path.insert(0, str(Path(__file__).parent))
    import plant_stream as ps
    first = last = None
    for kind, header, _ in ps.RunLog(log).frames():
        if kind == ps.STATE:
            first = header["t"] if first is None else first
            last = header["t"]
    if first is None:
        raise ValueError(f"{log} has no States")
    return first, last


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("log", type=Path)
    p.add_argument("journal", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--speed", type=finite, default=1.0)
    p.add_argument("--from", dest="start", type=finite)
    p.add_argument("--to", dest="end", type=finite)
    p.add_argument("--png", action="store_true")
    p.add_argument("--quality")
    p.add_argument("--app", type=Path, default=DEFAULT_APP)
    p.add_argument("--keep-frames", action="store_true")
    a = p.parse_args(argv)
    a.log, a.journal, a.out = a.log.resolve(), a.journal.resolve(), a.out.resolve()  # the player runs from /
    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg not found: brew install ffmpeg")
    start_t, end_t = log_span(a.log)
    frames = frame_count(start_t, end_t, a.start, a.end, a.speed)
    if frames == 0:
        sys.exit(f"nothing to render: the log spans {start_t:.2f}-{end_t:.2f} s")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    frames_dir = Path(tempfile.mkdtemp(prefix="render-", dir=a.out.parent))
    reason = check_disk(frames, a.png, shutil.disk_usage(frames_dir).free)
    if reason:
        frames_dir.rmdir()
        sys.exit(reason)
    print(f"rendering {frames} frames ({frames / FPS:.1f} s of video) into {frames_dir}")
    try:
        player_log = frames_dir / "player.log"
        player = subprocess.Popen(["caffeinate", "-d", "-i", str(player_executable(a.app)), "-logFile", str(player_log),
                                   *player_args(a.log, a.journal, frames_dir, a.speed, a.start, a.end, a.png, a.quality)],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            player.wait(timeout=render_timeout(frames))
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as stopped:
            player.kill()  # caffeinate and the player it runs
            subprocess.run(["pkill", "-f", f"-out {frames_dir}"], check=False)
            tail = player_log.read_text(errors="replace").splitlines()[-5:] if player_log.exists() else []
            sys.exit(f"the render was stopped ({type(stopped).__name__}); the player was ended. Log tail:\n" +
                     "\n".join(tail))
        summary_path = frames_dir / "render-summary.json"
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        if not summary.get("ok"):
            sys.exit(f"the render failed: {summary.get('refusal') or summary.get('error') or 'no summary written'}")
        if summary.get("warning"):
            print("warning:", summary["warning"])
        written = summary.get("frames", 0)
        if written != frames:
            print(f"warning: {written} frames written, {frames} expected")
        subprocess.run(ffmpeg_command(frames_dir, a.out, a.png), check=True)
        shutil.copy(summary_path, a.out.with_suffix(".render.json"))
        print(f"{a.out}: {written} frames, {written / FPS:.1f} s")
    finally:
        if not a.keep_frames:
            shutil.rmtree(frames_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
