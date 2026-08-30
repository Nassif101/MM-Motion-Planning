#!/usr/bin/env python3
"""Read-only inspection of standard ROS 2 log files.

This project implementation is informed by the host-log inspection capability in
adityakamath/ros2-skill (Apache-2.0) but contains no copied upstream source.
It intentionally has no ROS client, subprocess, or network dependency.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import os
from pathlib import Path
import re
import sys
from typing import Iterable, Iterator


LOG_PATTERN = re.compile(
    r"^\[(?P<level>DEBUG|INFO|WARN|WARNING|ERROR|FATAL|CRITICAL)\]\s+"
    r"\[(?P<stamp>\d+(?:\.\d+)?)\]"
    r"(?:\s+\[(?P<node>[^\]]+)\])?:\s*(?P<message>.*)$"
)
LEVELS = {"DEBUG": 0, "INFO": 1, "WARN": 2, "ERROR": 3, "FATAL": 4}
NORMALIZE_LEVEL = {"WARNING": "WARN", "CRITICAL": "FATAL"}


def default_log_dir() -> Path:
    if value := os.environ.get("ROS_LOG_DIR"):
        return Path(value).expanduser()
    if value := os.environ.get("ROS_HOME"):
        return Path(value).expanduser() / "log"
    return Path.home() / ".ros" / "log"


def runs(log_dir: Path) -> list[dict]:
    if not log_dir.is_dir():
        return []
    found = []
    for directory in log_dir.iterdir():
        if not directory.is_dir() or directory.name == "latest":
            continue
        files = sorted(directory.glob("*.log"))
        if not files:
            continue
        modified = max(path.stat().st_mtime for path in files)
        found.append(
            {
                "run": directory.name,
                "path": str(directory),
                "files": len(files),
                "bytes": sum(path.stat().st_size for path in files),
                "modified": dt.datetime.fromtimestamp(
                    modified, tz=dt.timezone.utc
                ).isoformat(),
            }
        )
    return sorted(found, key=lambda item: item["modified"], reverse=True)


def resolve_run(log_dir: Path, name: str) -> Path:
    if name != "latest":
        candidate = log_dir / name
        if not candidate.is_dir():
            raise FileNotFoundError(f"ROS log run not found: {candidate}")
        return candidate
    latest = log_dir / "latest"
    if latest.exists():
        return latest.resolve()
    available = runs(log_dir)
    if not available:
        raise FileNotFoundError(f"No ROS log runs found under {log_dir}")
    return Path(available[0]["path"])


def parse_entries(run_dir: Path) -> Iterator[dict]:
    for path in sorted(run_dir.glob("*.log")):
        try:
            with path.open(encoding="utf-8", errors="replace") as stream:
                for line_number, line in enumerate(stream, start=1):
                    match = LOG_PATTERN.match(line.rstrip())
                    if not match:
                        continue
                    level = NORMALIZE_LEVEL.get(
                        match.group("level"), match.group("level")
                    )
                    yield {
                        "level": level,
                        "stamp": float(match.group("stamp")),
                        "node": match.group("node") or "",
                        "message": match.group("message"),
                        "file": path.name,
                        "line": line_number,
                    }
        except OSError:
            continue


def filtered_entries(
    entries: Iterable[dict],
    *,
    minimum_level: str,
    node: str | None,
    contains: str | None,
    regex: str | None,
) -> Iterator[dict]:
    expression = re.compile(regex, re.IGNORECASE) if regex else None
    threshold = LEVELS[minimum_level]
    for entry in entries:
        if LEVELS[entry["level"]] < threshold:
            continue
        if node and node.lower() not in entry["node"].lower():
            continue
        if contains and contains.lower() not in entry["message"].lower():
            continue
        if expression and not expression.search(entry["message"]):
            continue
        yield entry


def emit(value: object) -> None:
    json.dump(value, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")


def command_runs(args: argparse.Namespace) -> None:
    emit({"log_dir": str(args.log_dir), "runs": runs(args.log_dir)})


def selected(args: argparse.Namespace) -> tuple[Path, list[dict]]:
    run_dir = resolve_run(args.log_dir, args.run)
    entries = list(
        filtered_entries(
            parse_entries(run_dir),
            minimum_level=args.min_level,
            node=args.node,
            contains=args.contains,
            regex=args.regex,
        )
    )
    return run_dir, entries[-args.limit :] if args.limit else entries


def command_query(args: argparse.Namespace) -> None:
    run_dir, entries = selected(args)
    emit({"run": str(run_dir), "count": len(entries), "entries": entries})


def command_summary(args: argparse.Namespace) -> None:
    run_dir, entries = selected(args)
    by_level = collections.Counter(entry["level"] for entry in entries)
    by_node = collections.Counter(entry["node"] or "<unknown>" for entry in entries)
    emit(
        {
            "run": str(run_dir),
            "count": len(entries),
            "by_level": dict(sorted(by_level.items())),
            "by_node": dict(by_node.most_common(args.top)),
        }
    )


def add_filters(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--run", default="latest", help="Run directory name")
    parser.add_argument(
        "--min-level", choices=tuple(LEVELS), default="DEBUG", help="Severity floor"
    )
    parser.add_argument("--node", help="Case-insensitive node substring")
    text = parser.add_mutually_exclusive_group()
    text.add_argument("--contains", help="Case-insensitive message substring")
    text.add_argument("--regex", help="Case-insensitive message regular expression")
    parser.add_argument("--limit", type=int, default=200, help="Keep newest N matches")


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    root.add_argument(
        "--log-dir", type=Path, default=default_log_dir(), help="ROS log root"
    )
    commands = root.add_subparsers(dest="command", required=True)
    run_parser = commands.add_parser("runs", help="List log runs")
    run_parser.set_defaults(handler=command_runs)
    query_parser = commands.add_parser("query", help="Query standard ROS log entries")
    add_filters(query_parser)
    query_parser.set_defaults(handler=command_query)
    summary_parser = commands.add_parser("summary", help="Summarize matching entries")
    add_filters(summary_parser)
    summary_parser.add_argument("--top", type=int, default=20, help="Top node count")
    summary_parser.set_defaults(handler=command_summary)
    return root


def main() -> int:
    args = parser().parse_args()
    try:
        args.handler(args)
    except (FileNotFoundError, re.error, ValueError) as error:
        emit({"error": str(error)})
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
