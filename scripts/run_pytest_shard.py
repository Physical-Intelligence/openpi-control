#!/usr/bin/env python3
"""Collect pytest tests and run one deterministic, round-robin shard."""

from __future__ import annotations

import argparse
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shard-index", required=True, type=int)
    parser.add_argument("--shard-count", required=True, type=int)
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args()

    if args.shard_count < 1:
        parser.error("--shard-count must be positive")
    if not 0 <= args.shard_index < args.shard_count:
        parser.error("--shard-index must be between zero and shard-count minus one")

    collect = subprocess.run(
        [sys.executable, "-m", "pytest", "-o", "addopts=", "--collect-only", "-q", *args.paths],
        check=False,
        capture_output=True,
        text=True,
    )
    if collect.returncode != 0:
        sys.stdout.write(collect.stdout)
        sys.stderr.write(collect.stderr)
        return collect.returncode

    node_ids = [line for line in collect.stdout.splitlines() if "::" in line]
    selected = node_ids[args.shard_index :: args.shard_count]
    if not selected:
        print("pytest shard selected no tests", file=sys.stderr)
        return 1

    print(
        f"Running pytest shard {args.shard_index + 1}/{args.shard_count}: "
        f"{len(selected)} of {len(node_ids)} tests",
        flush=True,
    )
    return subprocess.run([sys.executable, "-m", "pytest", "-v", *selected], check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
