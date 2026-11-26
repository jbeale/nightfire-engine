#!/usr/bin/env python3
"""Capture or reuse a PS2 reference trace and compare both native replay modes."""

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from compare import diff, load, make_inputs, summarize
from pine import Pine, PineError
from trace import GLB_PLAYERS


@contextmanager
def launched_pcsx2(args):
    # Give this instance its own socket directory so it cannot take over another session's PINE socket.
    with tempfile.TemporaryDirectory(prefix="nf-pine-", dir="/tmp") as runtime:
        env = dict(os.environ, TMPDIR=runtime, XDG_RUNTIME_DIR=runtime)
        socket_name = "pcsx2.sock" if args.pine_slot == 28011 else f"pcsx2.sock.{args.pine_slot}"
        socket_path = Path(runtime, socket_name)
        log_path = args.out / "pcsx2.log"
        with log_path.open("w") as log:
            emulator = subprocess.Popen(
                [args.pcsx2, "-batch", "-nofullscreen", "-earlyconsolelog", "-datapath", args.profile.resolve(),
                 "-fastboot", "-statefile", args.statefile.resolve(), "--", args.iso.resolve()],
                env=env, stdout=log, stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + args.timeout
                while time.monotonic() < deadline and emulator.poll() is None:
                    try:
                        pine = Pine(path=socket_path, timeout=1.0)
                        try:
                            if pine.game_id() == "SLUS-20579" and pine.read32(GLB_PLAYERS) != 0:
                                break
                        finally:
                            pine.close()
                    except (OSError, PineError):
                        pass
                    time.sleep(0.1)
                else:
                    raise RuntimeError(f"PCSX2 did not reach the saved on-foot level. See {log_path}.")
                yield socket_path
            finally:
                if emulator.poll() is None:
                    emulator.terminate()
                    try:
                        emulator.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        emulator.kill()
                        emulator.wait()


def capture(args, path, socket_path=None):
    command = [sys.executable, Path(__file__).with_name("trace.py"), path, "--frames", str(args.frames),
               "--pine-slot", str(args.pine_slot), "--timeout", str(args.timeout), "--level", args.level[:-4]]
    if socket_path is not None:
        command += ["--pine-socket", socket_path]
    if args.load_slot is not None:
        command += ["--load-slot", str(args.load_slot)]
    if args.script is not None:
        command += ["--script", args.script]
    subprocess.run(command, check=True)


def compare_replays(args, oracle):
    records = load(oracle)
    frames = [record["frame"] for record in records]
    if len(frames) < 2 or any(b <= a for a, b in zip(frames, frames[1:])):
        raise RuntimeError("The reference needs at least two records with strictly increasing frame numbers.")
    missed = frames[-1] - frames[0] + 1 - len(frames)
    if missed > args.max_missed:
        raise RuntimeError(f"Reference missed {missed} frames (limit {args.max_missed}).")
    for record in records:
        if "level" in record and record["level"] != int(args.level[:-4], 16):
            raise RuntimeError("The reference was captured in a different level.")
        if not all(math.isfinite(v) for v in (*record["pos"], record["yaw"], *record["act"])):
            raise RuntimeError("The reference contains non-finite player state.")

    inputs = args.out / "capture.inputs"
    make_inputs(oracle, inputs)
    report = {"level": args.level, "reference_first": frames[0], "reference_last": frames[-1],
              "missed": missed, "reference": str(oracle.resolve()),
              "reference_displacement_cm": 100 * math.dist(records[0]["pos"], records[-1]["pos"]),
              "replays": {}}
    for name, extra in (("free", []), ("synced", ["--sync"])):
        replay = args.out / f"{name}.jsonl"
        with (args.out / f"{name}.log").open("w") as log:
            subprocess.run([args.nfgame.resolve(), args.disc.resolve(), args.level, "--no-mission", *extra,
                            "--inputs", inputs, "--trace", replay], check=True, stdout=log,
                           stderr=subprocess.STDOUT, timeout=args.timeout)
        rows = diff(oracle, replay)
        if len(rows) != len(frames) - 1:
            raise RuntimeError(f"{name} replay did not cover every recorded reference frame.")
        if any(not math.isfinite(row[k]) for row in rows for k in ("pos_err", "yaw_err", "act_err")):
            raise RuntimeError(f"{name} replay produced non-finite comparison data.")
        print(summarize(name, rows))
        report["replays"][name] = {
            "frames": len(rows), "max_position_cm": 100 * max(r["pos_err"] for r in rows),
            "end_position_cm": 100 * rows[-1]["pos_err"], "max_yaw_rad": max(r["yaw_err"] for r in rows),
            "max_action_error": max(r["act_err"] for r in rows),
            "flag_mismatches": sum(r["flag_mismatch"] for r in rows),
        }
    with (args.out / "report.json").open("w") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")
    if args.max_error_cm is not None:
        for name, result in report["replays"].items():
            if result["max_position_cm"] > args.max_error_cm:
                raise RuntimeError(f"{name} replay exceeded the {args.max_error_cm:g} cm position limit.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("disc", type=Path, help="extracted US PS2 disc directory")
    parser.add_argument("nfgame", type=Path)
    parser.add_argument("level", help="the on-foot level recorded in PCSX2, for example 07000024.bin")
    parser.add_argument("out", type=Path, help="new directory for captures, logs and report")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--trace", type=Path, help="reuse an existing reference JSONL")
    source.add_argument("--live", action="store_true", help="record an already-running PCSX2 instance")
    source.add_argument("--pcsx2", type=Path, help="start and stop this PCSX2 executable from a savestate")
    parser.add_argument("--profile", type=Path, help="isolated PCSX2 -datapath root with PINE enabled")
    parser.add_argument("--iso", type=Path, help="original disc image for PCSX2")
    parser.add_argument("--statefile", type=Path, help="saved on-foot state for --pcsx2")
    parser.add_argument("--load-slot", type=int, help="reload a state slot in an already-running instance")
    parser.add_argument("--pine-slot", type=int, default=28011)
    parser.add_argument("--script", type=Path, help="drive a live capture with an existing vpad socket bridge")
    parser.add_argument("--frames", type=int, default=150)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--max-missed", type=int, default=0)
    parser.add_argument("--max-error-cm", type=float, help="fail if either replay exceeds this position error")
    args = parser.parse_args()
    if len(args.level) != 12 or not args.level.endswith(".bin") or any(c not in "0123456789abcdef" for c in args.level[:8]):
        parser.error("level must be a lowercase eight-digit hex filename ending with .bin")
    if args.frames < 1 or not math.isfinite(args.timeout) or args.timeout <= 0 or args.max_missed < 0:
        parser.error("frames and timeout must be positive and max-missed must be nonnegative")
    if args.max_error_cm is not None and (not math.isfinite(args.max_error_cm) or args.max_error_cm < 0):
        parser.error("max-error-cm must be finite and nonnegative")
    if args.pcsx2 and not (args.profile and args.iso and args.statefile):
        parser.error("--pcsx2 needs --profile, --iso and --statefile")
    if not args.pcsx2 and any((args.profile, args.iso, args.statefile)):
        parser.error("--profile, --iso and --statefile require --pcsx2")
    if args.load_slot is not None and not args.live:
        parser.error("--load-slot requires --live")
    if args.script and not args.live:
        parser.error("--script requires a live session and its vpad bridge")

    args.out.mkdir(parents=True, exist_ok=False)
    oracle = args.trace if args.trace else args.out / "oracle.jsonl"
    if args.pcsx2:
        with launched_pcsx2(args) as socket_path:
            capture(args, oracle, socket_path)
    elif args.live:
        capture(args, oracle)
    compare_replays(args, oracle)
    print(f"Comparison report: {args.out / 'report.json'}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        raise SystemExit(f"Comparison failed: {error}") from error
