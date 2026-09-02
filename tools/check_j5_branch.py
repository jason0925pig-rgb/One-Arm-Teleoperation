#!/usr/bin/env python3
"""Report the joint-5 (state/action index 4) 2*pi branch of every episode.

The JAKA controller reports joint 5 on one of two branches that differ by
exactly one turn.  The branch is picked once and then held for the whole
episode -- no episode in the dataset has an internal jump -- so a wrong branch
is a clean constant offset, not a discontinuity.  It still splits the state
distribution in two, and a task collected across a switch trains on a bimodal
input.  That is what happened to the red-parcel task (19 episodes on one
branch, 31 on the other).

The dataset as of 2026-09-02: episodes 0-68 on the positive branch, 69-200 on
the negative one, switched exactly once.  Mug, stapler and screwdriver are
wholly on the negative branch.

Run this between rounds or after a session, never during a recording.

    python3 tools/check_j5_branch.py [--data-root /home/nvidia/work/telop/onearm_Tele]
"""
import argparse
import glob
import math
import os
import sys

TAU = 2.0 * math.pi
JOINT = 4  # joint 5, zero-indexed into the 8-wide state/action vectors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default="/home/nvidia/work/telop/onearm_Tele")
    parser.add_argument("--joint", type=int, default=JOINT)
    parser.add_argument("--last", type=int, default=0,
                        help="only report the last N episodes (0 = all)")
    args = parser.parse_args()

    try:
        import pyarrow.parquet as pq
    except ImportError:
        print("ERROR: pyarrow is missing. Use the LeRobot venv python:", file=sys.stderr)
        print("  /home/nvidia/work/telop/.venvs/onearm-lerobot/bin/python "
              "tools/check_j5_branch.py", file=sys.stderr)
        return 2

    meta = os.path.join(args.data_root, "lerobot_dataset", "meta", "episodes")
    files = sorted(glob.glob(os.path.join(meta, "**", "*.parquet"), recursive=True))
    if not files:
        print("ERROR: no episode metadata under " + meta, file=sys.stderr)
        return 2

    rows = []
    for path in files:
        table = pq.read_table(path).to_pydict()
        for i in range(len(table["episode_index"])):
            tasks = table["tasks"][i]
            rows.append((
                table["episode_index"][i],
                tasks[0] if tasks else "",
                table["stats/observation.state/mean"][i][args.joint],
                table["stats/action/mean"][i][args.joint],
                table["stats/observation.state/max"][i][args.joint]
                - table["stats/observation.state/min"][i][args.joint],
            ))
    rows.sort()

    # The two branches are separated by one turn, so the midpoint between the
    # extreme means is a safe split regardless of which branch dominates.
    means = sorted(r[2] for r in rows)
    split = (means[0] + means[-1]) / 2.0 if means[-1] - means[0] > TAU / 2 else means[-1] + 1.0

    per_task = {}
    for ep, task, s, a, rng in rows:
        lo = s < split
        counts = per_task.setdefault(task, [0, 0, []])
        counts[0 if lo else 1] += 1
        if rng > math.pi:
            counts[2].append(ep)

    print("joint index %d, branch split at %.4f (2*pi = %.4f)" % (args.joint, split, TAU))
    print()
    print("%-6s %-6s %-6s  %s" % ("lower", "upper", "jumps", "task"))
    problems = 0
    for task in sorted(per_task):
        lo, hi, jumps = per_task[task]
        mark = ""
        if lo and hi:
            mark = "   <== SPLIT ACROSS BOTH BRANCHES"
            problems += 1
        if jumps:
            mark += "   <== %d episode(s) with an internal jump: %s" % (len(jumps), jumps[:5])
            problems += 1
        print("%-6d %-6d %-6d  %s%s" % (lo, hi, len(jumps), task, mark))

    tail = rows[-args.last:] if args.last else []
    if tail:
        print()
        print("last %d episodes:" % len(tail))
        for ep, task, s, a, rng in tail:
            print("  ep%-4d state=%8.4f action=%8.4f range=%6.4f  %s"
                  % (ep, s, a, rng, "lower" if s < split else "UPPER"))

    print()
    if problems:
        print("RESULT: %d task(s) need attention before training." % problems)
        return 1
    print("RESULT: every task sits wholly on one branch, no internal jumps.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
