#!/usr/bin/env python3
"""Summarize a ros2_tracing CTF trace for beambot perf review.

Produces a markdown report with:
  1. Top callbacks by total time spent (count * mean) — cumulative load
  2. Top callbacks by p95 latency — worst-case outliers
  3. Per-node callback counts + totals — which node is hottest

Usage:
    ros2 run beambot perf_trace_summarize.py <trace_dir> [--top N] [--out report.md]

Default is top 20, written to stdout.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil


def format_duration(td: pd.Timedelta | np.timedelta64 | float) -> str:
    """Format a duration as ms (or us for sub-ms)."""
    if isinstance(td, (pd.Timedelta, np.timedelta64)):
        ms = pd.Timedelta(td).total_seconds() * 1000.0
    else:
        ms = float(td) * 1000.0
    if ms < 1.0:
        return f"{ms * 1000:.1f} us"
    if ms < 1000.0:
        return f"{ms:.2f} ms"
    return f"{ms / 1000:.2f} s"


def collect_callback_stats(util: Ros2DataModelUtil) -> pd.DataFrame:
    """Build a per-callback stats DataFrame: symbol, owner, count, mean, p50, p95, max, total.

    Iterates over callback objects from the instances table directly and looks
    up symbols via a safe .get() — tolerates missing symbol rows, which happen
    whenever a callback was registered before tracing started (pre-init) but
    executed after (so only the instance event is in the trace, not the
    register event). util.get_callback_symbols() raises KeyError on these;
    we skip them gracefully with a synthetic "<unknown symbol>".
    """
    data = util.data
    callback_instances = data.callback_instances
    callback_symbols = data.callback_symbols  # DataFrame indexed by callback_object
    callback_objects = set(callback_instances['callback_object'])

    rows = []
    for cb_obj in callback_objects:
        try:
            durations_df = util.get_callback_durations(cb_obj)
        except Exception:
            continue
        if durations_df is None or durations_df.empty:
            continue
        durs = durations_df['duration']
        durs_s = durs.dt.total_seconds() if hasattr(durs, 'dt') else pd.to_timedelta(durs).dt.total_seconds()
        count = int(len(durs_s))
        if count == 0:
            continue

        # Safe symbol lookup: some callback_objects legitimately lack
        # registration events in the trace window.
        try:
            raw_symbol = callback_symbols.loc[cb_obj, 'symbol']
            symbol = util._prettify(raw_symbol) if hasattr(util, '_prettify') else str(raw_symbol)
        except (KeyError, AttributeError):
            symbol = f'<unknown symbol cb={cb_obj}>'

        try:
            owner = util.get_callback_owner_info(cb_obj) or '(unknown)'
        except Exception:
            owner = '(unknown)'

        rows.append({
            'symbol': symbol,
            'owner': owner,
            'count': count,
            'mean_s': float(durs_s.mean()),
            'p50_s': float(durs_s.median()),
            'p95_s': float(durs_s.quantile(0.95)),
            'max_s': float(durs_s.max()),
            'total_s': float(durs_s.sum()),
        })
    return pd.DataFrame(rows)


def truncate(text: str, width: int) -> str:
    if len(text) <= width:
        return text
    return text[: max(0, width - 1)] + '…'


def md_table(df: pd.DataFrame, columns: list[tuple[str, str, int]]) -> str:
    """Emit a markdown table. columns = [(df_col, header, width), ...]."""
    header = '| ' + ' | '.join(truncate(h, w) for _, h, w in columns) + ' |'
    sep = '|' + '|'.join('-' * (w + 2) for _, _, w in columns) + '|'
    lines = [header, sep]
    for _, row in df.iterrows():
        cells = []
        for col, _, w in columns:
            val = row[col]
            if isinstance(val, float) and col.endswith('_s'):
                cells.append(truncate(format_duration(val), w))
            elif isinstance(val, (int, np.integer)):
                cells.append(str(val))
            else:
                cells.append(truncate(str(val), w))
        lines.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('trace_dir', type=Path, help='CTF trace directory')
    ap.add_argument('--top', type=int, default=20, help='Show top N rows per table (default 20)')
    ap.add_argument('--out', type=Path, default=None, help='Write report to this file (default: stdout)')
    args = ap.parse_args()

    if not args.trace_dir.exists():
        print(f"ERROR: trace dir not found: {args.trace_dir}", file=sys.stderr)
        return 2

    print(f"Loading trace from {args.trace_dir}...", file=sys.stderr)
    events = load_file(str(args.trace_dir))
    handler = Ros2Handler.process(events)
    util = Ros2DataModelUtil(handler)

    print("Collecting callback stats...", file=sys.stderr)
    df = collect_callback_stats(util)
    if df.empty:
        print("No callback data found in trace.", file=sys.stderr)
        return 1

    n = args.top

    # Top N by cumulative time
    top_total = df.sort_values('total_s', ascending=False).head(n)
    # Top N by p95 latency (only include callbacks with enough samples to be meaningful)
    top_p95 = df[df['count'] >= 3].sort_values('p95_s', ascending=False).head(n)

    # Per-owner rollup
    per_owner = (
        df.groupby('owner')
          .agg(n_callbacks=('symbol', 'count'),
               total_count=('count', 'sum'),
               total_s=('total_s', 'sum'))
          .sort_values('total_s', ascending=False)
          .reset_index()
          .head(n)
    )

    out_lines: list[str] = []
    out_lines.append("# Beambot perf trace summary")
    out_lines.append("")
    out_lines.append(f"- **Trace:** `{args.trace_dir}`")
    out_lines.append(f"- **Callbacks observed:** {len(df)}")
    out_lines.append(f"- **Total callback time:** {format_duration(df['total_s'].sum())}")
    out_lines.append("")
    out_lines.append(f"## Top {n} callbacks by cumulative time (count × mean)")
    out_lines.append("")
    out_lines.append(md_table(top_total, [
        ('symbol', 'Callback', 60),
        ('owner', 'Owner', 45),
        ('count', 'Count', 7),
        ('mean_s', 'Mean', 10),
        ('p95_s', 'p95', 10),
        ('total_s', 'Total', 10),
    ]))
    out_lines.append("")
    out_lines.append(f"## Top {n} callbacks by p95 latency (>=3 samples)")
    out_lines.append("")
    out_lines.append(md_table(top_p95, [
        ('symbol', 'Callback', 60),
        ('owner', 'Owner', 45),
        ('count', 'Count', 7),
        ('p50_s', 'p50', 10),
        ('p95_s', 'p95', 10),
        ('max_s', 'Max', 10),
    ]))
    out_lines.append("")
    out_lines.append(f"## Top {n} owners by cumulative time")
    out_lines.append("")
    out_lines.append(md_table(per_owner, [
        ('owner', 'Owner (node / topic / service)', 55),
        ('n_callbacks', '# cbs', 7),
        ('total_count', 'Invocations', 12),
        ('total_s', 'Total', 10),
    ]))
    out_lines.append("")
    out_lines.append("---")
    out_lines.append(f"_For publish/receive latency and timeline, run "
                     f"`ros2 run tracetools_analysis auto {args.trace_dir}`._")

    report = '\n'.join(out_lines) + '\n'
    if args.out:
        args.out.write_text(report)
        print(f"Wrote report to {args.out}", file=sys.stderr)
    else:
        sys.stdout.write(report)

    return 0


if __name__ == '__main__':
    sys.exit(main())
