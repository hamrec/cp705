#!/usr/bin/env python3
"""
Generate YouTube timestamps from:
  - .MOV (creation_time UTC)
  - .log (T/R lines)
  - .adi (ADIF)

For each ADIF call, find the first time the log contains the adjacent token pair:
  "<CALL> AG6AQ"
Then compute offset = log_time - video_start_time and print:
  HH:MM:SS CALL
"""

from __future__ import annotations
import argparse
import datetime as dt
import json
import re
import shutil
import subprocess
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

LOG_RE = re.compile(r"""^\s*([TR])\s+\[(\d{8})\s+(\d{6})\]\[(.*?)\]\s*(.*?)\s*$""", re.IGNORECASE)
ADIF_FIELD_RE = re.compile(r"<\s*([a-z0-9_]+)\s*:\s*(\d+)(?::[^>]*)?\s*>", re.IGNORECASE)

MYCALL = "AG6AQ"  # change if needed

@dataclass
class LogEntry:
    ts: dt.datetime  # naive UTC
    text: str

def run_ffprobe_get_creation_time_utc(mov_path: str) -> Optional[dt.datetime]:
    if not shutil.which("ffprobe"):
        return None
    cmd = ["ffprobe", "-v", "error", "-show_entries", "format_tags=creation_time", "-of", "json", mov_path]
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
        data = json.loads(out.decode("utf-8", errors="replace"))
        ct = (data.get("format", {}).get("tags", {}) or {}).get("creation_time")
        if not ct:
            return None
        ct = ct.strip()
        if ct.endswith("Z"):
            ct = ct[:-1] + "+00:00"
        d = dt.datetime.fromisoformat(ct)
        if d.tzinfo is not None:
            d = d.astimezone(dt.timezone.utc).replace(tzinfo=None)
        return d
    except Exception:
        return None

def parse_log(log_path: str) -> List[LogEntry]:
    entries: List[LogEntry] = []
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = LOG_RE.match(line.rstrip("\n"))
            if not m:
                continue
            _, ymd, hms, _, rest = m.groups()
            try:
                ts = dt.datetime.strptime(ymd + hms, "%Y%m%d%H%M%S")  # UTC naive
            except ValueError:
                continue
            text = " ".join(rest.strip().split())
            entries.append(LogEntry(ts=ts, text=text))
    entries.sort(key=lambda e: e.ts)
    return entries

def parse_adi_calls(adi_path: str) -> List[str]:
    with open(adi_path, "r", encoding="utf-8", errors="replace") as f:
        s = f.read()

    recs = re.split(r"<\s*eor\s*>", s, flags=re.IGNORECASE)
    calls: List[str] = []
    seen: Set[str] = set()

    for rec in recs:
        rec = rec.strip()
        if not rec:
            continue

        i = 0
        call_val = None
        while True:
            m = ADIF_FIELD_RE.search(rec, i)
            if not m:
                break
            field = m.group(1).lower()
            length = int(m.group(2))
            val_start = m.end()
            val = rec[val_start:val_start + length]
            i = val_start + length
            if field == "call":
                call_val = val.strip().upper()
                break

        if call_val and call_val not in seen:
            seen.add(call_val)
            calls.append(call_val)

    return calls

def normalize_tok(tok: str) -> str:
    """Keep calls like W6RSH/P matching W6RSH (strip portable suffix/prefix)."""
    tok = tok.strip().upper()
    # If token is like "W6RSH/P" or "EA/AG6AQ", keep as-is for pair checking,
    # but also allow stripping one side around slash.
    return tok

def tok_matches_call(tok: str, call: str) -> bool:
    tok = tok.upper()
    call = call.upper()
    return tok == call or tok.startswith(call + "/") or tok.endswith("/" + call)

def find_first_reply_time(entries: List[LogEntry], dx_call: str, my_call: str) -> Optional[dt.datetime]:
    """
    Find first time tokens contain adjacent pair either:
      "<DXCALL> <MYCALL>" OR "<MYCALL> <DXCALL>"
    Accept DXCALL with /P etc (W6RSH/P) as match.
    """
    my_call = my_call.upper()
    dx_call_u = dx_call.upper()

    for e in entries:
        toks = [normalize_tok(t) for t in e.text.split()]
        for i in range(len(toks) - 1):
            a, b = toks[i], toks[i + 1]

            # DX MY
            if tok_matches_call(a, dx_call_u) and b == my_call:
                return e.ts

            # MY DX
            if a == my_call and tok_matches_call(b, dx_call_u):
                return e.ts

    return None

def fmt_hhmmss(seconds: float) -> str:
    if seconds < 0:
        seconds = 0.0
    total = int(round(seconds))
    h = total // 3600
    m = (total % 3600) // 60
    s = total % 60
    return f"{h:02d}:{m:02d}:{s:02d}"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mov", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--adi", required=True)
    ap.add_argument("--video_start_utc", default=None, help='Override MOV creation_time: "YYYYMMDD HHMMSS" UTC')
    ap.add_argument("--mycall", default=MYCALL, help="Your callsign (default AG6AQ)")
    ap.add_argument("--min_offset", type=float, default=0.0)
    args = ap.parse_args()

    # Video start time (UTC)
    # If --video_start_utc is provided, it ALWAYS takes precedence over MOV metadata.
    if args.video_start_utc:
        video_start = dt.datetime.strptime(args.video_start_utc, "%Y%m%d %H%M%S")
    else:
        video_start = run_ffprobe_get_creation_time_utc(args.mov)
        if video_start is None:
            raise SystemExit('MOV has no readable creation_time. Provide --video_start_utc "YYYYMMDD HHMMSS" (UTC).')

    calls = parse_adi_calls(args.adi)
    if not calls:
        raise SystemExit("No calls found in ADI.")

    log_entries = parse_log(args.log)
    if not log_entries:
        raise SystemExit("No valid log entries parsed.")

    # Only keep log entries during/after the video
    log_entries = [e for e in log_entries if e.ts >= video_start]

    first_seen: Dict[str, dt.datetime] = {}
    missing: List[str] = []

    for call in calls:
        ts = find_first_reply_time(log_entries, call, args.mycall)
        if ts is None:
            missing.append(call)
        else:
            first_seen[call] = ts

    rows: List[Tuple[float, str]] = []
    for call, ts in first_seen.items():
        offset = (ts - video_start).total_seconds()
        if offset >= args.min_offset:
            rows.append((offset, call))

    rows.sort(key=lambda x: x[0])

    for offset, call in rows:
        print(f"{fmt_hhmmss(offset)} {call}")

    if missing:
        print("\n# Not found as '<CALL> {0}' in log:".format(args.mycall.upper()))
        for call in missing:
            print(f"# {call}")

if __name__ == "__main__":
    main()
