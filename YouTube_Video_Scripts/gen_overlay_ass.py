#!/usr/bin/env python3
from __future__ import annotations
import argparse
import datetime as dt
import json
import math
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from typing import List, Optional, Tuple

LINE_RE = re.compile(r"""^\s*([TR])\s+\[(\d{8})\s+(\d{6})\]\[(.*?)\]\s*(.*?)\s*$""")

# ASS colors are BGR: &HBBGGRR&
ASS_RED   = "&H0000FF&"
ASS_GREEN = "&H00FF00&"

FONT_SCALE = 1.5  # +50% requested
MONO_ADV_FACTOR = 0.60  # approx monospace advance width ≈ factor * font_size (pixels)
PAD_X_FACTOR    = 0.60  # pad_x ≈ factor * font_size
ASS_OUTLINE_PX  = 2     # matches Style outline value used below


DEFAULT_MAX_CHARS = 40  # clamp for on-screen text; adjust via --max_chars
TX_TTL = 12.0
RX_TTL = 12.0
RX_MAX_LINES = 6
RX_SLOT_WINDOW_SEC = 4.0

@dataclass
class Entry:
    kind: str
    ts: dt.datetime          # naive UTC
    freq: str
    text: str

def clamp_text(s: str, max_chars: int) -> str:
    s = " ".join(s.strip().split())
    return s if len(s) <= max_chars else s[:max_chars]

def parse_log(path: str) -> List[Entry]:
    out: List[Entry] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = LINE_RE.match(line.strip())
            if not m:
                continue
            kind, ymd, hms, freq, rest = m.groups()
            try:
                ts = dt.datetime.strptime(ymd + hms, "%Y%m%d%H%M%S")  # UTC (naive)
            except ValueError:
                continue
            out.append(Entry(kind=kind, ts=ts, freq=freq.strip(), text=" ".join(rest.split())))
    out.sort(key=lambda e: e.ts)
    return out

def ass_time(seconds: float) -> str:
    if seconds < 0:
        seconds = 0.0
    total_cs = int(round(seconds * 100.0))
    cs = total_cs % 100
    total_s = total_cs // 100
    s = total_s % 60
    total_m = total_s // 60
    m = total_m % 60
    h = total_m // 60
    return f"{h}:{m:02d}:{s:02d}.{cs:02d}"

def run_ffprobe_json(mov_path: str) -> dict:
    if not shutil.which("ffprobe"):
        raise SystemExit("ffprobe not found. Install ffmpeg or provide --res and --video_start_utc.")
    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries",
        "stream=width,height:stream_tags=rotate,creation_time:"
        "format_tags=creation_time",
        "-of", "json",
        mov_path,
    ]
    out = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
    return json.loads(out.decode("utf-8", errors="replace"))

def parse_creation_time_utc(s: str) -> Optional[dt.datetime]:
    s = s.strip()
    try:
        if s.endswith("Z"):
            s = s[:-1] + "+00:00"
        d = dt.datetime.fromisoformat(s)
        if d.tzinfo is not None:
            d = d.astimezone(dt.timezone.utc).replace(tzinfo=None)
        return d
    except Exception:
        return None

def get_mov_info(mov_path: str) -> Tuple[int, int, int, Optional[dt.datetime]]:
    data = run_ffprobe_json(mov_path)
    st = (data.get("streams") or [{}])[0]
    w = int(st.get("width", 0))
    h = int(st.get("height", 0))
    rotate = int((st.get("tags") or {}).get("rotate", 0) or 0) % 360

    ct = None
    ct_s = (st.get("tags") or {}).get("creation_time")
    if ct_s:
        ct = parse_creation_time_utc(ct_s)
    if ct is None:
        ct_s2 = (data.get("format", {}).get("tags") or {}).get("creation_time")
        if ct_s2:
            ct = parse_creation_time_utc(ct_s2)

    # Use display orientation if rotate is 90/270
    if rotate in (90, 270):
        w, h = h, w

    if w <= 0 or h <= 0:
        raise SystemExit("Could not read width/height from MOV.")
    return w, h, rotate, ct

def parse_res(s: str) -> Tuple[int, int]:
    m = re.match(r"^\s*(\d+)\s*[xX]\s*(\d+)\s*$", s)
    if not m:
        raise ValueError("Resolution must look like 1920x1080")
    return int(m.group(1)), int(m.group(2))

@dataclass
class Segment:
    start: float
    end: float
    tx: str
    rx_lines: List[str]  # length RX_MAX_LINES

def build_segments(entries: List[Entry], video_start: dt.datetime, offset: float, max_chars: int) -> List[Segment]:
    def rel(ts: dt.datetime) -> float:
        return (ts - video_start).total_seconds() + offset

    # Events: (time, ("set_tx", id, text)), (time, ("clr_tx", id))
    #         (time, ("set_rx", line_idx, id, text)), (time, ("clr_rx", line_idx, id))
    events: List[Tuple[float, Tuple]] = []

    tx_id = 0
    for e in entries:
        if e.kind != "T":
            continue
        t = rel(e.ts)
        txt = clamp_text(e.text, max_chars)
        tx_id += 1
        my_id = tx_id
        events.append((t, ("set_tx", my_id, txt)))
        events.append((t + TX_TTL, ("clr_tx", my_id)))

        # RX slot grouping: collect all R entries within a 4s window (slot).
    # For each slot:
    #   1) de-dup messages
    #   2) sort by priority: reply-to-me, CQ, other
    #   3) keep up to RX_MAX_LINES
    #   4) all lines appear together and disappear together (RX_TTL)
    rx_global_id = 0

    def rx_prio(txt: str) -> int:
        t = txt.strip()
        if t.startswith("CQ"):
            return 1
        # Reply-to-me: contains our callsign somewhere (but not CQ ...)
        if "AG6AQ" in t:
            return 0
        return 2

    slot_start: Optional[dt.datetime] = None
    slot_msgs: List[Tuple[dt.datetime, str]] = []

    def flush_slot():
        nonlocal slot_start, slot_msgs, rx_global_id
        if slot_start is None or not slot_msgs:
            slot_start = None
            slot_msgs = []
            return

        # De-dup (keep earliest occurrence)
        seen = set()
        uniq: List[Tuple[dt.datetime, str]] = []
        for ts, raw in sorted(slot_msgs, key=lambda x: x[0]):
            txt = clamp_text(raw, max_chars)
            if txt in seen:
                continue
            seen.add(txt)
            uniq.append((ts, txt))

        uniq.sort(key=lambda x: (rx_prio(x[1]), x[0]))
        chosen = uniq[:RX_MAX_LINES]

        t0 = max(0.0, rel(slot_start))
        t1 = t0 + RX_TTL

        # Clear all RX lines at slot start to prevent any overlap artifacts.
        for i in range(RX_MAX_LINES):
            events.append((t0, ("clr_rx_any", i)))

        for i, (_, txt) in enumerate(chosen):
            rx_global_id += 1
            my_id = rx_global_id
            events.append((t0, ("set_rx", i, my_id, txt)))
            events.append((t1, ("clr_rx", i, my_id)))

        slot_start = None
        slot_msgs = []

    for e in entries:
        if e.kind != "R":
            continue
        if slot_start is None:
            slot_start = e.ts
        elif (e.ts - slot_start).total_seconds() > RX_SLOT_WINDOW_SEC:
            flush_slot()
            slot_start = e.ts
        slot_msgs.append((e.ts, e.text))

    flush_slot()

# Sort all events by time; tie-breaker: clears before sets at same timestamp
    def prio(action: Tuple) -> int:
        return 0 if action[0].startswith("clr") else 1

    events.sort(key=lambda x: (x[0], prio(x[1])))

    # Build segments by applying events
    cur_tx = ""
    cur_tx_id = 0
    cur_rx = [""] * RX_MAX_LINES
    cur_rx_ids = [0] * RX_MAX_LINES

    segs: List[Segment] = []
    cur_t: Optional[float] = None

    for t, action in events:
        t = max(0.0, t)
        if cur_t is None:
            cur_t = t

        if t > cur_t:
            segs.append(Segment(cur_t, t, cur_tx, list(cur_rx)))
            cur_t = t

        if action[0] == "set_tx":
            _, my_id, txt = action
            cur_tx = txt
            cur_tx_id = my_id

        elif action[0] == "clr_tx":
            _, my_id = action
            if my_id == cur_tx_id:
                cur_tx = ""
                cur_tx_id = 0


        elif action[0] == "clr_rx_any":
            _, idx = action
            if 0 <= idx < RX_MAX_LINES:
                cur_rx[idx] = ""
                cur_rx_ids[idx] = 0

        elif action[0] == "set_rx":
            _, idx, my_id, txt = action
            if 0 <= idx < RX_MAX_LINES:
                cur_rx[idx] = txt
                cur_rx_ids[idx] = my_id

        elif action[0] == "clr_rx":
            _, idx, my_id = action
            if 0 <= idx < RX_MAX_LINES and cur_rx_ids[idx] == my_id:
                cur_rx[idx] = ""
                cur_rx_ids[idx] = 0

    if cur_t is not None:
        last_t = max(0.0, events[-1][0])
        segs.append(Segment(cur_t, last_t + 2.0, cur_tx, list(cur_rx)))

    return [s for s in segs if s.end > s.start]

def make_ass(out_path: str, width: int, height: int, segments: List[Segment],
             margin: int, box_width: int, video_start: dt.datetime, offset: float, font_name: str, max_chars: int, font_size_override: int | None = None) -> None:
    base_font_size = max(24, int(round(height * 0.037)))
    auto_size = max(18, int(round(base_font_size * FONT_SCALE)))
    # If using a monospace font and a fixed box_width, cap font size so max_chars fit inside box.
    # total_width ≈ font_size*(2*PAD_X_FACTOR + max_chars*MONO_ADV_FACTOR) + 2*ASS_OUTLINE_PX
    fit_cap = int((box_width - 2 * ASS_OUTLINE_PX) / (2 * PAD_X_FACTOR + max_chars * MONO_ADV_FACTOR)) if box_width else auto_size
    fit_cap = max(12, fit_cap)
    if font_size_override is not None:
        font_size = int(font_size_override)
    else:
        font_size = min(auto_size, fit_cap)

    line_gap = int(round(font_size * 1.15))

    # Reserve one line for "Date Time (UTC)" above TX
    time_y = margin
    tx_y = margin + line_gap
    rx0_y = margin + 2 * line_gap

    # Right-side box, but left aligned inside it:
    x_left = max(0, width - margin - box_width)

    # Background ("obstacle layer") behind the full text area
    pad_x = int(round(font_size * 0.6))
    pad_y = int(round(font_size * 0.45))
    rect_w = box_width + 2 * pad_x
    rect_h = int(round((RX_MAX_LINES + 2) * line_gap + 2 * pad_y))  # time + TX + RX lines
    bg_x = x_left - pad_x
    bg_y = margin - pad_y

    def esc_ass(s: str) -> str:
        return s.replace("{", r"\{").replace("}", r"\}")

    max_end = max((s.end for s in segments), default=0.0)

    header = [
        "[Script Info]",
        "ScriptType: v4.00+",
        f"PlayResX: {width}",
        f"PlayResY: {height}",
        "WrapStyle: 2",
        "ScaledBorderAndShadow: yes",
        "",
        "[V4+ Styles]",
        "Format: Name,Fontname,Fontsize,PrimaryColour,SecondaryColour,OutlineColour,BackColour,"
        "Bold,Italic,Underline,StrikeOut,ScaleX,ScaleY,Spacing,Angle,BorderStyle,Outline,Shadow,"
        "Alignment,MarginL,MarginR,MarginV,Encoding",
        # Alignment=7 => top-left
        f"Style: TIME,{font_name},{font_size},&H00FFFFFF&,&H000000FF&,&H00000000&,&H64000000&,0,0,0,0,100,100,0,0,1,2,2,7,0,0,0,1",
        f"Style: TX,{font_name},{font_size},&H00FFFFFF&,&H000000FF&,&H00000000&,&H64000000&,0,0,0,0,100,100,0,0,1,2,2,7,0,0,0,1",
        f"Style: RX,{font_name},{font_size},&H00FFFFFF&,&H000000FF&,&H00000000&,&H64000000&,0,0,0,0,100,100,0,0,1,2,2,7,0,0,0,1",
        f"Style: BG,{font_name},1,&H00FFFFFF&,&H000000FF&,&H00000000&,&H00000000&,0,0,0,0,100,100,0,0,1,0,0,7,0,0,0,1",
        "",
        "[Events]",
        "Format: Layer,Start,End,Style,Name,MarginL,MarginR,MarginV,Effect,Text",
    ]

    events: List[str] = []

    # Obstacle layer spanning the whole overlay duration
    if max_end > 0:
        st0 = ass_time(0.0)
        et0 = ass_time(max_end)
        # Semi-transparent black rectangle using vector drawing
        bg_ev = rf"{{\an7\pos({bg_x},{bg_y})\p1\bord0\shad0\1c&H000000&\1a&H80&}}m 0 0 l {rect_w} 0 l {rect_w} {rect_h} l 0 {rect_h}"
        events.append(f"Dialogue: 0,{st0},{et0},BG,,0,0,0,,{bg_ev}")

    # Date Time (UTC) line updated every second
    if max_end > 0:
        t_end = int(math.ceil(max_end))
        for sec in range(0, t_end + 1):
            st = float(sec)
            et = min(max_end, float(sec + 1))
            if et <= st:
                continue
            # Convert overlay time -> UTC timestamp (respecting --offset)
            utc_ts = video_start + dt.timedelta(seconds=(st - offset))
            label = utc_ts.strftime("%Y-%m-%d %H:%M:%S")
            txt = esc_ass(label)
            ev = rf"{{\an7\pos({x_left},{time_y})\c&H00FFFFFF&}}{txt}"
            events.append(f"Dialogue: 8,{ass_time(st)},{ass_time(et)},TIME,,0,0,0,,{ev}")

    # TX/RX segments (from log)
    for seg in segments:
        st = ass_time(seg.start)
        et = ass_time(seg.end)

        if seg.tx:
            txt = esc_ass(seg.tx)
            ev = rf"{{\an7\pos({x_left},{tx_y})\c{ASS_RED}}}{txt}"
            events.append(f"Dialogue: 10,{st},{et},TX,,0,0,0,,{ev}")

        for i, line in enumerate(seg.rx_lines):
            if not line:
                continue
            y = rx0_y + i * line_gap
            txt = esc_ass(line)
            ev = rf"{{\an7\pos({x_left},{y})\c{ASS_GREEN}}}{txt}"
            events.append(f"Dialogue: 9,{st},{et},RX,,0,0,0,,{ev}")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(header) + "\n")
        f.write("\n".join(events) + "\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mov", default=None, help="Input .MOV (reads resolution + creation_time via ffprobe)")
    ap.add_argument("--log", required=True, help="Input log file (T/R lines)")
    ap.add_argument("--out", default="overlay.ass", help="Output .ass path")
    ap.add_argument("--res", default=None, help="Fallback resolution WxH (e.g. 1920x1080)")
    ap.add_argument("--video_start_utc", default=None, help='UTC start like "20260103 183121" (overrides MOV creation_time if provided)')
    ap.add_argument("--offset", type=float, default=0.0, help="Seconds to shift overlay times (+ later, - earlier)")
    ap.add_argument("--margin", type=int, default=40, help="Top/right margin in pixels")
    ap.add_argument("--box_width", type=int, default=520, help="Overlay box width in pixels (left aligned inside)")
    ap.add_argument("--font", default="DejaVu Sans Mono", help="ASS font name (monospace recommended, e.g. DejaVu Sans Mono, Consolas)")
    ap.add_argument("--font_size", type=int, default=None, help="Override font size (points). If omitted, auto-capped to fit max_chars inside box_width.")
    ap.add_argument("--max_chars", type=int, default=DEFAULT_MAX_CHARS, help="Max characters per line before truncation")
    args = ap.parse_args()

    entries = parse_log(args.log)
    if not entries:
        raise SystemExit("No valid log lines parsed.")

    width = height = 0
    video_start: Optional[dt.datetime] = None

    if args.mov:
        if not os.path.exists(args.mov):
            raise SystemExit(f"MOV not found: {args.mov}")
        width, height, rotate, ct = get_mov_info(args.mov)
        video_start = ct
        print(f"MOV: {width}x{height} rotate={rotate} creation_time_utc={video_start}")

    if width == 0 or height == 0:
        if not args.res:
            raise SystemExit("No MOV or ffprobe unavailable. Provide --res WxH.")
        width, height = parse_res(args.res)

    # If user provided an explicit start time, always use it (even if MOV has creation_time).
    if args.video_start_utc:
        try:
            video_start = dt.datetime.strptime(args.video_start_utc, "%Y%m%d %H%M%S")
        except ValueError:
            raise SystemExit('Bad --video_start_utc. Use "YYYYMMDD HHMMSS" (UTC).')


    # If you provide --video_start_utc, it ALWAYS overrides MOV creation_time
    # (useful when the camera clock is wrong or you trimmed/concatenated videos).
    if args.video_start_utc:
        video_start = dt.datetime.strptime(args.video_start_utc, "%Y%m%d %H%M%S")

    if video_start is None:
        raise SystemExit('No creation_time in MOV. Provide --video_start_utc "YYYYMMDD HHMMSS".')

    # Filter out earlier sessions (your Jan-2 line issue)
    entries = [e for e in entries if e.ts >= video_start]
    if not entries:
        raise SystemExit("After filtering by video start time, no log entries remain.")

    segments = build_segments(entries, video_start=video_start, offset=args.offset, max_chars=args.max_chars)
    make_ass(args.out, width, height, segments, margin=args.margin, box_width=args.box_width, video_start=video_start, offset=args.offset, font_name=args.font, max_chars=args.max_chars, font_size_override=args.font_size)
    print(f"Wrote {args.out}  segments={len(segments)}")

if __name__ == "__main__":
    main()