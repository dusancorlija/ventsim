#!/usr/bin/env python3
"""Stacked-panel SVG of a ventsim run. Standard library only.

    ./build/ventsim > docs/scenario.csv
    python3 tools/plot.py docs/scenario.csv docs/scenario.svg [t0 t1]
"""
import csv
import sys

INK, MUTED, GRID, ACCENT, BLUE, GREEN = "#15171a", "#7a7e85", "#eceae6", "#e2531a", "#3d6fd8", "#2f9e6e"

EVENTS = [(30, "bronchospasm: R 10 → 20"), (50, "stiffer lung: C 50 → 25"),
          (64, "patient breathes"), (80, "circuit open 80–86 s")]


def main(src, dst, t0=None, t1=None):
    rows = list(csv.DictReader(open(src)))
    t0 = float(t0) if t0 else float(rows[0]["t"])
    t1 = float(t1) if t1 else float(rows[-1]["t"])
    rows = [r for r in rows if t0 <= float(r["t"]) <= t1]
    col = lambda k, f=1.0: [float(r[k]) * f for r in rows]
    t = col("t")

    panels = [
        ("Airway pressure (cmH₂O)", (0, 25), [(col("setpoint"), "#c9c6c0", 1.2, "set point"), (col("paw"), INK, 1.1, "measured")]),
        ("Flow (L/min)", (-80, 120), [(col("flow", 60), BLUE, 1.0, None)]),
        ("Lung volume above FRC (mL)", (0, 1000), [(col("volume", 1000), GREEN, 1.1, None)]),
        ("Resistance (cmH₂O·s/L)", (0, 25), [(col("r_true"), "#c9c6c0", 2.0, "true"), (col("r_est"), ACCENT, 1.4, "estimated")]),
        ("Compliance (mL/cmH₂O)", (0, 70), [(col("c_true", 1000), "#c9c6c0", 2.0, "true"), (col("c_est", 1000), ACCENT, 1.4, "estimated")]),
    ]
    W, L, R, top, ph, gap = 1000, 70, 20, 70, 110, 30
    H = top + len(panels) * (ph + gap) + 20
    sx = lambda x: L + (x - t0) / (t1 - t0) * (W - L - R)

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
           'font-family="Inter, -apple-system, Segoe UI, sans-serif" font-size="11">',
           f'<rect width="{W}" height="{H}" fill="#fff"/>',
           f'<text x="{L}" y="24" font-size="15" font-weight="600" fill="{INK}">Pressure-controlled ventilation, closed loop at 1 kHz</text>',
           f'<text x="{L}" y="42" fill="{MUTED}">Rate 15/min, Pinsp 15 + PEEP 5 cmH₂O, Ti 1 s. Online R/C estimation by recursive least squares.</text>']

    # disconnect alarm shading across all panels
    alarm = [int(r["alarms"]) & 4 for r in rows]
    spans, start = [], None
    for ti, a in zip(t, alarm):
        if a and start is None: start = ti
        if not a and start is not None: spans.append((start, ti)); start = None
    if start is not None: spans.append((start, t[-1]))
    for a, b in spans:
        out.append(f'<rect x="{sx(a):.1f}" y="{top - 8}" width="{sx(b) - sx(a):.1f}" height="{H - top - 12}" fill="#fdecea"/>')
        out.append(f'<text x="{sx(a) + 4:.1f}" y="{H - 24}" fill="#c0392b" font-weight="600">DISCONNECT alarm</text>')

    for ev_t, label in EVENTS:
        if t0 < ev_t < t1:
            x = sx(ev_t)
            out.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{top - 14}" y2="{H - 20}" stroke="{MUTED}" stroke-dasharray="3 3" stroke-width="0.8"/>')
            out.append(f'<text x="{x + 4:.1f}" y="{top - 16}" fill="{MUTED}">{label}</text>')

    for i, (title, (lo, hi), series) in enumerate(panels):
        y0 = top + i * (ph + gap) + 12
        sy = lambda v: y0 + ph - (v - lo) / (hi - lo) * ph
        out.append(f'<clipPath id="p{i}"><rect x="{L}" y="{y0}" width="{W - L - R}" height="{ph}"/></clipPath>')
        out.append(f'<text x="{L}" y="{y0 - 4}" fill="{INK}" font-weight="600">{title}</text>')
        for frac in (0, 0.5, 1):
            v = lo + frac * (hi - lo)
            out.append(f'<line x1="{L}" x2="{W - R}" y1="{sy(v):.1f}" y2="{sy(v):.1f}" stroke="{GRID}"/>')
            out.append(f'<text x="{L - 8}" y="{sy(v) + 4:.1f}" text-anchor="end" fill="{MUTED}">{v:.0f}</text>')
        lx = W - R - 10
        for s, color, width, label in series:
            # thin the polyline to ~2 points per pixel
            step = max(1, len(t) // ((W - L - R) * 2))
            pts = " ".join(f"{sx(t[k]):.1f},{sy(s[k]):.1f}" for k in range(0, len(t), step))
            out.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" clip-path="url(#p{i})"/>')
        for s, color, width, label in reversed(series):
            if label:
                out.append(f'<text x="{lx}" y="{y0 - 4}" text-anchor="end" fill="{color if color != "#c9c6c0" else MUTED}">{label}</text>')
                lx -= 8 + len(label) * 6
    for tick in range(int(t0) // 10 * 10, int(t1) + 1, 10):
        if tick >= t0:
            out.append(f'<text x="{sx(tick):.1f}" y="{H - 6}" text-anchor="middle" fill="{MUTED}">{tick} s</text>')
    out.append("</svg>")
    open(dst, "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main(*sys.argv[1:])
