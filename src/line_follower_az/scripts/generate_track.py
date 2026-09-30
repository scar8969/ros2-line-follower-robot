#!/usr/bin/env python3
"""generate_track.py — procedural stadium-track SDF generator (kept from the
original architecture).

Builds the <visual> elements for a stadium-shaped (rounded-rectangle) line
track: two straights joined by two semicircles. Run once, offline, to produce
the XML block that lives in worlds/line_track.world. Re-run after changing
STRAIGHT_LEN / TURN_RADIUS to reshape the track without hand-editing SDF.
"""

import math

STRAIGHT_LEN = 2.0      # length of each straight section (m)
TURN_RADIUS = 0.6       # radius of each semicircular turn (m)
LINE_WIDTH = 0.04       # width of the painted line (m)
LINE_THICKNESS = 0.002  # height of the line strip (avoids z-fighting)
SEGMENT_LEN = 0.08      # target length of each discretized straight segment (m)
ARC_STEP_DEG = 8        # angular step for curve discretization (degrees)

visuals = []
seg_id = 0


def add_straight_segment(x0, y0, x1, y1):
    global seg_id
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    visuals.append(make_visual(seg_id, cx, cy, yaw, length))
    seg_id += 1


def add_arc(cx, cy, radius, start_deg, end_deg):
    """Discretize a semicircular arc into short straight chords."""
    steps = max(1, int(abs(end_deg - start_deg) / ARC_STEP_DEG))
    angles = [start_deg + (end_deg - start_deg) * i / steps for i in range(steps + 1)]
    pts = [(cx + radius * math.cos(math.radians(a)),
            cy + radius * math.sin(math.radians(a))) for a in angles]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        add_straight_segment(x0, y0, x1, y1)


def make_visual(idx, x, y, yaw, length):
    return f"""        <visual name="line_segment_{idx}">
          <pose>{x:.4f} {y:.4f} 0.006 0 0 {yaw:.4f}</pose>
          <geometry>
            <box>
              <size>{length:.4f} {LINE_WIDTH} {LINE_THICKNESS}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>"""


def build_segments_for_straight(x0, y0, x1, y1):
    length = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(length / SEGMENT_LEN))
    for i in range(n):
        t0 = i / n
        t1 = (i + 1) / n
        sx0 = x0 + (x1 - x0) * t0
        sy0 = y0 + (y1 - y0) * t0
        sx1 = x0 + (x1 - x0) * t1
        sy1 = y0 + (y1 - y0) * t1
        add_straight_segment(sx0, sy0, sx1, sy1)


L = STRAIGHT_LEN
R = TURN_RADIUS


def build_track_visuals():
    """Build the list of SDF <visual> strings for the stadium track."""
    global visuals, seg_id
    visuals = []
    seg_id = 0
    # Top straight: left-to-right at y = +R
    build_segments_for_straight(-L / 2, R, L / 2, R)
    # Right semicircle: centered at (L/2, 0), from 90deg down to -90deg
    add_arc(L / 2, 0, R, 90, -90)
    # Bottom straight: right-to-left at y = -R
    build_segments_for_straight(L / 2, -R, -L / 2, -R)
    # Left semicircle: centered at (-L/2, 0), from -90deg (bottom) up to
    # -270deg (top) through the left side, closing the loop.
    add_arc(-L / 2, 0, R, -90, -270)
    return visuals


if __name__ == '__main__':
    build_track_visuals()
    print(f"<!-- Stadium track: straight={L}m, radius={R}m, {len(visuals)} visual segments -->")
    print('\n'.join(visuals))
    print(f"<!-- total path length approx {2*L + 2*math.pi*R:.2f} m -->")
