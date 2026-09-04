#!/usr/bin/env python3
"""
hazard_map.py - turn a survey run into an actual deliverable.

Takes the two CSVs a mission produces and renders:

  * <out>.png      the map: survey area, flown track, geotagged hazards
  * <out>.geojson  the same hazards as WGS84 points, openable in QGIS /
                   Google Earth / geojson.io

A CSV is evidence. A map is the deliverable - this is the artefact that goes in
the report, the slide deck and the demo.

Usage (defaults pick the NEWEST track in ~/maps):

    ros2 run perception hazard_map
    python3 hazard_map.py --area 0,30,0,20 --truth 10,15
    python3 hazard_map.py --track ~/maps/survey_track_1788287098.csv \
                          --hazards ~/maps/hazard_points.csv \
                          --out ~/maps/hazard_map

Coordinates are PX4 local NED metres with home at (0,0): +X = North, +Y = East.
The plot puts East on the horizontal axis and North on the vertical, which is
how a map is normally read - so it is NOT a raw x-vs-y plot of the CSV columns.
"""

import argparse
import csv
import glob
import json
import math
import os
import sys

# --- palette (validated: all-pairs CVD dE 9.2, normal-vision 24.0, light mode) ---
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
INK_MUTED = '#8a8984'
GRID = '#e6e5e1'
TRACK = '#9a9c9f'
# Categorical slots 1-3. Scatter is an all-pairs form, so THREE is the cap -
# a 4th slot puts yellow next to orange and fails the separation floor.
# Anything past 3 classes folds into "Other" rather than inventing a hue.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']
OTHER = '#52514e'

LABEL_BG = dict(facecolor=SURFACE, edgecolor='none', pad=1.4)

EARTH_R = 6378137.0
DEFAULT_HOME = (23.2127, 72.6846)      # IIT Gandhinagar, matches start_px4_sim.sh


def newest_track(maps_dir):
    hits = sorted(glob.glob(os.path.join(maps_dir, 'survey_track_*.csv')))
    return hits[-1] if hits else None


def read_track(path):
    """-> list of (t, north, east, alt_m). Skips the header and short rows."""
    out = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                out.append((float(r['t_s']), float(r['x_ned']),
                            float(r['y_ned']), -float(r['z_ned'])))
            except (KeyError, TypeError, ValueError):
                continue
    return out


def read_hazards(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                out.append({
                    't': float(r['t_s']),
                    'north': float(r['x_ned_north']),
                    'east': float(r['y_ned_east']),
                    'cls': r['class'],
                    'conf': float(r['conf']),
                    'alt': float(r['alt_m']),
                })
            except (KeyError, TypeError, ValueError):
                continue
    return out


def ned_to_wgs84(north, east, home_lat, home_lon):
    """Flat-earth local NED -> lat/lon. Fine over a survey-sized area."""
    lat = home_lat + math.degrees(north / EARTH_R)
    lon = home_lon + math.degrees(east / (EARTH_R * math.cos(math.radians(home_lat))))
    return lat, lon


def write_geojson(hazards, path, home_lat, home_lon):
    feats = []
    for i, h in enumerate(hazards, 1):
        lat, lon = ned_to_wgs84(h['north'], h['east'], home_lat, home_lon)
        feats.append({
            'type': 'Feature',
            'geometry': {'type': 'Point', 'coordinates': [round(lon, 8), round(lat, 8)]},
            'properties': {
                'id': i, 'class': h['cls'], 'confidence': h['conf'],
                'north_m': round(h['north'], 2), 'east_m': round(h['east'], 2),
                'detected_at_alt_m': h['alt'],
            },
        })
    with open(path, 'w') as f:
        json.dump({'type': 'FeatureCollection',
                   'properties': {'home_lat': home_lat, 'home_lon': home_lon,
                                  'frame': 'PX4 local NED, home at origin'},
                   'features': feats}, f, indent=2)


def parse_pair(s, name):
    try:
        a, b = (float(v) for v in s.split(','))
        return a, b
    except Exception:
        sys.exit(f"--{name} expects two comma-separated numbers, got {s!r}")


def main(argv=None):
    maps_dir = os.path.expanduser('~/maps')
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--track', default=None, help='survey_track_*.csv (default: newest)')
    ap.add_argument('--hazards', default=os.path.join(maps_dir, 'hazard_points.csv'))
    ap.add_argument('--out', default=None, help='output path WITHOUT extension')
    ap.add_argument('--area', default=None,
                    help='survey rectangle x_min,x_max,y_min,y_max (NED metres)')
    ap.add_argument('--truth', default=None,
                    help='known target north,east - drawn as a validation ring')
    ap.add_argument('--home', default=None,
                    help='home lat,lon for the GeoJSON (default IIT Gandhinagar)')
    ap.add_argument('--title', default='Autonomous Survey - Hazard Map')
    # Post-hoc filters: let an OLD hazard CSV be re-read under the rules the
    # pipeline now applies, without re-flying the mission.
    ap.add_argument('--classes', default=None,
                    help="comma-separated class names to keep, e.g. 'person'")
    ap.add_argument('--min-conf', type=float, default=None,
                    help='drop detections below this confidence')
    ap.add_argument('--max-alt', type=float, default=None,
                    help='drop detections recorded above this altitude (m)')
    # `ros2 run` can append --ros-args; don't let that kill the tool.
    a, unknown = ap.parse_known_args(argv)
    if unknown:
        print(f"[hazard_map] ignoring unrecognised args: {' '.join(unknown)}")

    track_path = a.track or newest_track(maps_dir)
    if not track_path or not os.path.exists(os.path.expanduser(track_path)):
        sys.exit(f"no track CSV found (looked in {maps_dir}). Fly a mission first, "
                 f"or pass --track.")
    track_path = os.path.expanduser(track_path)
    haz_path = os.path.expanduser(a.hazards)

    track = read_track(track_path)
    hazards = read_hazards(haz_path)
    if not track:
        sys.exit(f"track CSV has no usable rows: {track_path}")

    n_raw = len(hazards)
    if a.classes:
        keep = {c.strip() for c in a.classes.split(',') if c.strip()}
        hazards = [h for h in hazards if h['cls'] in keep]
    if a.min_conf is not None:
        hazards = [h for h in hazards if h['conf'] >= a.min_conf]
    if a.max_alt is not None:
        hazards = [h for h in hazards if h['alt'] <= a.max_alt]
    n_filtered = n_raw - len(hazards)

    out = os.path.expanduser(a.out) if a.out else os.path.join(
        maps_dir, os.path.basename(track_path).replace('survey_track_', 'hazard_map_')
                                              .replace('.csv', ''))
    home_lat, home_lon = parse_pair(a.home, 'home') if a.home else DEFAULT_HOME

    # ---------------- figure ----------------
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    fig, ax = plt.subplots(figsize=(9.5, 8.0), dpi=160)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    # survey area - recessive, it is context not data
    if a.area:
        try:
            x0, x1, y0, y1 = (float(v) for v in a.area.split(','))
        except Exception:
            sys.exit(f"--area expects x_min,x_max,y_min,y_max, got {a.area!r}")
        ax.add_patch(plt.Rectangle((y0, x0), y1 - y0, x1 - x0, fill=False,
                                   ec=INK_MUTED, lw=1.0, ls=(0, (5, 4)), zorder=1))
        ax.annotate('survey area', (y0, x1), textcoords='offset points',
                    xytext=(2, 6), color=INK_MUTED, fontsize=8.5,
                    va='bottom', ha='left', zorder=8, bbox=LABEL_BG)

    # flown track - 2px line, recessive. East horizontal, North vertical.
    te = [p[2] for p in track]
    tn = [p[1] for p in track]
    ax.plot(te, tn, color=TRACK, lw=1.4, zorder=2, solid_capstyle='round')

    # home
    ax.plot([0], [0], marker='s', ms=7, color=INK_2, zorder=5,
            mec=SURFACE, mew=1.5)
    ax.annotate('home', (0, 0), textcoords='offset points', xytext=(10, -14),
                color=INK_2, fontsize=8.5, zorder=8, bbox=LABEL_BG)

    # hazards, coloured by class; >3 classes fold into "Other"
    by_cls = {}
    for h in hazards:
        by_cls.setdefault(h['cls'], []).append(h)
    ranked = sorted(by_cls, key=lambda c: -len(by_cls[c]))
    colors = {c: SERIES[i] for i, c in enumerate(ranked[:3])}
    for c in ranked[3:]:
        colors[c] = OTHER

    for cls in ranked:
        pts = by_cls[cls]
        ax.scatter([p['east'] for p in pts], [p['north'] for p in pts],
                   s=95, c=colors[cls], edgecolors=SURFACE, linewidths=2.0,
                   zorder=6 if cls in ranked[:3] else 4, label=None)

    # known target - validation ring, deliberately NOT a colour slot
    if a.truth:
        tnn, tee = parse_pair(a.truth, 'truth')
        ax.plot([tee], [tnn], marker='o', ms=20, mfc='none', mec=INK,
                mew=1.6, zorder=7)
        ax.plot([tee], [tnn], marker='+', ms=11, color=INK, mew=1.6, zorder=7)
        ax.annotate('known target', (tee, tnn), textcoords='offset points',
                    xytext=(15, 8), color=INK, fontsize=9, zorder=8,
                    bbox=LABEL_BG)
        # nearest detection to truth - the calibration number, stated on the map
        if hazards:
            d, nearest = min((math.hypot(h['north'] - tnn, h['east'] - tee), h)
                             for h in hazards)
            ax.annotate(f"nearest: {nearest['cls']} {d:.1f} m",
                        (nearest['east'], nearest['north']),
                        textcoords='offset points', xytext=(12, -16),
                        color=INK_2, fontsize=8.5, zorder=7)

    # A map must not distort distance, but 'datalim' pads the axes out to huge
    # empty margins. Square the window on the data and keep aspect exact.
    xs = list(te) + [h['east'] for h in hazards] + [0.0]
    ys = list(tn) + [h['north'] for h in hazards] + [0.0]
    if a.area:
        xs += [y0, y1]
        ys += [x0, x1]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    span = max(max(xs) - min(xs), max(ys) - min(ys), 10.0) * 1.14
    ax.set_xlim(cx - span / 2, cx + span / 2)
    ax.set_ylim(cy - span / 2, cy + span / 2)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('East (m)', color=INK_2, fontsize=10)
    ax.set_ylabel('North (m)', color=INK_2, fontsize=10)
    ax.grid(True, color=GRID, lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=9, length=0)

    # north arrow - it is a map
    ax.annotate('N', xy=(0.965, 0.955), xytext=(0.965, 0.885),
                xycoords='axes fraction', textcoords='axes fraction',
                ha='center', va='bottom', color=INK_2, fontsize=10,
                arrowprops=dict(arrowstyle='-|>', color=INK_2, lw=1.2))

    # legend carries the counts, which doubles as the relief the palette needs
    handles = [Line2D([], [], color=TRACK, lw=1.4,
                      label=f'flown track ({len(track)} samples)')]
    for cls in ranked:
        handles.append(Line2D([], [], marker='o', ls='none', ms=9,
                              mfc=colors[cls], mec=SURFACE, mew=1.5,
                              label=f'{cls} ({len(by_cls[cls])})'))
    leg = ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.0, -0.09),
                    ncol=min(5, len(handles)), frameon=False, fontsize=9,
                    handletextpad=0.6, columnspacing=1.6)
    for t in leg.get_texts():
        t.set_color(INK_2)

    alts = [p[3] for p in track]
    filt = f" ({n_filtered} filtered out)" if n_filtered else ""
    sub = (f"{len(hazards)} detection{'' if len(hazards) == 1 else 's'}{filt} · "
           f"track {len(track)} samples · "
           f"altitude {max(0.0, min(alts)):.1f}-{max(alts):.1f} m · "
           f"frame: PX4 local NED, home at origin")
    ax.set_title(a.title, color=INK, fontsize=14, pad=18, loc='left',
                 fontweight='semibold')
    ax.text(0, 1.015, sub, transform=ax.transAxes, color=INK_2, fontsize=9,
            va='bottom', ha='left')

    fig.tight_layout()
    png = out + '.png'
    fig.savefig(png, facecolor=SURFACE, bbox_inches='tight')
    plt.close(fig)

    gj = out + '.geojson'
    write_geojson(hazards, gj, home_lat, home_lon)

    # the table view - the other half of the relief rule, and useful on its own
    print(f"\n  {'#':>3}  {'class':<10} {'conf':>5}  {'North':>8} {'East':>8}  {'alt':>6}")
    print(f"  {'-'*3}  {'-'*10} {'-'*5}  {'-'*8} {'-'*8}  {'-'*6}")
    for i, h in enumerate(hazards, 1):
        print(f"  {i:>3}  {h['cls']:<10} {h['conf']:>5.2f}  "
              f"{h['north']:>8.2f} {h['east']:>8.2f}  {h['alt']:>5.1f}m")
    if not hazards:
        print("   (none)")
    print(f"\n  map      -> {png}")
    print(f"  geojson  -> {gj}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
