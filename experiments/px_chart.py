#!/usr/bin/env python3
"""Chart: YOLO confidence vs target pixel size, with the two real flights marked."""
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

SURFACE='#fcfcfb'; INK='#0b0b0b'; INK_2='#52514e'; INK_MUTED='#8a8984'; GRID='#e6e5e1'
S1='#2a78d6'; S2='#eb6834'          # validated categorical slots 1 and 2
HFOV=1.74; PERSON_M=0.5

# measured by px_sweep.py
px   = [6, 8, 10, 13, 16, 20, 24, 27, 32, 40, 50, 64, 80, 100]
conf = [0.01,0.03,0.03,0.12,0.37,0.19,0.74,0.76,0.75,0.84,0.90,0.87,0.91,0.91]

def alt_for(p, cap_w):
    return PERSON_M*cap_w/(2*math.tan(HFOV/2)*p)

fig, ax = plt.subplots(figsize=(9.2,5.4), dpi=160)
fig.patch.set_facecolor(SURFACE); ax.set_facecolor(SURFACE)

# the usable-detection threshold the mission launch file actually uses
ax.axhline(0.40, color=INK_MUTED, lw=1.0, ls=(0,(5,4)), zorder=1)
ax.annotate('mission conf threshold 0.40', (101, 0.40), textcoords='offset points',
            xytext=(-4,6), ha='right', color=INK_MUTED, fontsize=8.5, zorder=4,
            bbox=dict(facecolor=SURFACE, edgecolor='none', pad=1.4))

# the unstable band, stated rather than smoothed away
ax.axvspan(13, 24, color=GRID, zorder=0)
ax.annotate('unstable band', (17, 0.03), color=INK_MUTED, fontsize=8.5,
            ha='center', zorder=4)

ax.plot(px, conf, color=S1, lw=2.0, zorder=3, solid_capstyle='round')
ax.scatter(px, conf, s=34, color=S1, edgecolors=SURFACE, linewidths=1.6, zorder=4)

# the two flights
for p, c, label, note in [
    (27, 0.76, '1 Sep · 5 m', 'detected (0.89 in flight)'),
    (13, 0.12, '3 Sep · 10 m', 'ZERO detections'),
]:
    ax.scatter([p],[c], s=150, facecolors='none', edgecolors=S2, linewidths=2.2, zorder=6)
    ax.annotate(f'{label}\n{note}', (p,c), textcoords='offset points',
                xytext=(-14, 34) if p==13 else (14, -34), color=S2, fontsize=9,
                ha='right' if p==13 else 'left', zorder=7,
                bbox=dict(facecolor=SURFACE, edgecolor='none', pad=1.6))

ax.set_xscale('log')
ax.set_xticks([6,10,16,24,40,64,100])
ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
ax.set_xlim(5.5, 110); ax.set_ylim(-0.03, 1.0)
ax.set_xlabel('target width in the frame (pixels)', color=INK_2, fontsize=10)
ax.set_ylabel('best "person" confidence', color=INK_2, fontsize=10)
ax.grid(True, color=GRID, lw=0.8, zorder=0); ax.set_axisbelow(True)
for s in ('top','right'): ax.spines[s].set_visible(False)
for s in ('left','bottom'): ax.spines[s].set_color(GRID)
ax.tick_params(colors=INK_2, labelsize=9, length=0)

# second axis: what that pixel size means in altitude at 640 capture
sec = ax.secondary_xaxis('top', functions=(lambda p: p, lambda p: p))
sec.set_xticks([6,10,16,24,40,64,100])
sec.set_xticklabels([f"{alt_for(p,640):.0f}" for p in [6,10,16,24,40,64,100]])
sec.set_xlabel('flight altitude at 640-wide capture (m)', color=INK_2, fontsize=9.5)
sec.tick_params(colors=INK_2, labelsize=9, length=0)
sec.spines['top'].set_color(GRID)

ax.set_title('YOLO needs about 24 pixels of person', color=INK, fontsize=14,
             pad=52, loc='left', fontweight='semibold')
ax.text(0, 1.135, 'yolov8n · COCO weights · side-on crop on the sim ground plane — '
        'nadir views will be harder still',
        transform=ax.transAxes, color=INK_2, fontsize=9, va='bottom', ha='left')

leg = ax.legend(handles=[
    Line2D([],[], color=S1, lw=2.0, marker='o', ms=6, mec=SURFACE, mew=1.4,
           label='measured confidence'),
    Line2D([],[], color=S2, lw=0, marker='o', ms=10, mfc='none', mew=2.0,
           label='actual flights'),
], loc='upper left', bbox_to_anchor=(0.0,-0.16), ncol=2, frameon=False, fontsize=9)
for t in leg.get_texts(): t.set_color(INK_2)

fig.tight_layout()
fig.savefig('out/detection_vs_altitude.png', facecolor=SURFACE, bbox_inches='tight')
print("wrote out/detection_vs_altitude.png")
