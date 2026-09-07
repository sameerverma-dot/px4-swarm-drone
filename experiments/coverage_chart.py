#!/usr/bin/env python3
"""Time to clear one hectare: drones vs camera resolution."""
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

SURFACE='#fcfcfb'; INK='#0b0b0b'; INK_2='#52514e'; INK_MUTED='#8a8984'; GRID='#e6e5e1'
S1='#2a78d6'; S2='#eb6834'          # validated categorical slots 1, 2

HFOV=1.74; ASPECT=0.75; TARGET=0.5; MINPX=24; SIDELAP=0.30; SPEED=3.8

def max_alt(cap):  return (TARGET/MINPX*cap)/(2*math.tan(HFOV/2))
def lane(alt):     return 2*alt*math.tan(HFOV/2)*(1-SIDELAP)
def t_clear(x,y,alt,n,turn=2.3):
    L=lane(alt); lanes=max(2, math.ceil((y/n)/L)+1)
    return (lanes*x+(lanes-1)*L)/SPEED + (lanes-1)*turn

X=Y=100
drones=[1,2,3]
t640  = [t_clear(X,Y,max_alt(640), n)/60  for n in drones]
t1280 = [t_clear(X,Y,max_alt(1280),n)/60  for n in drones]

fig, ax = plt.subplots(figsize=(8.4,5.0), dpi=160)
fig.patch.set_facecolor(SURFACE); ax.set_facecolor(SURFACE)

w=0.34
xs=[i for i in range(len(drones))]
b1=ax.bar([i-w/2 for i in xs], t640,  w, color=S1, zorder=3)
b2=ax.bar([i+w/2 for i in xs], t1280, w, color=S2, zorder=3)

for bars, vals in ((b1,t640),(b2,t1280)):
    for b,v in zip(bars,vals):
        ax.annotate(f'{v:.1f}', (b.get_x()+b.get_width()/2, v),
                    textcoords='offset points', xytext=(0,4), ha='center',
                    color=INK_2, fontsize=9, zorder=5)

# The honest comparison: doubling resolution is worth about the same as
# adding a second drone (3.8 vs 3.5 min) - not more, not three drones' worth.
ax.annotate('', xy=(1-w/2, t640[1]), xytext=(0+w/2, t1280[0]),
            arrowprops=dict(arrowstyle='-', color=INK_MUTED, lw=1.0,
                            ls=(0,(4,3))), zorder=2)
ax.annotate('≈ same result — one config line, or a second aircraft',
            xy=(0.5, max(t640)*1.06), ha='center', va='center',
            color=INK_2, fontsize=9, zorder=6,
            bbox=dict(facecolor=SURFACE, edgecolor='none', pad=3))

ax.set_xticks(xs); ax.set_xticklabels([f'{n} drone{"s" if n>1 else ""}' for n in drones])
ax.set_ylabel('minutes to clear 1 hectare', color=INK_2, fontsize=10)
ax.set_ylim(0, max(t640)*1.22)
ax.grid(True, axis='y', color=GRID, lw=0.8, zorder=0); ax.set_axisbelow(True)
for s in ('top','right'): ax.spines[s].set_visible(False)
for s in ('left','bottom'): ax.spines[s].set_color(GRID)
ax.tick_params(colors=INK_2, labelsize=9.5, length=0)

ax.set_title('Doubling resolution is worth about one extra drone',
             color=INK, fontsize=14, pad=30, loc='left', fontweight='semibold')
ax.text(0, 1.045, 'Detection needs ~24 px on target, which caps altitude: 5.6 m at 640 px, '
        '11.2 m at 1280 px.\nDouble the altitude, double the lane spacing — 38 % less flight time '
        'for one drone.',
        transform=ax.transAxes, color=INK_2, fontsize=9, va='bottom', ha='left')

leg=ax.legend(handles=[
    Line2D([],[],color=S1,lw=8,label='640 px capture · max 5.6 m'),
    Line2D([],[],color=S2,lw=8,label='1280 px capture · max 11.2 m (imgsz raised to match)'),
], loc='upper left', bbox_to_anchor=(0,-0.13), ncol=1, frameon=False, fontsize=9)
for t in leg.get_texts(): t.set_color(INK_2)

fig.tight_layout()
fig.savefig('coverage.png', facecolor=SURFACE, bbox_inches='tight')
print("wrote coverage.png")
