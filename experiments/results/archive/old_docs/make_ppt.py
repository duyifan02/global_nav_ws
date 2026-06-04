#!/usr/bin/env python3
"""Generate progress report PPT for TopoNav ablation & baseline experiments."""

from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.chart import XL_CHART_TYPE, XL_LEGEND_POSITION
from pptx.chart.data import CategoryChartData
import os

WHITE = RGBColor(0xFF, 0xFF, 0xFF)
BLACK = RGBColor(0x00, 0x00, 0x00)
DARK = RGBColor(0x1A, 0x1A, 0x2E)
ACCENT = RGBColor(0x00, 0x7A, 0xCC)
ACCENT2 = RGBColor(0xE8, 0x4D, 0x3D)
ACCENT3 = RGBColor(0x2E, 0xCC, 0x71)
GRAY = RGBColor(0x66, 0x66, 0x66)
TABLEBG = RGBColor(0xF8, 0xF9, 0xFA)
HEADERBG = RGBColor(0x00, 0x52, 0x8A)
HIGHLIGHT_ROW = RGBColor(0xE6, 0xF3, 0xFF)

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)


def add_text(slide, left, top, width, height, text, size=18, bold=False,
             color=BLACK, align=PP_ALIGN.LEFT):
    txBox = slide.shapes.add_textbox(Inches(left), Inches(top),
                                      Inches(width), Inches(height))
    tf = txBox.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = text
    p.font.size = Pt(size)
    p.font.bold = bold
    p.font.color.rgb = color
    p.font.name = 'Calibri'
    p.alignment = align
    return txBox


def add_bullets(slide, left, top, width, height, items, size=14,
                color=BLACK, spacing=Pt(6)):
    txBox = slide.shapes.add_textbox(Inches(left), Inches(top),
                                      Inches(width), Inches(height))
    tf = txBox.text_frame
    tf.word_wrap = True
    for i, item in enumerate(items):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.text = item
        p.font.size = Pt(size)
        p.font.color.rgb = color
        p.font.name = 'Calibri'
        p.space_after = spacing
    return txBox


def add_table(slide, left, top, width, height, data, col_widths=None,
              highlight_rows=None):
    rows, cols = len(data), len(data[0])
    ts = slide.shapes.add_table(rows, cols, Inches(left), Inches(top),
                                 Inches(width), Inches(height))
    table = ts.table
    if col_widths:
        for i, w in enumerate(col_widths):
            table.columns[i].width = Inches(w)
    for r in range(rows):
        for c in range(cols):
            cell = table.cell(r, c)
            cell.text = str(data[r][c])
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            for p in cell.text_frame.paragraphs:
                p.font.size = Pt(11)
                p.font.name = 'Calibri'
                if r == 0:
                    p.font.bold = True
                    p.font.color.rgb = WHITE
                    p.alignment = PP_ALIGN.CENTER
                else:
                    p.font.color.rgb = BLACK
                    p.alignment = PP_ALIGN.CENTER if c > 0 else PP_ALIGN.LEFT
            if r == 0:
                cell.fill.solid()
                cell.fill.fore_color.rgb = HEADERBG
            elif highlight_rows and r in highlight_rows:
                cell.fill.solid()
                cell.fill.fore_color.rgb = HIGHLIGHT_ROW
            else:
                cell.fill.solid()
                cell.fill.fore_color.rgb = WHITE if r % 2 == 1 else TABLEBG
    return ts


def title_bar(slide, title, subtitle=None):
    shape = slide.shapes.add_shape(1, Inches(0), Inches(0),
                                    prs.slide_width, Inches(1.2))
    shape.fill.solid()
    shape.fill.fore_color.rgb = DARK
    shape.line.fill.background()
    add_text(slide, 0.5, 0.15, 12, 0.6, title, size=28, bold=True, color=WHITE)
    if subtitle:
        add_text(slide, 0.5, 0.7, 12, 0.4, subtitle, size=14,
                 color=RGBColor(0xAA, 0xBB, 0xDD))


def add_chart(slide, left, top, width, height, categories, series_dict, title=''):
    chart_data = CategoryChartData()
    chart_data.categories = categories
    for name, values in series_dict.items():
        chart_data.add_series(name, values)
    cs = slide.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED,
                                 Inches(left), Inches(top),
                                 Inches(width), Inches(height), chart_data)
    chart = cs.chart
    chart.has_legend = True
    chart.legend.position = XL_LEGEND_POSITION.BOTTOM
    chart.legend.include_in_layout = False
    chart.legend.font.size = Pt(10)
    if title:
        chart.has_title = True
        chart.chart_title.text_frame.paragraphs[0].text = title
        chart.chart_title.text_frame.paragraphs[0].font.size = Pt(13)
    chart.plots[0].gap_width = 80
    return cs


# ═══════════════════════════════════════════════════
# SLIDE 1: Title
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
sl.background.fill.solid()
sl.background.fill.fore_color.rgb = DARK
add_text(sl, 1, 1.5, 11, 1.2, 'Stitched-Geodesic Planner',
         size=40, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
add_text(sl, 1, 2.7, 11, 0.8,
         'Cross-Resolution Ablation, Baseline Comparison & Tau Sensitivity',
         size=22, color=RGBColor(0xAA, 0xCC, 0xFF), align=PP_ALIGN.CENTER)
add_text(sl, 1, 4.0, 11, 0.5,
         '6 Matterport3D Scenes  |  0.1m / 0.075m / 0.05m  |  7 Configurations',
         size=16, color=RGBColor(0x88, 0x99, 0xBB), align=PP_ALIGN.CENTER)
add_text(sl, 1, 4.8, 11, 0.5,
         'feat/stitched-geodesic-planner  |  2026-05-14',
         size=14, color=RGBColor(0x66, 0x77, 0x99), align=PP_ALIGN.CENTER)

# ═══════════════════════════════════════════════════
# SLIDE 2: Experiment Overview
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Experiment Overview', '7 Configurations x 3 Resolutions')
data = [
    ['Config', 'Label', 'Description', 'Precompute'],
    ['B', 'flat_surface', 'Unrestricted voxel A* (no hierarchy)', 'None'],
    ['C', 'grid_partition', 'Uniform grid rooms + HPA* stitch', 'Grid seg + Dijkstra'],
    ['D', 'no_stitch', 'PH rooms + online voxel A* per room', 'PH only'],
    ['E', 'full (ours)', 'PH rooms + HPA* + stitchPath', 'PH + Dijkstra'],
    ['--', 'PRM', '1000-sample roadmap on surface graph', 'Roadmap build'],
    ['--', 'ALT A*', '16-landmark triangle-inequality A*', '16 Dijkstras'],
]
add_table(sl, 0.5, 1.5, 12.3, 3.2, data,
          col_widths=[0.8, 1.5, 5.5, 2.5], highlight_rows=[4])
add_bullets(sl, 0.5, 5.0, 12, 2.0, [
    'Ablation (B/C/D/E): isolate PH segmentation, stitch precompute, room hierarchy',
    'Baselines (PRM/ALT): established graph-search algorithms on surface graphs',
    'Same (start, goal) pairs, deterministic .npy occmaps across all configs',
    'S2 (1pXnuDYAj8r) excluded at 0.075m/0.05m — insufficient walkable voxels at fine resolution',
], size=13, color=GRAY)

# ═══════════════════════════════════════════════════
# SLIDE 3: THE MAIN RESULT — Cross-Resolution Scaling
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Main Result: Cross-Resolution Scaling',
          'Our method dominates at fine resolution')

# Scaling chart
res_labels = ['0.10m\n(|V|~10K)', '0.075m\n(|V|~25K)', '0.05m\n(|V|~80K)']
add_chart(sl, 0.3, 1.4, 6.5, 4.0, res_labels, {
    'Surface A*': [4.16, 8.83, 26.01],
    'flat_surface': [1.44, 2.91, 6.15],
    'full (ours)': [0.73, 0.89, 0.47],
    'ALT A*': [0.43, 0.70, 1.63],
}, title='Mean Query Time vs Resolution (ms)')

# Scaling table
data = [
    ['Resolution', '|V| (avg)', 'Surface A*', 'full (ours)', 'ALT A*', 'Ours vs ALT'],
    ['0.10m', '~10K', '4.16 ms', '0.73 ms (tau=8)', '0.43 ms', 'ALT 1.7x faster'],
    ['0.075m', '~25K', '8.83 ms', '0.89 ms (tau=8)', '0.70 ms', '~tie'],
    ['0.05m', '~80K', '26.01 ms', '0.47 ms (tau=20)', '1.63 ms', 'Ours 3.5x faster'],
]
add_table(sl, 7.0, 1.5, 6.0, 2.5, data,
          col_widths=[1.0, 0.7, 1.0, 1.3, 0.8, 1.2], highlight_rows=[3])

add_bullets(sl, 7.0, 4.2, 6.0, 3.0, [
    'ALT query grows ~3.8x (0.1m->0.05m): still searches graph',
    'Ours shrinks from 0.73 to 0.47ms with adaptive tau',
    'Crossover at ~0.065m resolution',
    'At 0.05m: stitch = O(1) lookup, zero voxel search',
    'Advantage grows with finer resolution (0.03m: est. >5x)',
], size=14, color=BLACK)

# ═══════════════════════════════════════════════════
# SLIDE 4: Per-Scene at 0.05m — The Money Slide
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Per-Scene at 0.05m: full (tau=20) vs ALT A*',
          'Zero voxel search on 3/5 scenes')

data = [
    ['Scene', 'full (tau=20)', 'ALT A*', 'Speedup', 'full nodes', 'ALT nodes'],
    ['S1', '0.060 ms', '0.972 ms', '16x', '0', '7,282'],
    ['S3', '1.387 ms', '0.674 ms', '0.5x (ALT)', '0', '5,491'],
    ['S4', '0.125 ms', '3.408 ms', '27x', '0', '27,499'],
    ['S5', '0.650 ms', '0.300 ms', '0.5x (ALT)', '8,153', '3,179'],
    ['S6', '0.120 ms', '2.776 ms', '23x', '0', '22,263'],
    ['Mean', '0.468 ms', '1.626 ms', '3.5x', '--', '--'],
]
add_table(sl, 0.5, 1.4, 8.0, 3.8, data,
          col_widths=[0.8, 1.3, 1.3, 1.2, 1.0, 1.0],
          highlight_rows=[1, 3, 5, 6])

# Per-scene chart
add_chart(sl, 8.8, 1.4, 4.2, 3.5,
          ['S1', 'S3', 'S4', 'S5', 'S6'],
          {
              'full (tau=20)': [0.060, 1.387, 0.125, 0.650, 0.120],
              'ALT A*': [0.972, 0.674, 3.408, 0.300, 2.776],
          }, title='Query Time at 0.05m (ms)')

add_bullets(sl, 0.5, 5.5, 12, 1.8, [
    'S1/S4/S6: stitch works perfectly (nodes=0) -> 16-27x faster than ALT',
    'S3: room A* overhead (tau=20 reduces 1700 rooms to ~200, but still 1.4ms)',
    'S5: fallback to corridor A* (stitch coverage gap, nodes=8153)',
    'Key: when stitch covers the query, query time is independent of graph size',
], size=14, color=GRAY)

# ═══════════════════════════════════════════════════
# SLIDE 5: Ablation Analysis (0.1m)
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Ablation Analysis (0.1m, tau=8)',
          'What does each component contribute?')

data = [
    ['Comparison', 'Speed Effect', 'Path Quality Effect', 'Key Insight'],
    ['flat -> full\n(add hierarchy)',
     '1.44 -> 0.73 ms\n(2.0x faster)', '+17.6% overhead',
     'Hierarchy trades path\nquality for speed'],
    ['no_stitch -> full\n(add stitch precompute)',
     '1.41 -> 0.73 ms\n(1.9x faster)', '+18.2% overhead',
     'Stitch gives O(1) query\nbut rigid Dijkstra trees'],
    ['grid -> full\n(PH vs uniform grid)',
     '0.13 -> 0.73 ms\n(5.7x slower)', 'Similar\n(15.8% vs 15.1%)',
     'PH captures topology\n(S2: 47% vs 3% overhead)'],
    ['no_stitch vs flat\n(PH segmentation alone)',
     '1.41 vs 1.44 ms\n(equivalent)', 'Both near-optimal\n(-0.9% vs -0.3%)',
     'PH segmentation alone\ndoes not degrade paths'],
]
add_table(sl, 0.3, 1.4, 12.7, 4.0, data,
          col_widths=[2.5, 2.3, 2.0, 3.0])

add_bullets(sl, 0.5, 5.7, 12, 1.5, [
    'Path overhead comes from room-level A* routing, NOT from stitch mechanism',
    'Stitch vs corridor A* within same config: delta_path = 0% (verified)',
    'PH segmentation captures corridors/rooms better than uniform grid — critical for S2',
], size=14, color=GRAY)

# ═══════════════════════════════════════════════════
# SLIDE 6: Path Quality Trade-off
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Path Quality: The Trade-off',
          'Speed vs path optimality')

data = [
    ['Method', '0.1m Ts', '0.1m Path', '0.05m Ts', '0.05m Path', 'Overall'],
    ['flat_surface', '1.44 ms', '34.09 m (-0.3%)', '6.15 ms', '34.85 m (ref)', 'Near-optimal, slow'],
    ['full (ours)', '0.73 ms', '40.08 m (+15.1%)', '0.47 ms', '42.15 m (+20%)', 'Fast, +20% overhead'],
    ['ALT A*', '0.43 ms', '34.39 m (+0.6%)', '1.63 ms', '34.54 m (+0.6%)', 'Near-optimal, scales'],
    ['grid_partition', '0.13 ms', '40.45 m (+15.8%)', '0.72 ms', '44.38 m (+27%)', 'Fastest, worst paths'],
    ['PRM (1000)', '0.53 ms', '47.37 m (+35.3%)', '3.34 ms', '54.56 m (+57%)', 'Poor paths overall'],
]
add_table(sl, 0.3, 1.4, 12.7, 3.5, data,
          col_widths=[1.4, 1.1, 2.0, 1.1, 2.0, 2.0],
          highlight_rows=[2, 3])

add_bullets(sl, 0.5, 5.2, 12, 2.0, [
    'Our method: 3.5x speed advantage at 0.05m, with ~20% path overhead trade-off',
    'Path overhead is from room-level routing (Euclidean heuristic between room centroids)',
    'ALT achieves near-optimal paths (+0.6%) because it still searches the original graph',
    'PRM worst overall: sparse sampling causes S2 path 2x optimal',
    'Trade-off is tunable via tau: higher tau = fewer rooms = faster but potentially worse paths',
], size=14, color=GRAY)

# ═══════════════════════════════════════════════════
# SLIDE 7: Tau Sensitivity & Resolution-Adaptive Tau
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Tau Sensitivity & Resolution-Adaptive Tau',
          'Key finding: tau must scale with resolution')

# Tau at 0.05m comparison
data = [
    ['Tau', 'Mean Ts', 'S3 Ts', 'Mean Path', 'S3 Rooms (est.)', 'Status'],
    ['8', '1.670 ms', '7.210 ms', '40.46 m', '~1700', 'S3 bottleneck'],
    ['20', '0.468 ms', '1.387 ms', '42.15 m', '~200', 'Best balance'],
    ['40', '0.530 ms', '1.650 ms', '40.52 m', '~50', 'Diminishing returns'],
]
add_table(sl, 0.5, 1.5, 6.0, 2.5, data,
          col_widths=[0.5, 1.0, 1.0, 1.0, 1.2, 1.3], highlight_rows=[2])

# Room count chart from tau sweep (0.1m)
taus = ['1', '2', '4', '6', '8', '10']
add_chart(sl, 6.8, 1.4, 6.0, 3.0, taus, {
    'S1': [128, 93, 56, 38, 20, 12],
    'S3': [288, 180, 90, 44, 17, 12],
    'S4': [180, 121, 60, 34, 18, 10],
    'S6': [172, 119, 68, 34, 16, 11],
}, title='Room Count vs Tau (0.1m)')

# Recommended tau table
data2 = [
    ['Resolution', 'Recommended Tau', 'Rationale'],
    ['0.10m', 'tau = 8', 'Good balance, 17-127 rooms'],
    ['0.075m', 'tau = 8', 'Similar room counts'],
    ['0.05m', 'tau = 20', 'Prevents room explosion'],
    ['0.03m', 'tau ~ 40', 'Estimated, needs validation'],
]
add_table(sl, 0.5, 4.3, 5.5, 2.5, data2,
          col_widths=[1.2, 1.5, 2.5], highlight_rows=[3])

add_bullets(sl, 6.5, 4.5, 6.3, 2.5, [
    'Finer resolution reveals more topological features',
    'tau=8 at 0.05m: S3 gets 1700 rooms -> roomAstar 7ms',
    'tau=20 fixes S3 (1.4ms) with minimal path quality loss',
    'Heuristic: tau_eff = tau_base * (0.1 / resolution)',
    'tau=1 pathological: S4 path = 89m (+116%)',
], size=13, color=GRAY)

# ═══════════════════════════════════════════════════
# SLIDE 8: Precompute & Amortization
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Precompute Cost & Amortization',
          'One-time cost, constant-time queries')

# Build time at 0.05m
data = [
    ['Method', 'S1', 'S3', 'S4', 'S5', 'S6', 'Mean'],
    ['full (tau=20)', '1.7s', '10.9s', '14.4s', '0.01s', '9.1s', '7.2s'],
    ['ALT A* (16L)', '2.3s', '12.2s', '19.1s', '0.05s', '12.2s', '9.2s'],
]
add_table(sl, 0.5, 1.5, 7.5, 1.5, data,
          col_widths=[1.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8])

add_text(sl, 0.5, 3.3, 7.5, 0.4, 'Amortized Cost Analysis (0.05m)',
         size=18, bold=True, color=ACCENT)

add_bullets(sl, 0.5, 3.8, 7.0, 3.0, [
    'Both methods: precompute 7-9 seconds at 0.05m (comparable)',
    'Our precompute slightly cheaper: O(R * k^2) vs O(L * |V|)',
    'Per-query savings: 1.16ms (1.63 - 0.47)',
    'Break-even: from query 1 (our precompute is also cheaper)',
    'At 100 queries: save 116ms total query time',
    'ALT memory: O(L * |V|) = 16 * 80K * 8B = 10MB distance tables',
    'Our memory: O(R * room_size) -- scales with rooms, not |V|',
], size=14, color=BLACK)

# Memory comparison
data2 = [
    ['Metric', 'full (ours)', 'ALT A*'],
    ['Precompute complexity', 'O(R * k^2)', 'O(L * |V| * log|V|)'],
    ['Memory', 'O(R * room_size)', 'O(L * |V|)'],
    ['Query complexity', 'O(path_len)', 'O(|V| * log|V|)'],
    ['Query scales with |V|?', 'No (constant)', 'Yes (sublinear)'],
]
add_table(sl, 8.3, 1.5, 4.7, 2.8, data2,
          col_widths=[1.8, 1.3, 1.6], highlight_rows=[4])

# ═══════════════════════════════════════════════════
# SLIDE 9: All Methods Comparison at 0.05m
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Full Comparison at 0.05m',
          'All methods, 5 scenes (S2 excluded)')

# Chart
add_chart(sl, 0.3, 1.4, 6.5, 5.5,
          ['S1', 'S3', 'S4', 'S5', 'S6'],
          {
              'Surface A*': [30.60, 14.41, 32.06, 21.34, 31.63],
              'flat_surface': [8.45, 3.30, 9.36, 0.71, 8.92],
              'full (tau=20)': [0.06, 1.39, 0.13, 0.65, 0.12],
              'ALT A*': [0.97, 0.67, 3.41, 0.30, 2.78],
              'grid_partition': [0.83, 0.51, 1.23, 0.10, 0.95],
          }, title='Query Time at 0.05m (ms) — All Methods')

# Summary table
data = [
    ['Method', 'Mean Ts', 'vs SurfA*', 'Mean Path', 'Path OH'],
    ['Surface A*', '26.01 ms', '1x', '34.85 m', 'ref'],
    ['flat_surface', '6.15 ms', '4.2x', '34.85 m', '-0.0%'],
    ['no_stitch', '5.47 ms', '4.8x', '34.63 m', '-0.6%'],
    ['grid_partition', '0.72 ms', '36x', '44.38 m', '+27%'],
    ['full (tau=20)', '0.47 ms', '55x', '42.15 m', '+20%'],
    ['PRM (1000)', '3.34 ms', '7.8x', '54.56 m', '+57%'],
    ['ALT A* (16L)', '1.63 ms', '16x', '34.54 m', '+0.6%'],
]
add_table(sl, 7.0, 1.5, 6.0, 4.2, data,
          col_widths=[1.4, 1.0, 0.8, 1.0, 0.8],
          highlight_rows=[5])

add_bullets(sl, 7.0, 5.9, 6.0, 1.3, [
    'Our method: fastest (0.47ms, 55x vs Surface A*)',
    'ALT: best path quality, 16x speedup',
    'PRM: poor on all metrics',
], size=13, color=GRAY)

# ═══════════════════════════════════════════════════
# SLIDE 10: Summary & Next Steps
# ═══════════════════════════════════════════════════
sl = prs.slides.add_slide(prs.slide_layouts[6])
title_bar(sl, 'Summary & Next Steps')

add_text(sl, 0.5, 1.5, 6.0, 0.5, 'Key Findings', size=20, bold=True, color=ACCENT)
add_bullets(sl, 0.5, 2.1, 6.0, 3.0, [
    'At 0.05m: 3.5x faster than ALT A* (strongest baseline)',
    'Crossover at ~0.065m — advantage grows with resolution',
    'Stitch gives O(1) query (nodes=0) on 3/5 scenes at 0.05m',
    'Resolution-adaptive tau is essential (tau=20 at 0.05m)',
    'Path overhead ~20% from room-level routing (not stitch)',
    'PH segmentation better than uniform grid (S2: 47% vs 3%)',
], size=14, color=BLACK)

add_text(sl, 6.8, 1.5, 6.0, 0.5, 'Completed Experiments', size=20, bold=True, color=ACCENT3)
add_bullets(sl, 6.8, 2.1, 6.0, 2.5, [
    '7 configs x 3 resolutions (0.1/0.075/0.05m)',
    'Ablation: flat_surface / grid_partition / no_stitch / full',
    'Baselines: PRM (1000), ALT A* (16 landmarks)',
    'Tau sensitivity: 8 values x 6 scenes (0.1m)',
    'Tau tuning: tau=20/40 at 0.05m, tau=20 at 0.075m',
], size=14, color=BLACK)

add_text(sl, 6.8, 4.8, 6.0, 0.5, 'Possible Next Steps', size=20, bold=True, color=ACCENT2)
add_bullets(sl, 6.8, 5.3, 6.0, 2.0, [
    'Improve room-level routing to reduce path overhead',
    'Multi-query amortization analysis (N=1..200)',
    'Run 0.03m to confirm scaling trend extrapolation',
    'Path visualization for qualitative comparison',
    'Generate LaTeX tables for paper submission',
], size=14, color=BLACK)

# ── Save ──
outpath = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'TopoNav_Progress_Report_20260514.pptx')
prs.save(outpath)
print(f'Saved: {outpath}')
