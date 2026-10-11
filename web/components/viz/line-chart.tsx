'use client';
import { ResponsiveLine, type LineCustomSvgLayerProps } from '@nivo/line';
import { roleVar, type Role } from '@/lib/roles';

export interface Series {
  id: string;
  role: Role;
  data: { x: number; y: number | null }[];
  /** Dashed stroke. Ground truth is always dashed (design/TOC.md §2); so are bounds. */
  dashed?: boolean;
  /** Stroke width in px (default 1.75). */
  width?: number;
}

/** The shape Nivo is given: the plotted part of a `Series`. */
type Plotted = { id: string; data: { x: number; y: number | null }[] };

export interface RefLine {
  y: number;
  label?: string;
  role?: Role;
  /** Which end of the line carries the label (default right). */
  side?: 'left' | 'right';
}

/**
 * A line chart (Nivo) in the book's palette. Series are colored by role through CSS custom
 * properties, so the chart follows the light/dark theme with no JavaScript. Horizontal
 * reference lines (chi-square bounds, the expected value of a statistic) are dashed.
 *
 * `yLog` puts the y axis on a base-10 logarithmic scale (Chapter 2: a NEES that spans four
 * orders of magnitude); every plotted y must then be positive. Lines are drawn by a layer of
 * this file rather than by Nivo's own, because Nivo has no per-series dash. On a log axis a value
 * of 0 or less is drawn as a gap.
 */
export function LineChart({
  series,
  xLabel,
  yLabel,
  refLines = [],
  height = 240,
  yMin,
  yMax,
  yLog = false,
  yTicks,
  ariaLabel,
}: {
  series: Series[];
  xLabel: string;
  yLabel: string;
  refLines?: RefLine[];
  height?: number;
  yMin?: number | 'auto';
  yMax?: number | 'auto';
  yLog?: boolean;
  /** Explicit y tick values (a log axis wants decades, not every minor tick). */
  yTicks?: number[];
  ariaLabel: string;
}) {
  const colorOf = new Map(series.map((s) => [s.id, roleVar(s.role)]));
  const style = new Map(series.map((s) => [s.id, s]));
  const Lines = ({ series: drawn, lineGenerator }: LineCustomSvgLayerProps<Plotted>) => (
    <g>
      {drawn.map((s) => {
        const d = lineGenerator(s.data.map((p) => p.position));
        const spec = style.get(String(s.id));
        return d ? (
          <path
            key={String(s.id)}
            d={d}
            fill="none"
            stroke={colorOf.get(String(s.id)) ?? roleVar('truth')}
            strokeWidth={spec?.width ?? 1.75}
            strokeDasharray={spec?.dashed ? '6 4' : undefined}
          />
        ) : null;
      })}
    </g>
  );
  return (
    <div style={{ height }} role="img" aria-label={ariaLabel}>
      <ResponsiveLine
        data={series.map((s) => ({
          id: s.id,
          // A log axis cannot place 0 (Nivo throws), and a running mean is 0 at its first sample: leave a gap.
          data: yLog ? s.data.map((d) => (d.y !== null && d.y <= 0 ? { x: d.x, y: null } : d)) : s.data,
        }))}
        margin={{ top: 12, right: 16, bottom: 44, left: 56 }}
        xScale={{ type: 'linear', min: 'auto', max: 'auto' }}
        yScale={
          yLog
            ? { type: 'log', base: 10, min: yMin ?? 'auto', max: yMax ?? 'auto' }
            : { type: 'linear', min: yMin ?? 'auto', max: yMax ?? 'auto' }
        }
        colors={(s) => colorOf.get(String(s.id)) ?? roleVar('truth')}
        layers={['grid', 'markers', 'axes', 'areas', 'crosshair', Lines, 'points', 'slices', 'mesh', 'legends']}
        enablePoints={false}
        enableGridX={false}
        useMesh
        animate={false}
        axisBottom={{ legend: xLabel, legendOffset: 36, legendPosition: 'middle', tickSize: 4 }}
        axisLeft={{ legend: yLabel, legendOffset: -46, legendPosition: 'middle', tickSize: 4, tickValues: yTicks }}
        markers={refLines.map((r) => ({
          axis: 'y' as const,
          value: r.y,
          legend: r.label,
          legendPosition: r.side === 'left' ? ('top-left' as const) : ('top-right' as const),
          lineStyle: { stroke: roleVar(r.role ?? 'truth'), strokeWidth: 1.25, strokeDasharray: '5 4' },
          textStyle: { fill: 'var(--ink-muted)', fontSize: 11 },
        }))}
        theme={{
          text: { fill: 'var(--ink-muted)', fontSize: 11 },
          axis: { legend: { text: { fill: 'var(--ink-muted)', fontSize: 12 } }, ticks: { line: { stroke: 'var(--grid)' } }, domain: { line: { stroke: 'var(--grid)' } } },
          grid: { line: { stroke: 'var(--grid)', strokeWidth: 1 } },
          crosshair: { line: { stroke: 'var(--ink-muted)' } },
          tooltip: { container: { background: 'var(--surface-raised)', color: 'var(--ink)', fontSize: 12, border: '1px solid var(--grid)' } },
        }}
      />
    </div>
  );
}
