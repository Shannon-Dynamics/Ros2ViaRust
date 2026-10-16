'use client';
import { ResponsiveLine, type LineCustomSvgLayerProps } from '@nivo/line';
import type { PartialTheme } from '@nivo/theming';
import { roleVar, type Role } from '@/lib/roles';
import { TooltipCard } from './tooltip-card';

/**
 * The book's Nivo theme (the reference's): a recessive hairline grid, no tick marks, tabular mono
 * numerals on every axis, small semibold axis legends (the reference's eyebrow style, minus the capitals), and a transparent background so a chart
 * takes the color of the panel that holds it. Every value is a CSS variable, so the chart follows
 * the light/dark theme with no JavaScript; the tooltip container is flattened because
 * <TooltipCard> draws the real one.
 */
const CHART_THEME: PartialTheme = {
  background: 'transparent',
  text: { fontFamily: 'var(--font-ui)', fontSize: 11, fill: 'var(--color-fd-muted-foreground)' },
  axis: {
    domain: { line: { stroke: 'var(--color-fd-border)', strokeWidth: 1 } },
    ticks: {
      // Gridlines carry the reading; tick marks would just add ink.
      line: { stroke: 'transparent', strokeWidth: 0 },
      text: { fontFamily: 'var(--font-mono)', fontSize: 10, fill: 'var(--color-fd-muted-foreground)', fontVariantNumeric: 'tabular-nums' },
    },
    legend: {
      text: {
        fontFamily: 'var(--font-ui)',
        fontSize: 10,
        fontWeight: 600,
        // Not uppercased as in the reference: an axis legend carries units, and "(m)" is not "(M)".
        letterSpacing: '0.03em',
        fill: 'var(--color-fd-muted-foreground)',
      },
    },
  },
  grid: { line: { stroke: 'var(--pr-grid)', strokeWidth: 1 } },
  crosshair: { line: { stroke: 'var(--color-fd-foreground)', strokeWidth: 1, strokeOpacity: 0.3, strokeDasharray: 'none' } },
  tooltip: {
    container: { background: 'transparent', boxShadow: 'none', border: 'none', borderRadius: 0, padding: 0, color: 'inherit', fontFamily: 'var(--font-ui)' },
  },
};

/** Compact fixed-point formatting for tooltips: enough digits to tell a value from zero, few enough to scan. */
function formatNumber(value: number): string {
  if (!Number.isFinite(value)) return '—';
  const magnitude = Math.abs(value);
  if (magnitude !== 0 && (magnitude >= 1e6 || magnitude < 1e-4)) return value.toExponential(2);
  const fixed = value.toFixed(magnitude >= 100 ? 1 : magnitude >= 1 ? 2 : 3);
  return fixed.replace(/0+$/, '').replace(/\.$/, '');
}

export interface Series {
  id: string;
  role: Role;
  data: { x: number; y: number | null }[];
  /** Dashed stroke. Ground truth is always dashed (design/TOC.md §2); so are bounds. */
  dashed?: boolean;
  /** Stroke width in px (default 2). */
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
            strokeWidth={spec?.width ?? 2}
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
        margin={{ top: 14, right: 22, bottom: 46, left: 58 }}
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
        axisTop={null}
        axisRight={null}
        axisBottom={{ tickSize: 0, tickPadding: 8, legend: xLabel, legendOffset: 36, legendPosition: 'middle' }}
        axisLeft={{ tickSize: 0, tickPadding: 8, legend: yLabel, legendOffset: -46, legendPosition: 'middle', tickValues: yTicks }}
        markers={refLines.map((r) => ({
          axis: 'y' as const,
          value: r.y,
          legend: r.label,
          legendPosition: r.side === 'left' ? ('top-left' as const) : ('top-right' as const),
          lineStyle: { stroke: roleVar(r.role ?? 'truth'), strokeWidth: 1.5, strokeOpacity: 0.7, strokeDasharray: '5 4' },
          textStyle: { fill: roleVar(r.role ?? 'truth'), fontFamily: 'var(--font-ui)', fontSize: 10 },
        }))}
        tooltip={({ point }) => (
          <TooltipCard
            title={`${xLabel} ${formatNumber(Number(point.data.x))}`}
            rows={[{ color: point.seriesColor, label: String(point.seriesId), value: formatNumber(Number(point.data.y)) }]}
          />
        )}
        theme={CHART_THEME}
      />
    </div>
  );
}
