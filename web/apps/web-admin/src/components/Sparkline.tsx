import { useState } from "react";
import styles from "./Sparkline.module.css";

export interface Point {
  t: number;
  v: number;
}

const W = 600;
const H = 110;

/** Single-series line+area over time, with a crosshair and tooltip on hover.
 *
 *  One series, so no legend: the panel title names it. The SVG is drawn in a
 *  fixed 600×110 user space and stretched to the panel width — strokes carry
 *  `vector-effect: non-scaling-stroke` so they stay 2 px instead of smearing
 *  with the aspect ratio, and the hover dot is an HTML element positioned in
 *  percent rather than a circle that would squash. */
export function Sparkline({
  points,
  unit,
  emptyLabel = "waiting for the first sample…",
}: {
  points: Point[];
  unit: string;
  emptyLabel?: string;
}) {
  const [hover, setHover] = useState<number | null>(null);

  if (points.length < 2) {
    return <div className={styles.empty}>{emptyLabel}</div>;
  }

  const peak = Math.max(1, ...points.map((p) => p.v));
  const x = (i: number) => (i / (points.length - 1)) * W;
  const y = (v: number) => H - (v / peak) * (H - 10) - 5;

  const line = points.map((p, i) => `${i === 0 ? "M" : "L"}${x(i).toFixed(1)},${y(p.v).toFixed(1)}`).join(" ");
  const area = `${line} L${W},${H} L0,${H} Z`;

  const active = hover != null ? points[hover] : undefined;

  return (
    <div
      className={styles.wrap}
      onMouseLeave={() => setHover(null)}
      onMouseMove={(e) => {
        const r = e.currentTarget.getBoundingClientRect();
        const frac = (e.clientX - r.left) / r.width;
        setHover(Math.min(points.length - 1, Math.max(0, Math.round(frac * (points.length - 1)))));
      }}
    >
      <svg className={styles.svg} viewBox={`0 0 ${W} ${H}`} preserveAspectRatio="none" role="img"
           aria-label={`${unit} over time, peak ${peak}`}>
        <path className={styles.area} d={area} />
        <path className={styles.line} d={line} />
        {hover != null && (
          <line className={styles.crosshair} x1={x(hover)} y1={0} x2={x(hover)} y2={H} />
        )}
      </svg>

      <span className={styles.peak}>peak {peak} {unit}</span>

      {active && hover != null && (
        <>
          <span
            className={styles.dot}
            style={{ left: `${(hover / (points.length - 1)) * 100}%`, top: `${(y(active.v) / H) * 100}%` }}
          />
          <div
            className={styles.tip}
            style={{ left: `${(hover / (points.length - 1)) * 100}%` }}
            // flip the tooltip to the inside near the edges so it never clips
            data-side={hover / (points.length - 1) > 0.75 ? "left" : "right"}
          >
            <strong>{active.v}</strong> {unit}
            <span>{new Date(active.t).toLocaleTimeString()}</span>
          </div>
        </>
      )}
    </div>
  );
}
