import { count, percent } from "../lib/format";
import type { CoverageMetrics } from "../lib/types";
import styles from "./CoverageBar.module.css";

/** How much of the city the survey currently knows about, as one stacked bar.
 *
 *  Free-green and occupied-red are the map's own status colours, and that pair
 *  is weak under deuteranopia (ΔE ~6), so identity never rests on the colour:
 *  every segment is directly labelled with its count below the bar, and the
 *  segments are separated by a 2 px surface gap so the boundaries stay legible
 *  in greyscale too. */
export function CoverageBar({ c }: { c: CoverageMetrics }) {
  const total = Math.max(1, c.bays_total);
  const segments = [
    { key: "occupied", label: "Occupied", n: c.bays_occupied },
    { key: "free", label: "Free", n: c.bays_free },
    { key: "unknown", label: "Unsurveyed", n: c.bays_unknown },
  ] as const;

  return (
    <div>
      <div className={styles.bar} role="img"
           aria-label={segments.map((s) => `${s.label} ${s.n}`).join(", ")}>
        {segments.map((s) =>
          s.n > 0 ? (
            <span
              key={s.key}
              className={`${styles.seg} ${styles[s.key]}`}
              style={{ flexGrow: s.n }}
              title={`${s.label}: ${count(s.n)}`}
            />
          ) : null,
        )}
      </div>
      <ul className={styles.legend}>
        {segments.map((s) => (
          <li key={s.key} className={styles.item}>
            <span className={`${styles.swatch} ${styles[s.key]}`} aria-hidden="true" />
            <span className={styles.name}>{s.label}</span>
            <span className={styles.n}>{count(s.n)}</span>
            <span className={styles.pct}>{percent(s.n / total, 0)}</span>
          </li>
        ))}
      </ul>
    </div>
  );
}
