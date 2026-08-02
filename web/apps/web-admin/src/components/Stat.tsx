import type { Tone } from "../lib/types";
import styles from "./Stat.module.css";

/** One reading: label, value, optional unit and sub-caption.
 *
 *  A number is a stat tile, not a chart — there is nothing to compare it
 *  against within itself. Tone tints the VALUE only; the label and caption keep
 *  text ink, so colour never becomes the only carrier of meaning. */
export function Stat({
  label,
  value,
  unit,
  caption,
  tone = "idle",
  wide,
}: {
  label: string;
  value: string;
  unit?: string;
  caption?: string;
  tone?: Tone;
  wide?: boolean;
}) {
  return (
    <div className={`${styles.stat} ${wide ? styles.wide : ""}`}>
      <span className={styles.label}>{label}</span>
      <span className={`${styles.value} ${styles[tone]}`}>
        {value}
        {unit && <span className={styles.unit}>{unit}</span>}
      </span>
      {caption && <span className={styles.caption}>{caption}</span>}
    </div>
  );
}

export function StatGrid({ children }: { children: React.ReactNode }) {
  return <div className={styles.grid}>{children}</div>;
}
