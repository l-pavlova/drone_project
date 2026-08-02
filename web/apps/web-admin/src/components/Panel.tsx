import type { ReactNode } from "react";
import type { Tone } from "../lib/types";
import styles from "./Panel.module.css";

/** A titled section of the readout. `note` is the right-aligned caption used for
 *  provenance ("last 5 min", "since page open") — every panel says where its
 *  numbers come from, so nothing on screen is unattributed. */
export function Panel({
  title,
  note,
  tone,
  children,
}: {
  title: string;
  note?: string;
  tone?: Tone;
  children: ReactNode;
}) {
  return (
    <section className={styles.panel}>
      <header className={styles.head}>
        <h2 className={styles.title}>
          {tone && <span className={`${styles.dot} ${styles[tone]}`} aria-hidden="true" />}
          {title}
        </h2>
        {note && <span className={styles.note}>{note}</span>}
      </header>
      {children}
    </section>
  );
}
