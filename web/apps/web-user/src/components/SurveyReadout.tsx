import { useState } from "react";
import { ChevronIcon } from "./Icons";
import type { ZoneSummary } from "../lib/types";
import styles from "./SurveyReadout.module.css";

/** The ground-station overlay panel (top-left, floating over the map).
 * Collapsible on mobile so it doesn't block the map — the toggle only
 * renders/does anything below the CSS mobile breakpoint (see .readout-toggle
 * in global.css); on desktop the body always stays expanded. */
export function SurveyReadout({
  connected,
  zones,
  counts,
  surveyTotal,
  airspace,
}: {
  connected: boolean;
  zones: ZoneSummary[];
  counts: { free: number; occupied: number; unknown: number };
  surveyTotal: number;
  /** Published UAS airspace overlay: how many zones are in view, and the
   *  on/off switch. Absent while the zone file is still loading. */
  airspace: { noFly: number; restricted: number; on: boolean; toggle: () => void } | null;
}) {
  const [open, setOpen] = useState(true);

  return (
    <section
      className={`${styles.readout} ${open ? "" : styles.closed}`}
      aria-label="Parking availability"
    >
      <div className={styles.eyebrow}>
        <span className={styles.wordmark}>PARKDRONE</span>
        <span className={`${styles.status} ${connected ? styles.live : styles.idle}`}>
          <i className={styles.beacon} />
          {connected ? "SURVEYING" : "IDLE"}
        </span>
        <button
          className={`${styles.readoutToggle} ${open ? "" : styles.closed}`}
          aria-label={open ? "Collapse panel" : "Expand panel"}
          aria-expanded={open}
          onClick={() => setOpen((o) => !o)}
        >
          <ChevronIcon />
        </button>
      </div>

      <div className={styles.readoutBody}>
        <div className={styles.locus}>SOFIA · {zones[0]?.zona ?? "—"}</div>

        <div className={styles.figure}>{counts.free}</div>
        <div className={styles.figureLabel}>free spaces</div>

        <div
          className={styles.meter}
          role="img"
          aria-label={`${counts.free} free, ${counts.occupied} occupied`}
        >
          <span className={`${styles.seg} ${styles.free}`} style={{ flexGrow: counts.free || 0.001 }} />
          <span className={`${styles.seg} ${styles.occ}`} style={{ flexGrow: counts.occupied || 0.001 }} />
        </div>
        <div className={styles.meterLegend}>
          <span>
            <b>{counts.free}</b> free
          </span>
          <span>
            <b>{counts.occupied}</b> occupied
          </span>
        </div>

        <div className={styles.coverage}>
          COVERAGE <b>{counts.free + counts.occupied}</b> / {surveyTotal} BAYS
        </div>

        {airspace && (
          <button
            className={`${styles.airspace} ${airspace.on ? styles.airspaceOn : ""}`}
            aria-pressed={airspace.on}
            onClick={airspace.toggle}
            title="Published UAS geographical zones (Bulgarian CAA)"
          >
            <i className={`${styles.swatch} ${styles.noFly}`} />
            <span>
              AIRSPACE <b>{airspace.noFly}</b> NO-FLY
              {airspace.restricted > 0 && <> · {airspace.restricted} RESTRICTED</>}
            </span>
            <span className={styles.airspaceState}>{airspace.on ? "ON" : "OFF"}</span>
          </button>
        )}
      </div>
    </section>
  );
}
