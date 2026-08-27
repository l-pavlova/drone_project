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
  kerbs,
  bays,
}: {
  connected: boolean;
  zones: ZoneSummary[];
  counts: { free: number; occupied: number; unknown: number; closed: number };
  surveyTotal: number;
  /** Published UAS airspace overlay: how many zones are in view, and the
   *  on/off switch. Absent while the zone file is still loading. */
  airspace: { noFly: number; restricted: number; on: boolean; toggle: () => void } | null;
  /** Curb-run layer: free spaces counted along kerbs, and its switch. Null
   *  until the first poll lands, or if the endpoint is unavailable — the map is
   *  fully usable without it and the row simply stays hidden. */
  kerbs: { free: number; cars: number; on: boolean; toggle: () => void } | null;
  /** The published per-bay rectangles and their switch. Switching them off
   *  leaves the kerb answer alone on the map; it is a VIEWING choice, so the
   *  COVERAGE line above is unaffected either way. */
  bays: { on: boolean; toggle: () => void } | null;
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
          aria-label={`${counts.free} free, ${counts.occupied} occupied${
            counts.closed ? `, ${counts.closed} on a closed street` : ""
          }`}
        >
          <span className={`${styles.seg} ${styles.free}`} style={{ flexGrow: counts.free || 0.001 }} />
          <span className={`${styles.seg} ${styles.occ}`} style={{ flexGrow: counts.occupied || 0.001 }} />
          {/* Only shown when there IS a closure: an always-present empty
              segment would imply the map tracks something it usually does not. */}
          {counts.closed > 0 && (
            <span className={`${styles.seg} ${styles.closedSeg}`} style={{ flexGrow: counts.closed }} />
          )}
        </div>
        <div className={styles.meterLegend}>
          <span>
            <b>{counts.free}</b> free
          </span>
          <span>
            <b>{counts.occupied}</b> occupied
          </span>
          {counts.closed > 0 && (
            <span>
              <b>{counts.closed}</b> closed
            </span>
          )}
        </div>

        {/* Closed bays count as COVERED — the survey's job is done for them, the
            answer just came from an authority instead of the camera. Leaving
            them out would report a coverage gap that does not exist. */}
        <div className={styles.coverage}>
          COVERAGE <b>{counts.free + counts.occupied + counts.closed}</b> / {surveyTotal} BAYS
        </div>

        {bays && (
          <button
            className={`${styles.airspace} ${bays.on ? styles.airspaceOn : ""}`}
            aria-pressed={bays.on}
            onClick={bays.toggle}
            title="Published parking-bay rectangles (Sofiaplan). Switch off to read the kerb layer alone"
          >
            <i className={`${styles.swatch} ${styles.bay}`} />
            <span>
              BAYS <b>{counts.free}</b> FREE · {counts.occupied} TAKEN
            </span>
            <span className={styles.airspaceState}>{bays.on ? "ON" : "OFF"}</span>
          </button>
        )}

        {kerbs && (
          <button
            className={`${styles.airspace} ${kerbs.on ? styles.airspaceOn : ""}`}
            aria-pressed={kerbs.on}
            onClick={kerbs.toggle}
            title="Free spaces counted along each kerb (curb-run layer)"
          >
            <i className={`${styles.swatch} ${styles.kerb}`} />
            <span>
              KERBS <b>{kerbs.free}</b> FREE · {kerbs.cars} CARS
            </span>
            <span className={styles.airspaceState}>{kerbs.on ? "ON" : "OFF"}</span>
          </button>
        )}

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
