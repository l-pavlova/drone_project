import { useEffect, useState } from "react";
import { ChevronIcon } from "./Icons";
import type { CountSource, ZoneSummary } from "../lib/types";
import styles from "./SurveyReadout.module.css";

/** Remembered across reloads: a driver who moved the panel off the map expects
 *  it to stay off the map. */
const OPEN_KEY = "parkdrone.readout.open";

function readOpen(): boolean {
  try {
    return window.localStorage.getItem(OPEN_KEY) !== "0";
  } catch {
    // private mode / blocked storage: start expanded, same as a first visit
    return true;
  }
}

export interface BayCounts {
  free: number;
  occupied: number;
  unknown: number;
  closed: number;
}

/** The kerb layer's totals over the runs in view. `capacityObserved` is capacity
 *  scaled to the stretch actually surveyed — never the city's full `capacity` —
 *  so a flight that saw a third of a kerb cannot advertise the rest. */
export interface KerbCounts {
  free: number;
  cars: number;
  capacityObserved: number;
  capacityTotal: number;
}

/** What the headline reports, resolved from the selected source. One shape for
 *  both layers, so the markup below is written once rather than branched three
 *  times.
 *
 *  `spare` is the third meter segment and means something different in each: a
 *  bay on a CLOSED street, or kerb that was observed and holds neither a counted
 *  car nor a gap long enough to park in. The kerb case is exactly what the run
 *  layer exists to express and the bay layer cannot — measured on flight 0035,
 *  1102 spaces really fit against 1164 by subtraction, i.e. 62 advertised spaces
 *  that do not. Both are shown only when non-zero, so an always-present empty
 *  segment never implies the map tracks something it usually does not. */
interface Readout {
  free: number;
  taken: number;
  takenLabel: string;
  spare: number;
  spareLabel: string;
  coverageSeen: number;
  coverageTotal: number;
  coverageUnit: string;
}

function bayReadout(c: BayCounts, surveyTotal: number): Readout {
  return {
    free: c.free,
    taken: c.occupied,
    takenLabel: "occupied",
    spare: c.closed,
    spareLabel: "closed",
    // Closed bays count as COVERED — the survey's job is done for them, the
    // answer just came from an authority instead of the camera. Leaving them
    // out would report a coverage gap that does not exist.
    coverageSeen: c.free + c.occupied + c.closed,
    coverageTotal: surveyTotal,
    coverageUnit: "BAYS",
  };
}

function kerbReadout(k: KerbCounts): Readout {
  return {
    free: k.free,
    taken: k.cars,
    takenLabel: "cars",
    spare: Math.max(0, k.capacityObserved - k.free - k.cars),
    spareLabel: "no fit",
    coverageSeen: k.capacityObserved,
    coverageTotal: k.capacityTotal,
    coverageUnit: "SPACES",
  };
}

/** The ground-station overlay panel (top-left, floating over the map).
 *
 * Collapsible at every width. It used to be mobile-only, which left a 264px
 * panel permanently over the top-left of the map on desktop with no way to move
 * it aside; the chevron now shows at all widths and the state is remembered.
 *
 * **The BAYS/KERBS selector is not a viewing choice.** The three ON/OFF rows at
 * the bottom are — hiding a layer never changes a number here. The selector
 * changes what is reported: the headline, the meter, the COVERAGE line and what
 * the nearest-free button drives to. The two geometries disagree by design, and
 * being able to flip between them is what makes that legible. The two axes are
 * independent: reading the kerb answer does not require drawing it.
 */
export function SurveyReadout({
  connected,
  zones,
  counts,
  surveyTotal,
  kerbCounts,
  source,
  onSource,
  airspace,
  kerbs,
  bays,
}: {
  connected: boolean;
  zones: ZoneSummary[];
  counts: BayCounts;
  surveyTotal: number;
  /** Null until the first runs poll lands, or if the endpoint is unavailable —
   *  the selector then stays hidden and the panel reports bays, exactly as it
   *  did before this existed. */
  kerbCounts: KerbCounts | null;
  source: CountSource;
  onSource: (s: CountSource) => void;
  /** Published UAS airspace overlay: how many zones are in view, and the
   *  on/off switch. Absent while the zone file is still loading. */
  airspace: { noFly: number; restricted: number; on: boolean; toggle: () => void } | null;
  /** Curb-run layer: free spaces counted along kerbs, and its switch. Null
   *  until the first poll lands, or if the endpoint is unavailable — the map is
   *  fully usable without it and the row simply stays hidden. */
  kerbs: { free: number; cars: number; on: boolean; toggle: () => void } | null;
  /** The published per-bay rectangles and their switch. Switching them off
   *  leaves the kerb answer alone on the map; it is a VIEWING choice, so the
   *  numbers above are unaffected either way. */
  bays: { on: boolean; toggle: () => void } | null;
}) {
  const [open, setOpen] = useState(readOpen);

  useEffect(() => {
    try {
      window.localStorage.setItem(OPEN_KEY, open ? "1" : "0");
    } catch {
      // nothing to do — the panel still works, it just won't be remembered
    }
  }, [open]);

  // Falls back to bays when the kerb totals are missing, so a failed runs poll
  // can never blank the headline.
  const showKerbs = source === "kerbs" && kerbCounts !== null;
  const r = showKerbs ? kerbReadout(kerbCounts) : bayReadout(counts, surveyTotal);

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

        {kerbCounts && (
          <div className={styles.source} role="group" aria-label="Count source">
            <button
              className={`${styles.sourceOpt} ${source === "bays" ? styles.sourceOn : ""}`}
              aria-pressed={source === "bays"}
              onClick={() => onSource("bays")}
              title="Count the published parking-bay rectangles (Sofiaplan)"
            >
              BAYS
            </button>
            <button
              className={`${styles.sourceOpt} ${showKerbs ? styles.sourceOn : ""}`}
              aria-pressed={showKerbs}
              onClick={() => onSource("kerbs")}
              title="Count measured free stretches along each kerb (curb-run layer)"
            >
              KERBS
            </button>
          </div>
        )}

        <div className={styles.figure}>{r.free}</div>
        <div className={styles.figureLabel}>free spaces</div>

        <div
          className={styles.meter}
          role="img"
          aria-label={`${r.free} free, ${r.taken} ${r.takenLabel}${
            r.spare ? `, ${r.spare} ${r.spareLabel}` : ""
          }`}
        >
          <span className={`${styles.seg} ${styles.free}`} style={{ flexGrow: r.free || 0.001 }} />
          <span className={`${styles.seg} ${styles.occ}`} style={{ flexGrow: r.taken || 0.001 }} />
          {/* Only shown when there IS something in it: an always-present empty
              segment would imply the map tracks something it usually does not. */}
          {r.spare > 0 && (
            <span className={`${styles.seg} ${styles.closedSeg}`} style={{ flexGrow: r.spare }} />
          )}
        </div>
        <div className={styles.meterLegend}>
          <span>
            <b>{r.free}</b> free
          </span>
          <span>
            <b>{r.taken}</b> {r.takenLabel}
          </span>
          {r.spare > 0 && (
            <span>
              <b>{r.spare}</b> {r.spareLabel}
            </span>
          )}
        </div>

        <div className={styles.coverage}>
          COVERAGE <b>{r.coverageSeen}</b> / {r.coverageTotal} {r.coverageUnit}
        </div>

        {bays && (
          <button
            className={`${styles.airspace} ${bays.on ? styles.airspaceOn : ""}`}
            aria-pressed={bays.on}
            onClick={bays.toggle}
            title="Draw the published parking-bay rectangles (Sofiaplan). Viewing only — it does not change the counts above"
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
            title="Draw the kerb runs and their measured free stretches. Viewing only — it does not change the counts above"
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
