import { Fragment, useMemo } from "react";
import { Polyline, Popup } from "react-leaflet";
import type { LatLngExpression } from "leaflet";
import type { CurbRunFC, CurbRunGap, CurbRunProps } from "../lib/types";
import { haversine } from "../lib/geo";
import styles from "./CurbRunLayer.module.css";

/** Free-space hues. Deliberately the bay palette's logic (green = space for you,
 *  vermilion = none) so a driver reads both layers the same way, but drawn as a
 *  thick line rather than a rectangle: this layer answers "how many spaces along
 *  this kerb", not "is that box occupied". */
function colour(p: CurbRunProps): string {
  // never surveyed, or the state has gone stale past the freshness window
  if (p.free == null || p.cars == null || !p.capacity_observed) return "#C6CED2";
  const frac = p.free / p.capacity_observed;
  if (p.free === 0) return "#D7263D";
  if (frac < 0.25) return "#F5A524";
  return "#2E9E5B";
}

/** The stretch of a run's polyline between two arclengths, as leaflet points.
 *
 *  Gaps are reported in the run's own arclength metres (the server measures them
 *  along the same polyline it publishes), so drawing one is a matter of walking
 *  the line and cutting it at s0 and s1 — including the partial segment at each
 *  end, or a short gap inside one long segment would render as nothing at all.
 *
 *  Distances are haversine where the server used ENU metres. The two differ by
 *  well under 0.1% at this latitude and block scale — under 10 cm on an 80 m
 *  run, far below a pixel at any zoom this map offers — and the alternative is a
 *  FIFTH copy of the ORIGIN/MLAT/MLON constants (generate_world.py,
 *  score_occupancy.py, packages/contracts, the sim controller) whose drift would
 *  be a real bug. Position is all that is taken from this; every number shown to
 *  the user is the server's.
 */
function sliceByArclength(
  positions: LatLngExpression[],
  s0: number,
  s1: number,
): LatLngExpression[] {
  const pts = positions as [number, number][];
  const out: [number, number][] = [];
  let acc = 0;
  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i];
    const b = pts[i + 1];
    if (!a || !b) continue;
    const [aLat, aLon] = a;
    const [bLat, bLon] = b;
    const seg = haversine(aLat, aLon, bLat, bLon);
    if (seg <= 0) continue;
    const segEnd = acc + seg;
    if (segEnd > s0 && acc < s1) {
      const t0 = Math.max(0, (s0 - acc) / seg);
      const t1 = Math.min(1, (s1 - acc) / seg);
      const at = (t: number): [number, number] => [
        aLat + (bLat - aLat) * t,
        aLon + (bLon - aLon) * t,
      ];
      if (out.length === 0) out.push(at(t0));
      out.push(at(t1));
    }
    acc = segEnd;
  }
  return out as LatLngExpression[];
}

/**
 * Parking as a kerb you can park ALONG, not a grid of boxes.
 *
 * This is the answer real footage supports. On flight 0035 the per-bay rule
 * finds 9 of 29 hand-counted cars and shows the street as ~11% full when it is
 * about 57% full; the run layer recovers significantly more (+0.58 cars per
 * 18 m segment, 95% CI [+0.21, +0.96]). The bays are wrong by more than a bay
 * width and no rigid correction fixes them, but a count along a kerb survives
 * that error because sliding every car along the kerb by the same amount does
 * not change how many fit in the gaps.
 *
 * Drawn BEFORE the bays for the same reason NoFlyLayer is: with `preferCanvas`
 * every vector shares one canvas and Leaflet's Canvas._onClick keeps the LAST
 * interactive layer under the cursor, so a run drawn after the bays would
 * swallow the clicks of every bay lying on it — and a run lies on its own bays
 * by construction, which makes this the worst possible case of that trap.
 *
 * A run with no fresh state is drawn thin and grey rather than hidden: "nobody
 * has surveyed this kerb" is a different statement from "there is no parking
 * here", and the whole layer exists to keep those apart.
 */
export function CurbRunLayer({
  fc,
  show,
}: {
  fc: CurbRunFC | null;
  show: boolean;
}) {
  const lines = useMemo(
    () =>
      (fc?.features ?? []).map((f) => ({
        id: f.properties.run_id,
        props: f.properties,
        positions: f.geometry.coordinates.map(
          ([lon, lat]) => [lat, lon] as LatLngExpression,
        ),
      })),
    [fc],
  );

  if (!show || !fc) return null;

  return (
    <>
      {lines.map((l) => {
        const known = l.props.free != null;
        return (
          <Fragment key={l.id}>
            <Polyline
              positions={l.positions}
              pathOptions={{
                color: colour(l.props),
                weight: known ? 7 : 3,
                opacity: known ? 0.8 : 0.35,
                lineCap: "round",
              }}
            >
              <Popup>
                <RunPopup props={l.props} />
              </Popup>
            </Polyline>
            {/* The measured gaps, drawn ON the run as bright inner segments —
                this is the answer the subtraction could not give: not how many
                spaces, but WHERE to drive. Non-interactive (`interactive:
                false`), so they cannot take the click from the run underneath
                them or from a bay: with `preferCanvas` every vector shares one
                canvas and Leaflet's Canvas._onClick keeps the last interactive
                layer under the cursor, which is the trap NoFlyLayer and the
                accuracy circle both fell into. A gap lies exactly on its own
                run's bays by construction, so it is the worst possible case. */}
            {(l.props.gaps ?? []).map((g, i) => {
              const seg = sliceByArclength(l.positions, g.s0, g.s1);
              if (seg.length < 2) return null;
              return (
                <Polyline
                  key={`${l.id}-gap-${i}`}
                  positions={seg}
                  interactive={false}
                  pathOptions={{
                    color: "#7BE3A3",
                    weight: 3,
                    opacity: 0.95,
                    lineCap: "butt",
                  }}
                />
              );
            })}
          </Fragment>
        );
      })}
    </>
  );
}

function RunPopup({ props: p }: { props: CurbRunProps }) {
  const seen = p.observed_fraction;
  return (
    <div className={styles.popup}>
      <div className={styles.street}>{p.street ?? "unnamed street"}</div>
      {p.free == null ? (
        <div className={styles.unknown}>Not surveyed recently</div>
      ) : (
        <>
          <div className={styles.headline}>
            <span className={styles.big}>{p.free}</span> free
            <span className={styles.of}> of {p.capacity_observed} seen</span>
          </div>
          <dl className={styles.rows}>
            <dt>Cars counted</dt>
            <dd>{p.cars}</dd>
            <dt>Kerb surveyed</dt>
            {/* Shown always, and shown as an upper bound on purpose: the mask is
                geometric, so kerb under tree canopy counts as seen. A driver
                sent to a "free" stretch nobody could actually see is the one
                failure worse than saying nothing. */}
            <dd>{seen == null ? "—" : `${Math.round(seen * 100)}%`}</dd>
            <dt>City capacity</dt>
            <dd>{p.capacity}</dd>
            {p.backend && (
              <>
                <dt>Source</dt>
                <dd>{p.backend}</dd>
              </>
            )}
          </dl>
          {/* The gaps themselves. `free` is the sum of their `spaces`, so this
              is the working behind the headline number rather than extra
              detail — and it is the part a driver acts on. */}
          {p.gaps && p.gaps.length > 0 && (
            <ul className={styles.gaps}>
              {p.gaps.map((g: CurbRunGap, i: number) => (
                <li key={i}>
                  {g.spaces} space{g.spaces === 1 ? "" : "s"} in {Math.round(g.len_m)} m
                </li>
              ))}
            </ul>
          )}
        </>
      )}
    </div>
  );
}
