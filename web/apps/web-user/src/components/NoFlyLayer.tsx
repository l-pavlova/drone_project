import { useMemo } from "react";
import { Polygon, Popup } from "react-leaflet";
import type { LatLngExpression } from "leaflet";
import type { NoFlyFC, NoFlyProps, Restriction } from "../lib/types";
import styles from "./NoFlyLayer.module.css";

/** Airspace hues, deliberately outside the bay palette. Bays are small solid
 *  green/vermilion rectangles; zones are large translucent areas with dashed
 *  borders, so even the crimson cannot be mistaken for an occupied bay. */
const ZONE: Record<Restriction, { color: string; fill: number; dash: string }> = {
  PROHIBITED: { color: "#D7263D", fill: 0.14, dash: "7 5" },
  REQ_AUTHORISATION: { color: "#F5A524", fill: 0.07, dash: "3 7" },
  CONDITIONAL: { color: "#F5A524", fill: 0.05, dash: "2 8" },
};

const LABEL: Record<Restriction, string> = {
  PROHIBITED: "NO-FLY",
  REQ_AUTHORISATION: "AUTHORISATION REQUIRED",
  CONDITIONAL: "CONDITIONAL",
};

/** "P07DT00H00M" -> "7 days". Only the day field is ever populated in the CAA
 *  file, so anything else is shown verbatim rather than half-parsed. */
function leadTime(iso: string | null): string | null {
  if (!iso) return null;
  const m = /^P(\d+)D/.exec(iso);
  return m ? `${Number(m[1])} day${Number(m[1]) === 1 ? "" : "s"} ahead` : iso;
}

function band(p: NoFlyProps): string {
  if (p.lower_limit == null || p.upper_limit == null) return "—";
  return `${p.lower_limit}–${p.upper_limit} m ${p.vertical_reference ?? ""}`.trim();
}

/**
 * Published UAS geographical zones, drawn UNDER the bays.
 *
 * Draw order is load-bearing, not cosmetic. With `preferCanvas` every vector
 * shares one canvas and Leaflet's Canvas._onClick keeps the LAST interactive
 * layer under the cursor — so a zone rendered after the bays would swallow
 * every bay click inside it (the same trap the accuracy circle fell into, see
 * BayMap). Rendering zones first means a bay always wins where they overlap,
 * while the zone stays clickable on bare ground. That also puts the hazard
 * shading beneath the data, which is the right visual order anyway.
 */
export function NoFlyLayer({ fc, show }: { fc: NoFlyFC | null; show: Set<Restriction> }) {
  const rings = useMemo(
    () =>
      (fc?.features ?? []).map((f) => ({
        id: f.properties.zone_id,
        props: f.properties,
        positions: f.geometry.coordinates[0]!.map(
          ([lon, lat]) => [lat, lon] as LatLngExpression,
        ),
      })),
    [fc],
  );

  return (
    <>
      {rings
        .filter((r) => show.has(r.props.restriction))
        .map((r) => {
          const s = ZONE[r.props.restriction];
          return (
            <Polygon
              key={r.id}
              positions={r.positions}
              pathOptions={{
                color: s.color,
                weight: 1.6,
                dashArray: s.dash,
                fillColor: s.color,
                fillOpacity: s.fill,
              }}
            >
              <Popup>
                <ZonePopup p={r.props} />
              </Popup>
            </Polygon>
          );
        })}
    </>
  );
}

function ZonePopup({ p }: { p: NoFlyProps }) {
  const lead = leadTime(p.authority?.interval_before ?? null);
  return (
    <div className={styles.popup}>
      <div className={`${styles.badge} ${styles[p.restriction]}`}>{LABEL[p.restriction]}</div>
      <strong>{p.name ?? p.identifier ?? "Unnamed zone"}</strong>
      <dl>
        <dt>Altitude</dt><dd>{band(p)}</dd>
        <dt>Reason</dt><dd>{p.reason.length ? p.reason.join(", ").toLowerCase() : "—"}</dd>
        <dt>Applies</dt><dd>{p.permanent ? "permanently" : "see schedule"}</dd>
        {p.authority?.name && (<><dt>Authority</dt><dd>{p.authority.name}</dd></>)}
        {lead && (<><dt>Apply</dt><dd>{lead}</dd></>)}
      </dl>
      {p.message && <p className={styles.msg}>{p.message}</p>}
      <div className={styles.src}>zone {p.identifier} · Bulgarian CAA</div>
    </div>
  );
}
