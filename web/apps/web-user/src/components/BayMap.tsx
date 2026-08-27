import { useEffect, useMemo, useRef } from "react";
import {
  Circle,
  CircleMarker,
  MapContainer,
  Polygon,
  Polyline,
  Popup,
  TileLayer,
  Tooltip,
  ZoomControl,
  useMap,
} from "react-leaflet";
import { latLngBounds, type LatLngExpression } from "leaflet";
import type {
  BayFC,
  BayFeature,
  BayProps,
  BayStatus,
  NoFlyFC,
  Restriction,
  RouteResult,
  CurbRunFC,
} from "../lib/types";
import { bayStatus } from "../lib/types";

/** How each occupancy backend is named to a driver. The stored values are
 *  `heuristic` / `detector` (migration 0010); anything unrecognised falls
 *  through and is shown verbatim rather than hidden. */
const BACKEND_LABEL: Record<string, string> = {
  heuristic: "colour classifier",
  detector: "learned detector",
};
import { bayCentroid, directionsUrl, type NearestTarget, type UserPos } from "../lib/geo";
import type { LiveState } from "../hooks/useOccupancySocket";
import { NoFlyLayer } from "./NoFlyLayer";
import { CurbRunLayer } from "./CurbRunLayer";
import styles from "./BayMap.module.css";

const ORIGIN: LatLngExpression = [42.6747105, 23.3298956];

/** Imperatively drives the map: recenters on locate, frames the journey once a
 *  target (and then its road route) arrives. */
function MapView({
  userPos,
  focusKey,
  target,
  routePositions,
}: {
  userPos: UserPos | null;
  focusKey: number;
  target: NearestTarget | null;
  routePositions: LatLngExpression[] | null;
}) {
  const map = useMap();

  useEffect(() => {
    if (userPos && focusKey > 0) {
      map.flyTo([userPos.lat, userPos.lon], Math.max(map.getZoom(), 17));
    }
  }, [focusKey]); // eslint-disable-line react-hooks/exhaustive-deps

  // Frame at most twice per target — straight-line guess, then the real route.
  // Re-routes while driving must not keep yanking the view out from under you.
  const framedFor = useRef<string | null>(null);
  useEffect(() => {
    if (!target) {
      framedFor.current = null;
      return;
    }
    const key = `${target.bayId}:${routePositions ? "route" : "direct"}`;
    if (framedFor.current === key) return;
    framedFor.current = key;

    if (routePositions && routePositions.length > 1) {
      map.fitBounds(latLngBounds(routePositions), { padding: [70, 70], maxZoom: 18 });
    } else if (userPos) {
      map.fitBounds(
        [
          [userPos.lat, userPos.lon],
          [target.lat, target.lon],
        ],
        { padding: [60, 60], maxZoom: 18 },
      );
    } else {
      map.flyTo([target.lat, target.lon], 18);
    }
  }, [target, routePositions]); // eslint-disable-line react-hooks/exhaustive-deps

  return null;
}

const COLOR: Record<BayStatus, string> = {
  free: "#35C46E", // zelena — Sofia green zone
  occupied: "#FF5C3E", // signal vermilion
  unknown: "#9AA6AE", // muted / unsurveyed
  // Amber, and deliberately none of the other three: a closure is not a verdict
  // the drone reached, so reading as either red or green would be a category
  // error, and grey would say "we did not see it" when in fact we know exactly
  // what is going on. Hazard amber is the road-works idiom drivers already read.
  closed: "#F0A020",
};
const TELEMETRY = "#14B8A6"; // drone/user accent

function mergeLive(p: BayProps, live?: LiveState): BayProps {
  if (!live) return p;
  return {
    ...p,
    occupied: live.occupied,
    confidence: live.confidence,
    updated_at: live.updated_at,
    // A closure delta carries `closed`; a vision delta does not, and must not
    // be read as "not closed" — undefined means "unchanged", so fall back to
    // what the last fetch said rather than clobbering it.
    closed: live.closed ?? p.closed,
  };
}

export function BayMap({
  fc,
  live,
  userPos,
  focusKey,
  target,
  route,
  nofly,
  showZones,
  runs,
  showRuns,
  showBays,
}: {
  fc: BayFC;
  live: Map<string, LiveState>;
  userPos: UserPos | null;
  focusKey: number;
  target: NearestTarget | null;
  route: RouteResult | null;
  nofly: NoFlyFC | null;
  showZones: Set<Restriction>;
  runs: CurbRunFC | null;
  showRuns: boolean;
  showBays: boolean;
}) {
  // Ring conversion is stable; recompute only when the feature set changes.
  const rings = useMemo(
    () =>
      fc.features.map((f) => ({
        id: f.properties.bay_id,
        positions: f.geometry.coordinates[0]!.map(
          ([lon, lat]) => [lat, lon] as LatLngExpression,
        ),
      })),
    [fc],
  );

  // GeoJSON [lon,lat] -> Leaflet [lat,lon]
  const routePositions = useMemo(
    () =>
      route
        ? route.coordinates.map(([lon, lat]) => [lat, lon] as LatLngExpression)
        : null,
    [route],
  );

  return (
    <MapContainer center={ORIGIN} zoom={17} className={styles.map} preferCanvas zoomControl={false}>
      <ZoomControl position="bottomleft" />
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>'
        url="https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png"
        subdomains="abcd"
        maxZoom={20}
      />
      {/* Airspace first: under the bays visually, and — because preferCanvas
          resolves clicks by draw order — behind them for hit-testing too.
          See the note in NoFlyLayer. */}
      <NoFlyLayer fc={nofly} show={showZones} />

      {/* Curb runs BEFORE the bays, and for a sharper version of the same
          reason: a run lies on top of its own bays by construction, so drawn
          after them it would swallow every one of their clicks. */}
      <CurbRunLayer fc={runs} show={showRuns} />

      {/* The bays are a LAYER, switchable like the kerbs and the airspace. The
          two answer the same question from incompatible geometry — a rectangle
          per published space against a count along a kerb — and on this data
          they disagree by design (the per-bay rule finds 9 of 29 hand-counted
          cars where the run layer finds 19), so being able to see either one
          alone is what makes the comparison readable instead of a thicket.
          Switching bays off leaves the kerb answer on its own.
          Note this only stops them being DRAWN: `fc` still feeds the counts in
          SurveyReadout and `nearestFree`, because hiding a layer is a viewing
          choice and must not change what the survey reports. */}
      {showBays &&
        rings.map((r, i) => {
          const props = mergeLive(fc.features[i]!.properties, live.get(r.id));
          const status = bayStatus(props);
          return (
            <Polygon
              key={r.id}
              positions={r.positions}
              pathOptions={{
                // painted-bay look: crisp outline, status-filled interior
                color: status === "unknown" ? "#C6CED2" : COLOR[status],
                weight: status === "unknown" ? 0.5 : 1.2,
                fillColor: COLOR[status],
                fillOpacity: status === "unknown" ? 0.12 : 0.55,
              }}
            >
              <Popup>
                <BayPopup
                  feature={fc.features[i]!}
                  props={props}
                  status={status}
                  userPos={userPos}
                />
              </Popup>
            </Polygon>
          );
        })}

      {/* Everything below the bays in intent but ABOVE them in draw order is
          marked `interactive: false`. With preferCanvas all vectors share one
          canvas, and Leaflet's Canvas._onClick walks the draw order keeping the
          LAST interactive layer under the cursor — so a decorative overlay drawn
          after the bays silently swallows their clicks (the 25 m accuracy circle
          hid every bay around the driver). CSS pointer-events can't help: one
          canvas, no per-shape elements. The you-are-here dot below stays
          interactive because its tooltip is hover-triggered. */}

      {/* the driving route: dark casing under a telemetry-coloured line */}
      {routePositions && (
        <>
          <Polyline
            positions={routePositions}
            pathOptions={{
              color: "#0b3a35",
              weight: 8,
              opacity: 0.5,
              lineCap: "round",
              lineJoin: "round",
              interactive: false,
            }}
          />
          <Polyline
            positions={routePositions}
            pathOptions={{
              color: TELEMETRY,
              weight: 4.5,
              opacity: 0.95,
              lineCap: "round",
              lineJoin: "round",
              interactive: false,
            }}
          />
        </>
      )}

      {/* fallback while the route loads, or if the router is unavailable */}
      {!routePositions && userPos && target && (
        <Polyline
          positions={[
            [userPos.lat, userPos.lon],
            [target.lat, target.lon],
          ]}
          pathOptions={{
            color: TELEMETRY,
            weight: 3,
            dashArray: "2 8",
            lineCap: "round",
            interactive: false,
          }}
        />
      )}

      {/* the chosen nearest free bay — non-interactive so the bay it marks stays
          clickable; a permanent tooltip needs no mouse events */}
      {target && (
        <CircleMarker
          center={[target.lat, target.lon]}
          radius={10}
          pathOptions={{
            color: TELEMETRY,
            fillColor: COLOR.free,
            fillOpacity: 1,
            weight: 3,
            interactive: false,
          }}
        >
          <Tooltip permanent direction="top" offset={[0, -8]} className={styles.teleTip}>
            NEAREST FREE · {Math.round(target.distance)} m
          </Tooltip>
        </CircleMarker>
      )}

      {/* the user's current location + GPS accuracy */}
      {userPos && (
        <>
          <Circle
            center={[userPos.lat, userPos.lon]}
            radius={userPos.accuracy}
            pathOptions={{ color: TELEMETRY, weight: 1, fillOpacity: 0.08, interactive: false }}
          />
          <CircleMarker
            center={[userPos.lat, userPos.lon]}
            radius={7}
            pathOptions={{ color: "#fff", fillColor: TELEMETRY, fillOpacity: 1, weight: 2 }}
          >
            <Tooltip direction="bottom">You are here</Tooltip>
          </CircleMarker>
        </>
      )}

      <MapView
        userPos={userPos}
        focusKey={focusKey}
        target={target}
        routePositions={routePositions}
      />
    </MapContainer>
  );
}

function BayPopup({
  feature,
  props,
  status,
  userPos,
}: {
  feature: BayFeature;
  props: BayProps;
  status: BayStatus;
  userPos: UserPos | null;
}) {
  return (
    <div className={styles.popup}>
      <strong>Bay {props.bay_id}</strong>
      <div className={`${styles.badge} ${styles[status]}`}>{status.toUpperCase()}</div>
      {/* The closure is stated FIRST and in its own block, not as one more row
          in the list: it is the reason the badge says CLOSED, and it overrides
          everything below it. The occupancy rows are deliberately still shown —
          the camera's reading is not hidden, it is simply not the answer. */}
      {status === "closed" && (
        <p className={styles.closureNote}>
          {props.closure?.reason ?? "Street closed"}
          {props.closure?.valid_to
            ? ` · until ${new Date(props.closure.valid_to).toLocaleString()}`
            : " · no end date given"}
        </p>
      )}
      <dl>
        <dt>Street</dt><dd>{props.street ?? "—"}</dd>
        <dt>Zone</dt><dd>{props.zona ?? "—"}</dd>
        {props.confidence != null && (<><dt>Confidence</dt><dd>{Math.round(props.confidence * 100)}%</dd></>)}
        {/* Which model decided this bay. Two now can, and they are not
            comparable -- the heuristic is calibrated on simulated tones and the
            detector is trained on real photographs -- so the verdict is only
            fully readable next to its provenance. Note `Confidence` above is
            the multi-view vote agreement, NOT a model score. */}
        {props.backend != null && (
          <><dt>Source</dt><dd>{BACKEND_LABEL[props.backend] ?? props.backend}</dd></>
        )}
        <dt>Updated</dt>
        <dd>{props.updated_at ? new Date(props.updated_at).toLocaleTimeString() : "never surveyed"}</dd>
      </dl>
      {status === "free" && (
        <a
          className={styles.dirLink}
          href={directionsUrl(userPos, bayCentroid(feature))}
          target="_blank"
          rel="noreferrer"
        >
          Directions ▸
        </a>
      )}
    </div>
  );
}
