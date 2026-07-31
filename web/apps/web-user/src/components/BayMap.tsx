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
import type { BayFC, BayFeature, BayProps, BayStatus, RouteResult } from "../lib/types";
import { bayStatus } from "../lib/types";
import { bayCentroid, directionsUrl, type NearestTarget, type UserPos } from "../lib/geo";
import type { LiveState } from "../hooks/useOccupancySocket";
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
};
const TELEMETRY = "#14B8A6"; // drone/user accent

function mergeLive(p: BayProps, live?: LiveState): BayProps {
  if (!live) return p;
  return { ...p, occupied: live.occupied, confidence: live.confidence, updated_at: live.updated_at };
}

export function BayMap({
  fc,
  live,
  userPos,
  focusKey,
  target,
  route,
}: {
  fc: BayFC;
  live: Map<string, LiveState>;
  userPos: UserPos | null;
  focusKey: number;
  target: NearestTarget | null;
  route: RouteResult | null;
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
      {rings.map((r, i) => {
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
          pathOptions={{ color: TELEMETRY, weight: 3, dashArray: "2 8", lineCap: "round" }}
        />
      )}

      {/* the chosen nearest free bay */}
      {target && (
        <CircleMarker
          center={[target.lat, target.lon]}
          radius={10}
          pathOptions={{ color: TELEMETRY, fillColor: COLOR.free, fillOpacity: 1, weight: 3 }}
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
            pathOptions={{ color: TELEMETRY, weight: 1, fillOpacity: 0.08 }}
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
      <dl>
        <dt>Street</dt><dd>{props.street ?? "—"}</dd>
        <dt>Zone</dt><dd>{props.zona ?? "—"}</dd>
        {props.confidence != null && (<><dt>Confidence</dt><dd>{Math.round(props.confidence * 100)}%</dd></>)}
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
