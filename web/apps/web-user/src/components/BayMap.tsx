import { useEffect, useMemo } from "react";
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
import type { LatLngExpression } from "leaflet";
import type { BayFC, BayProps, BayStatus } from "../lib/types";
import { bayStatus } from "../lib/types";
import type { NearestTarget, UserPos } from "../lib/geo";
import type { LiveState } from "../hooks/useOccupancySocket";

const ORIGIN: LatLngExpression = [42.6747105, 23.3298956];

/** Imperatively drives the map: recenters on locate, frames user+target on route. */
function MapView({
  userPos,
  focusKey,
  target,
}: {
  userPos: UserPos | null;
  focusKey: number;
  target: NearestTarget | null;
}) {
  const map = useMap();

  useEffect(() => {
    if (userPos && focusKey > 0) {
      map.flyTo([userPos.lat, userPos.lon], Math.max(map.getZoom(), 17));
    }
  }, [focusKey]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!target) return;
    if (userPos) {
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
  }, [target]); // eslint-disable-line react-hooks/exhaustive-deps

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
}: {
  fc: BayFC;
  live: Map<string, LiveState>;
  userPos: UserPos | null;
  focusKey: number;
  target: NearestTarget | null;
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

  return (
    <MapContainer center={ORIGIN} zoom={17} className="map" preferCanvas zoomControl={false}>
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
              <BayPopup props={props} status={status} />
            </Popup>
          </Polygon>
        );
      })}

      {/* line from the user to the chosen free bay */}
      {userPos && target && (
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
          <Tooltip permanent direction="top" offset={[0, -8]} className="tele-tip">
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

      <MapView userPos={userPos} focusKey={focusKey} target={target} />
    </MapContainer>
  );
}

function BayPopup({ props, status }: { props: BayProps; status: BayStatus }) {
  return (
    <div className="popup">
      <strong>Bay {props.bay_id}</strong>
      <div className={`badge ${status}`}>{status.toUpperCase()}</div>
      <dl>
        <dt>Street</dt><dd>{props.street ?? "—"}</dd>
        <dt>Zone</dt><dd>{props.zona ?? "—"}</dd>
        {props.confidence != null && (<><dt>Confidence</dt><dd>{Math.round(props.confidence * 100)}%</dd></>)}
        <dt>Updated</dt>
        <dd>{props.updated_at ? new Date(props.updated_at).toLocaleTimeString() : "never surveyed"}</dd>
      </dl>
    </div>
  );
}
