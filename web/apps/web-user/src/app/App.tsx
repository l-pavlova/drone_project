import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  fetchBays,
  fetchCurbRuns,
  fetchNoFly,
  fetchServerInfo,
  fetchSummary,
} from "../api/client";
import { BayMap } from "../components/BayMap";
import { CarIcon, LocateIcon, NavigateIcon, Spinner } from "../components/Icons";
import { SurveyReadout } from "../components/SurveyReadout";
import {
  bayStatus,
  setFreshnessWindow,
  type BayFC,
  type BayFeature,
  type BayProps,
  type CurbRunFC,
  type NoFlyFC,
  type Restriction,
  type ZoneSummary,
} from "../lib/types";
import {
  directionsUrl,
  FMI_DEFAULT,
  formatDistance,
  formatDuration,
  getPosition,
  nearestFree,
  watchPosition,
  type NearestTarget,
  type UserPos,
} from "../lib/geo";
import { useOccupancySocket } from "../hooks/useOccupancySocket";
import { useLiveRoute } from "../hooks/useLiveRoute";
import styles from "./App.module.css";

const FLASH_MS = 5000;

// Airspace is fetched once for a box around the block rather than per viewport:
// it is static reference data (the CAA republishes it, the drone does not), so
// re-fetching on pan would only re-download the same polygons. ~6 km covers the
// whole surveyable area and anywhere a driver would realistically pan to.
const AIRSPACE_BBOX = "23.262,42.625,23.398,42.725";

// Shown by default: PROHIBITED is the only class that actually stops the survey.
// The lighter classes come down the same request and are styled to sit back, so
// the switch is one control rather than three.
const ZONES_SHOWN = new Set<Restriction>([
  "PROHIBITED",
  "REQ_AUTHORISATION",
  "CONDITIONAL",
]);
const ZONES_HIDDEN = new Set<Restriction>();

export default function App() {
  const [fc, setFc] = useState<BayFC | null>(null);
  const [zones, setZones] = useState<ZoneSummary[]>([]);
  const [nofly, setNofly] = useState<NoFlyFC | null>(null);
  const [zonesOn, setZonesOn] = useState(true);
  const [runs, setRuns] = useState<CurbRunFC | null>(null);
  const [runsOn, setRunsOn] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const { live, connected, syncVersion } = useOccupancySocket();

  const [userPos, setUserPos] = useState<UserPos | null>(FMI_DEFAULT);
  const [focusKey, setFocusKey] = useState(0);
  const [target, setTarget] = useState<NearestTarget | null>(null);
  const [busy, setBusy] = useState(false);
  const [geoMsg, setGeoMsg] = useState<string | null>(null);
  const [following, setFollowing] = useState(false);

  const { route, status: routeStatus } = useLiveRoute(userPos, target);
  const navigating = target !== null;

  useEffect(() => {
    // Adopt the server's occupancy window BEFORE the bays land, so the first
    // paint already uses the same staleness rule the vote does. A failure
    // leaves the 2 h fallback, which is the server default.
    fetchServerInfo()
      .then((i) => setFreshnessWindow(i.occupancy_window_s))
      .catch(() => {})
      .finally(() => {
        fetchBays().then(setFc).catch((e) => setError(String(e)));
        fetchSummary().then((s) => setZones(s.zones)).catch(() => {});
      });
    // Reference data, so a failure here is not worth a toast — the map is still
    // fully usable without the airspace overlay, and the switch stays hidden.
    fetchNoFly(AIRSPACE_BBOX).then(setNofly).catch(() => {});
  }, []);

  // Curb runs are POLLED, not pushed. The delta socket carries bay_id payloads;
  // widening it to runs means a second delta table and a listener change, which
  // is a lot of machinery for a layer that only changes as fast as a drone
  // flies. A failure is swallowed like the airspace one — the bay map is still
  // fully usable and the switch simply stays hidden.
  useEffect(() => {
    let alive = true;
    const pull = () =>
      fetchCurbRuns(AIRSPACE_BBOX)
        .then((r) => alive && setRuns(r))
        .catch(() => {});
    pull();
    const t = window.setInterval(pull, 3000);
    return () => {
      alive = false;
      window.clearInterval(t);
    };
  }, []);

  // The initial REST snapshot and opening the socket are separate requests.
  // Reconcile after the socket has replayed its missed events so a burst of
  // completed frames during page load/reconnect cannot wait for a browser
  // refresh to become visible. Live deltas keep winning while this fetch runs.
  useEffect(() => {
    if (!syncVersion) return;
    fetchBays().then(setFc).catch((e) => setError(String(e)));
    fetchSummary().then((s) => setZones(s.zones)).catch(() => {});
  }, [syncVersion]);

  // transient status line, so it can't linger over the route toast
  const flashTimer = useRef<number | null>(null);
  const flash = useCallback((msg: string) => {
    setGeoMsg(msg);
    if (flashTimer.current) window.clearTimeout(flashTimer.current);
    flashTimer.current = window.setTimeout(() => setGeoMsg(null), FLASH_MS);
  }, []);
  useEffect(
    () => () => {
      if (flashTimer.current) window.clearTimeout(flashTimer.current);
    },
    [],
  );

  const statusOf = useMemo(() => {
    return (p: BayProps) => {
      const l = live.get(p.bay_id);
      const merged = l ? { ...p, occupied: l.occupied, updated_at: l.updated_at } : p;
      return bayStatus(merged);
    };
  }, [live]);

  const counts = useMemo(() => {
    // Every BayStatus needs a slot here: the accumulator is indexed by the
    // status itself, so a missing key increments `undefined` and the tile shows
    // NaN rather than failing loudly.
    const c = { free: 0, occupied: 0, unknown: 0, closed: 0 };
    if (fc) for (const f of fc.features) c[statusOf(f.properties)]++;
    return c;
  }, [fc, statusOf]);
  const surveyTotal = counts.free + counts.occupied + counts.unknown + counts.closed;

  const airspaceCounts = useMemo(() => {
    if (!nofly) return null;
    let noFly = 0;
    for (const f of nofly.features) if (f.properties.restriction === "PROHIBITED") noFly++;
    return { noFly, restricted: nofly.features.length - noFly };
  }, [nofly]);

  const featureById = useMemo(() => {
    const m = new Map<string, BayFeature>();
    if (fc) for (const f of fc.features) m.set(f.properties.bay_id, f);
    return m;
  }, [fc]);

  // Follow the driver while navigating, so the route redraws as they move. The
  // watch is intentionally scoped to an active target — no background GPS.
  useEffect(() => {
    if (!navigating) {
      setFollowing(false);
      return;
    }
    const stop = watchPosition(
      (p) => {
        setUserPos(p);
        setFollowing(true);
      },
      // denied / unavailable: keep the last known position (may be FMI_DEFAULT)
      () => setFollowing(false),
    );
    return () => {
      stop();
      setFollowing(false);
    };
  }, [navigating]);

  // If the bay we're driving to gets taken, hop to the next nearest free one.
  // This is what the live occupancy push is for: the deltas arrive on the same
  // socket that colours the map, so a spot lost mid-drive re-routes by itself.
  useEffect(() => {
    if (!target || !fc) return;
    const f = featureById.get(target.bayId);
    if (f && statusOf(f.properties) === "free") return;

    const next = nearestFree(fc.features, statusOf, userPos ?? FMI_DEFAULT);
    if (next && next.bayId !== target.bayId) {
      setTarget(next);
      flash(`Bay ${target.bayId} taken — rerouting to ${next.bayId}.`);
    } else if (!next) {
      setTarget(null);
      flash("That spot was taken and there are no free bays left nearby.");
    }
  }, [live, fc, target, featureById, statusOf, userPos, flash]);

  async function locateMe(): Promise<UserPos | null> {
    try {
      const pos = await getPosition();
      setUserPos(pos);
      setFocusKey((k) => k + 1);
      return pos;
    } catch {
      // fall back to the FMI block so the demo still works off-site
      setUserPos(FMI_DEFAULT);
      setFocusKey((k) => k + 1);
      flash("Location unavailable — using FMI block.");
      return FMI_DEFAULT;
    }
  }

  async function findNearest(): Promise<void> {
    if (!fc) return;
    setBusy(true);
    try {
      const pos = userPos ?? (await locateMe());
      if (!pos) return;
      const t = nearestFree(fc.features, statusOf, pos);
      if (!t) flash("No free spaces found nearby.");
      setTarget(t);
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className={styles.app}>
      {fc ? (
        <BayMap
          fc={fc}
          live={live}
          userPos={userPos}
          focusKey={focusKey}
          target={target}
          route={route}
          nofly={nofly}
          showZones={zonesOn ? ZONES_SHOWN : ZONES_HIDDEN}
          runs={runs}
          showRuns={runsOn}
        />
      ) : (
        <div className={styles.loading}>Loading map…</div>
      )}

      <SurveyReadout
        connected={connected}
        zones={zones}
        counts={counts}
        surveyTotal={surveyTotal}
        kerbs={
          runs
            ? {
                free: runs.features.reduce((n, f) => n + (f.properties.free ?? 0), 0),
                cars: runs.features.reduce((n, f) => n + (f.properties.cars ?? 0), 0),
                on: runsOn,
                toggle: () => setRunsOn((v) => !v),
              }
            : null
        }
        airspace={
          airspaceCounts
            ? { ...airspaceCounts, on: zonesOn, toggle: () => setZonesOn((v) => !v) }
            : null
        }
      />

      {/* toasts, bottom-centre: transient status stacks above the route bar */}
      <div className={styles.toastStack}>
        {error && <div className={`${styles.toast} ${styles.warn}`}>Failed to load bays.</div>}
        {geoMsg && <div className={`${styles.toast} ${styles.warn}`}>{geoMsg}</div>}
        {target && (
          <div className={`${styles.toast} ${styles.ok}`}>
            {route ? (
              <>
                <span className={styles.tag}>ROUTE</span>
                <b>{formatDistance(route.distance_m)}</b>
                <span className={styles.sep}>·</span>
                <span>{formatDuration(route.duration_s)}</span>
                <span className={styles.sep}>·</span>
                <span>bay {target.bayId}</span>
              </>
            ) : routeStatus === "loading" ? (
              <>
                <Spinner />
                <span>routing to bay {target.bayId}…</span>
              </>
            ) : (
              <>
                {/* router unavailable: honest about it being the straight line */}
                <span className={`${styles.tag} ${styles.tagDim}`}>DIRECT</span>
                <b>{formatDistance(target.distance)}</b>
                <span className={styles.sep}>·</span>
                <span>bay {target.bayId}</span>
              </>
            )}
            {following && <span className={styles.liveDot} title="following your location" />}
            <a
              className={styles.navBtn}
              href={directionsUrl(userPos, target)}
              target="_blank"
              rel="noreferrer"
            >
              <NavigateIcon />
              Navigate
            </a>
            <button className={styles.toastX} title="Cancel" onClick={() => setTarget(null)}>
              ✕
            </button>
          </div>
        )}
      </div>

      {/* Google-Maps-style FAB controls, bottom-right */}
      <div className={styles.fabStack}>
        <button className={styles.fab} title="Locate me" onClick={() => void locateMe()}>
          <LocateIcon />
        </button>
        <button
          className={`${styles.fab} ${styles.car}`}
          title="Find nearest free spot"
          disabled={busy || !fc}
          onClick={() => void findNearest()}
        >
          {busy ? <Spinner /> : <CarIcon />}
        </button>
      </div>
    </div>
  );
}
