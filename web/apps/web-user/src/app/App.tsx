import { useEffect, useMemo, useState } from "react";
import { fetchBays, fetchSummary } from "../api/client";
import { BayMap } from "../components/BayMap";
import { CarIcon, LocateIcon, Spinner } from "../components/Icons";
import { bayStatus, type BayFC, type BayProps, type ZoneSummary } from "../lib/types";
import { FMI_DEFAULT, getPosition, nearestFree, type NearestTarget, type UserPos } from "../lib/geo";
import { useOccupancySocket } from "../hooks/useOccupancySocket";

export default function App() {
  const [fc, setFc] = useState<BayFC | null>(null);
  const [zones, setZones] = useState<ZoneSummary[]>([]);
  const [error, setError] = useState<string | null>(null);
  const { live, connected } = useOccupancySocket();

  const [userPos, setUserPos] = useState<UserPos | null>(FMI_DEFAULT);
  const [focusKey, setFocusKey] = useState(0);
  const [target, setTarget] = useState<NearestTarget | null>(null);
  const [busy, setBusy] = useState(false);
  const [geoMsg, setGeoMsg] = useState<string | null>(null);

  useEffect(() => {
    fetchBays().then(setFc).catch((e) => setError(String(e)));
    fetchSummary().then((s) => setZones(s.zones)).catch(() => {});
  }, []);

  const statusOf = useMemo(() => {
    return (p: BayProps) => {
      const l = live.get(p.bay_id);
      const merged = l ? { ...p, occupied: l.occupied, updated_at: l.updated_at } : p;
      return bayStatus(merged);
    };
  }, [live]);

  const counts = useMemo(() => {
    const c = { free: 0, occupied: 0, unknown: 0 };
    if (fc) for (const f of fc.features) c[statusOf(f.properties)]++;
    return c;
  }, [fc, statusOf]);
  const surveyTotal = counts.free + counts.occupied + counts.unknown;

  async function locateMe(): Promise<UserPos | null> {
    setGeoMsg(null);
    try {
      const pos = await getPosition();
      setUserPos(pos);
      setFocusKey((k) => k + 1);
      return pos;
    } catch {
      // fall back to the FMI block so the demo still works off-site
      setUserPos(FMI_DEFAULT);
      setFocusKey((k) => k + 1);
      setGeoMsg("Location unavailable — using FMI block.");
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
      setGeoMsg(t ? null : "No free spaces found nearby.");
      setTarget(t);
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="app">
      {fc ? (
        <BayMap fc={fc} live={live} userPos={userPos} focusKey={focusKey} target={target} />
      ) : (
        <div className="loading">Loading map…</div>
      )}

      {/* survey readout — the ground-station panel */}
      <section className="readout" aria-label="Parking availability">
        <div className="eyebrow">
          <span className="wordmark">PARKDRONE</span>
          <span className={`status ${connected ? "live" : "idle"}`}>
            <i className="beacon" />
            {connected ? "SURVEYING" : "IDLE"}
          </span>
        </div>
        <div className="locus">SOFIA · {zones[0]?.zona ?? "—"}</div>

        <div className="figure">{counts.free}</div>
        <div className="figure-label">free spaces</div>

        <div
          className="meter"
          role="img"
          aria-label={`${counts.free} free, ${counts.occupied} occupied`}
        >
          <span className="seg free" style={{ flexGrow: counts.free || 0.001 }} />
          <span className="seg occ" style={{ flexGrow: counts.occupied || 0.001 }} />
        </div>
        <div className="meter-legend">
          <span>
            <b>{counts.free}</b> free
          </span>
          <span>
            <b>{counts.occupied}</b> occupied
          </span>
        </div>

        <div className="coverage">
          COVERAGE <b>{counts.free + counts.occupied}</b> / {surveyTotal} BAYS
        </div>
      </section>

      {/* toasts */}
      {error && <div className="toast warn">Failed to load bays.</div>}
      {geoMsg && <div className="toast warn">{geoMsg}</div>}
      {target && !geoMsg && (
        <div className="toast ok">
          Nearest free bay <b>{target.bayId}</b> · {Math.round(target.distance)} m away
          <button className="toast-x" onClick={() => setTarget(null)}>
            ✕
          </button>
        </div>
      )}

      {/* Google-Maps-style FAB controls, bottom-right */}
      <div className="fab-stack">
        <button className="fab" title="Locate me" onClick={() => void locateMe()}>
          <LocateIcon />
        </button>
        <button
          className="fab car"
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
