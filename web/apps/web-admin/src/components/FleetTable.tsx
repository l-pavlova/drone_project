import { age, count, missionTone } from "../lib/format";
import type { FleetMetrics } from "../lib/types";
import styles from "./FleetTable.module.css";

/** Open missions: who is flying, how far in, and how long since their last
 *  frame. `frames_expected` is what the drone declared at mission start, so the
 *  progress bar is a plan-vs-actual, not a guess. */
export function FleetTable({ fleet }: { fleet: FleetMetrics }) {
  if (fleet.missions.length === 0) {
    return (
      <p className={styles.empty}>
        No open missions. {count(fleet.drones_total)} drone
        {fleet.drones_total === 1 ? "" : "s"} registered.
      </p>
    );
  }

  return (
    <div className={styles.scroll}>
      <table className={styles.table}>
        <thead>
          <tr>
            <th>Drone</th>
            <th>Survey area</th>
            <th>Progress</th>
            <th className={styles.right}>Frames</th>
            <th className={styles.right}>Last frame</th>
          </tr>
        </thead>
        <tbody>
          {fleet.missions.map((m) => {
            const tone = missionTone(m.since_last_frame_s);
            const pct =
              m.frames_expected && m.frames_expected > 0
                ? Math.min(100, (m.frames_done / m.frames_expected) * 100)
                : null;
            return (
              <tr key={m.mission_id}>
                <td>
                  <span className={`${styles.dot} ${styles[tone]}`} aria-hidden="true" />
                  {m.drone_id}
                </td>
                <td className={styles.dim}>{m.survey_area}</td>
                <td>
                  {pct == null ? (
                    <span className={styles.dim}>no target</span>
                  ) : (
                    <span className={styles.track}>
                      <span className={styles.fill} style={{ width: `${pct}%` }} />
                    </span>
                  )}
                </td>
                <td className={styles.right}>
                  {count(m.frames_done)}
                  {m.frames_expected != null && (
                    <span className={styles.dim}> / {count(m.frames_expected)}</span>
                  )}
                </td>
                <td className={`${styles.right} ${tone === "warn" ? styles.warnText : ""}`}>
                  {m.since_last_frame_s == null ? "none yet" : `${age(m.since_last_frame_s)} ago`}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
