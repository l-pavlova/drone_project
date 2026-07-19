import type { PoolClient } from "pg";
import type { BayId } from "@parkdrone/contracts";
import { getPool } from "./pool.js";

export interface Bbox {
  minLon: number;
  minLat: number;
  maxLon: number;
  maxLat: number;
}

export interface BayStateInput {
  bayId: BayId;
  occupied: boolean;
  confidence: number;
  lastFrame: number | null;
  source: "vision" | "manual";
}

/**
 * Thin data-access layer. All geospatial predicates push down into PostGIS
 * (ST_MakeEnvelope + the GiST index on bay.geom / bay.centroid), so bbox and
 * nearest-bay queries never materialize the full bay set in the app layer.
 */
export const bayRepo = {
  /** Bays (+ current state) as a GeoJSON FeatureCollection. Grey/unknown = no state row. */
  async featureCollection(opts: { bbox?: Bbox; zona?: string } = {}): Promise<unknown> {
    const env = opts.bbox
      ? `ST_MakeEnvelope(${sqlNum(opts.bbox.minLon)}, ${sqlNum(opts.bbox.minLat)}, ${sqlNum(
          opts.bbox.maxLon,
        )}, ${sqlNum(opts.bbox.maxLat)}, 4326)`
      : "NULL";
    const { rows } = await getPool().query(
      `SELECT json_build_object(
                'type', 'FeatureCollection',
                'features', COALESCE(json_agg(feat), '[]'::json)
              ) AS fc
         FROM (
           SELECT json_build_object(
                    'type', 'Feature',
                    'geometry', ST_AsGeoJSON(b.geom)::json,
                    'properties', json_build_object(
                      'bay_id', b.bay_id, 'zona', b.zona, 'street', b.street,
                      'park_txt', b.park_txt, 'bearing_deg', b.bearing_deg, 'public', b.public,
                      'occupied', s.occupied, 'confidence', s.confidence,
                      'last_frame', s.last_frame, 'updated_at', s.updated_at, 'source', s.source
                    )
                  ) AS feat
             FROM bay b
             LEFT JOIN bay_state s USING (bay_id)
            WHERE (${env} IS NULL OR b.geom && ${env})
              AND ($1::text IS NULL OR b.zona = $1)
         ) t`,
      [opts.zona ?? null],
    );
    return rows[0]?.fc ?? { type: "FeatureCollection", features: [] };
  },

  /** Free/occupied/unknown counts grouped by zone. */
  async summary(): Promise<Array<{ zona: string | null; free: number; occupied: number; unknown: number }>> {
    const { rows } = await getPool().query(
      `SELECT b.zona,
              COUNT(*) FILTER (WHERE s.occupied IS FALSE) AS free,
              COUNT(*) FILTER (WHERE s.occupied IS TRUE)  AS occupied,
              COUNT(*) FILTER (WHERE s.occupied IS NULL)  AS unknown
         FROM bay b LEFT JOIN bay_state s USING (bay_id)
        GROUP BY b.zona
        ORDER BY b.zona`,
    );
    return rows.map((r) => ({
      zona: r.zona,
      free: Number(r.free),
      occupied: Number(r.occupied),
      unknown: Number(r.unknown),
    }));
  },

  /** One bay + current state + recent observations (default 20). */
  async detail(bayId: BayId, historyLimit = 20): Promise<unknown | null> {
    const bay = await getPool().query(
      `SELECT b.bay_id, b.zona, b.street, b.park_txt, b.bearing_deg, b.public,
              ST_AsGeoJSON(b.geom)::json AS geometry,
              s.occupied, s.confidence, s.last_frame, s.updated_at, s.source
         FROM bay b LEFT JOIN bay_state s USING (bay_id)
        WHERE b.bay_id = $1`,
      [bayId],
    );
    if (!bay.rowCount) return null;
    const history = await getPool().query(
      `SELECT occupied, frame_idx, world, votes_occupied, views, vis, gt, observed_at
         FROM observation WHERE bay_id = $1 ORDER BY observed_at DESC LIMIT $2`,
      [bayId, historyLimit],
    );
    return { ...bay.rows[0], history: history.rows };
  },

  /** Upsert current occupancy for a bay (used by manual admin override; the
   *  Python worker writes the same row directly). */
  async upsertState(input: BayStateInput, client?: PoolClient): Promise<void> {
    const q = client ?? getPool();
    await q.query(
      `INSERT INTO bay_state (bay_id, occupied, confidence, last_frame, source, updated_at)
       VALUES ($1, $2, $3, $4, $5, now())
       ON CONFLICT (bay_id) DO UPDATE SET
         occupied = EXCLUDED.occupied,
         confidence = EXCLUDED.confidence,
         last_frame = EXCLUDED.last_frame,
         source = EXCLUDED.source,
         updated_at = now()`,
      [input.bayId, input.occupied, input.confidence, input.lastFrame, input.source],
    );
  },
};

function sqlNum(n: number): string {
  if (!Number.isFinite(n)) throw new Error("bbox must be finite numbers");
  return n.toString();
}
