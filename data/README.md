# Sofia parking data

Source: **Sofiaplan open-data API** — `GET https://api.sofiaplan.bg/datasets/:id` → GeoJSON,
**WGS84 / EPSG:4326** (lon, lat). The main `urbandata.sofia.bg` portal only has GTFS + Park&Ride.
Download note (Windows): `curl --ssl-no-revoke`.

## Files here

| File | Dataset id | What | Features | Geometry |
|------|-----------|------|----------|----------|
| `zones_34.geojson` | 34 (2020) | Blue/green zone boundaries + subzones | 48 | MultiPolygon |
| `spaces_25.geojson` | 25 (2020-09) | Individual parking spaces (the spot registry) | 31,705 | MultiPoint (1 pt each) |

Other ids on the API (not downloaded): zones 470 (2016), 291 (2019); spaces 181 (2019), 274 (2016).

## `zones_34.geojson` — zone polygons (for route planning / which streets are payable)

Props: `razshireni` = zone type (`"Зелена зона"` green ×34, `"Синя зона"` blue ×5, null ×9),
`podzoni_20` = subzone name (e.g. "Подзона 19"), `num_2018`, `shape_leng`, `shape_area`.

## `spaces_25.geojson` — parking-space registry (the occupancy targets)

Each feature is a **point** (bay centroid) in central Sofia (bbox lon 23.238–23.514,
lat 42.630–42.748). Key props:

- `zona` — **Зелена зона** (green) 26,008 · **Синя зона** (blue) 5,134 · **Извън зона** (out-of-zone) 563
- `park_txt` — bay orientation: **Надлъжн** parallel 22,656 · **Напречн** perpendicular 6,304 · **Косо** angled 2,736
- `vid_txt_20` — space type: **Зона** public 28,005 · **Служебен** reserved 2,029 · **Инвалид** disabled 1,125 ·
  **ПИМ** 333 · **Такси** taxi 132 · **Дипломатически** 80
- `mestopoloz` — street name (e.g. "ул. Юри Венелин")
- `georef_201` — georeferencing year · `sf_rayon` — district · `id`, `objectid`

## How this slots into the pipeline

- This is the **pre-mapped spot map** → detector classifies occupied/free **against known spots**,
  not live geometry invention. Match a detected car's ground position to the nearest space point.
- Points have no footprint → synthesize a bay rectangle (~2.5×5 m) **oriented by `park_txt`** for
  matching and for drawing bays in the Webots world.
- For "free blue-zone spots" filter `zona == "Синя зона"` (and/or green); exclude non-public
  `vid_txt_20` (Инвалид/Служебен/Такси/Дипломатически) from general availability or flag separately.
- Pick a small dense central area (subset by bbox) for the demo/sim (tens–hundreds of spaces).

## Caveats

- **Currency:** 2020 (spaces) / 2016–2020 (zones); blue zone has expanded since → verify demo
  area against current signage.
- Some fields are "-" placeholders.
- **License:** confirm the Sofiaplan open-data license/attribution before publishing built results.
