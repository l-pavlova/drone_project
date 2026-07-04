# Терминологичен речник / Terminology glossary

*Companion to `documentation.md`. Fixes the Bulgarian term for every concept in
the thesis ONCE, so the text stays consistent and natural. Grows together with
the documentation; the thesis chapters will be held to this glossary.*

**Usage rules**

1. At first mention in the thesis use the Bulgarian term with the English in
   parentheses: „задача за китайския пощальон (Chinese Postman Problem)“ —
   after that, Bulgarian only.
2. Product names, file formats, protocol names and code identifiers stay in
   English and are not inflected: Webots, GeoJSON, MAVLink, `route.json`,
   `AIM_BLEND`. Wrap code identifiers in `код`.
3. Terms marked ⚠ have a tempting literal translation that sounds wrong —
   use the fixed term.

---

## Общи / General

| English | Български | Бележки |
|---|---|---|
| thesis | дипломна работа | |
| pipeline | работен процес / верига за обработка | ⚠ не „пайплайн“ в академичен текст |
| dataset | набор от данни | |
| open-source | с отворен код | |
| ground truth | еталонни данни (ground truth) | ⚠ не „наземна истина“; при първа употреба с англ. в скоби |
| simulation | симулация | |
| prototype | прототип | |
| regression check | регресионна проверка | |
| verification | верификация / проверка | |
| deterministic | детерминиран | |
| random seed | начална стойност на генератора (seed) | |
| trade-off | компромис | ⚠ не „трейдоф“ |
| future work | насоки за бъдещо развитие | |

## ГИС и данни / GIS & data

| English | Български | Бележки |
|---|---|---|
| parking space | паркомясто | терминът от данните на Sofiaplan |
| parking bay (rectangle) | очертание на паркомясто | нашият ориентиран правоъгълник |
| parallel / perpendicular / angled parking | надлъжно / напречно / косо паркиране | точно като `park_txt` в данните |
| occupancy | заетост | |
| occupied / free | заето / свободно | |
| paid-parking zone | зона за платено паркиране (синя/зелена зона) | |
| street centerline | осева линия на улицата | |
| geocoding | геокодиране | |
| bounding box | ограждащ правоъгълник | |
| centroid | центроид | |
| coordinate frame | координатна система | |
| origin (of frame) | начало на координатната система | |
| projection | проекция | |
| ENU (East-North-Up) | локална координатна система „изток–север–нагоре“ (ENU) | |
| WGS84 / EPSG:4326 | WGS84 / EPSG:4326 | остава |
| GeoJSON, OSM, Overpass API | — | остават на английски |
| bearing | посока / азимут | за улици: „посока на улицата“ |
| PCA (principal component analysis) | метод на главните компоненти (PCA) | |
| clipping (Liang–Barsky) | изрязване (алгоритъм на Лианг–Барски) | |

## Симулация и свят / Simulation & world

| English | Български | Бележки |
|---|---|---|
| world (Webots) | свят | „светът fmi_block“ |
| window (study area) | прозорец / изследвана област | |
| scale | мащаб | |
| frame (image) | кадър | |
| camera footprint | наземно покритие на кадъра | ⚠ не „отпечатък“ |
| nadir | надир; надирна снимка | |
| gimbal | стабилизиращо окачване (гимбал) | „гимбал“ е приета чуждица |
| viewpoint / follow camera | гледна точка / следяща камера | |
| headless (run) | без графичен интерфейс (headless) | |
| painted bay | маркировка на паркомясто | |
| ground plane | земна равнина | |
| proto / PROTO | PROTO модел | остава |
| occupied fraction | дял на заетите места | |
| collision-aware placement | разполагане със следене за застъпване | |
| separating-axis test | тест с разделяща ос | |
| rear axle | задна ос | |

## Контролер и полет / Controller & flight

| English | Български | Бележки |
|---|---|---|
| controller | контролер | |
| control loop | цикъл на управление | |
| PD controller | ПД регулатор (пропорционално-диференциален) | |
| gain | коефициент на усилване | |
| damping | демпфиране | |
| roll / pitch / yaw | крен / тангаж / рискане | авиационната триада; при първа употреба с англ. в скоби |
| heading | курс | за посоката на носа; различно от „рискане“ (въртенето) |
| altitude | височина на полета | |
| altitude hold | задържане на височина | |
| vertical speed | вертикална скорост | |
| takeoff | излитане | |
| hover / station-keeping | задържане на позиция | |
| drift | дрейф | |
| cruise speed | крейсерска скорост | |
| tilt | наклон | |
| thrust | тяга | |
| propeller mixing | смесване на команди към витлата | |
| turn radius | радиус на завоя | |
| U-turn | обратен завой | |
| waypoint | пътна точка | |
| arrival basin | зона на пристигане | |
| closest approach | най-близко преминаване | |
| orbit (trap) | орбитиране (около пътна точка) | нашият „orbit trap“ |
| timeout | краен срок (timeout) | |
| pure pursuit | метод на преследване (pure pursuit) | |
| aim point / carrot | точка на прицелване | „морков“ само разговорно |
| lookahead | изпреварващо прицелване | |
| speed gate | ограничение на скоростта при отклонен курс | описателно; в кода `v_des` гейт |
| autopilot | автопилот | |
| companion computer | бордови компютър | |
| telemetry | телеметрия | |
| flight log | полетен дневник / лог на полета | |
| mission (MAVLink) | мисия | |
| EKF, RTK, GPS, IMU | — | остават абревиатури |

## Графи и маршрутизация / Graphs & routing

| English | Български | Бележки |
|---|---|---|
| graph | граф | |
| vertex / node | връх | |
| edge | ребро | |
| edge weight | тегло на ребро | |
| degree (of vertex) | степен на връх | |
| odd-degree vertex | връх от нечетна степен | |
| multigraph | мултиграф | |
| connected component | свързана компонента | |
| spanning tree | покриващо дърво | |
| minimum spanning tree (MST) | минимално покриващо дърво | |
| depth-first search (DFS) | обхождане в дълбочина | |
| backtracking (flight) | връщане по вече обходен участък | ⚠ различно от алгоритмичния „бектракинг“ |
| shortest path | най-кратък път | |
| Dijkstra's algorithm | алгоритъм на Дейкстра | |
| multi-source Dijkstra | алгоритъм на Дейкстра с множество източници | |
| Euler path / circuit | Ойлеров път / Ойлеров цикъл | |
| Eulerian graph | Ойлеров граф | |
| Hierholzer's algorithm | алгоритъм на Хирхолцер | |
| Route Inspection Problem | задача за инспекция на маршрути | |
| Chinese Postman Problem (CPP) | задача за китайския пощальон | |
| Rural Postman Problem (RPP) | задача за селския пощальон (Rural Postman Problem) | required/optional ребра |
| open path (vs closed tour) | отворен маршрут (срещу затворен обход) | |
| matching | съчетание | |
| minimum-weight perfect matching | перфектно съчетание с минимално тегло | |
| blossom algorithm | алгоритъм на Едмъндс (blossom) | |
| virtual vertex | виртуален връх | |
| coverage | покритие | |
| coverage graph | граф на покритието | |
| deadhead | празен пробег | реален транспортен термин — ползвай го |
| transit (leg) | преход | |
| route densification | сгъстяване на маршрута | пътни точки на всеки 10 м |
| lawnmower pattern | обхождане тип „косачка“ | резервната схема |
| greedy (algorithm) | алчен алгоритъм | утвърден термин |
| NP-hard | NP-трудна (задача) | |

## Компютърно зрение (предстоящо) / Vision (upcoming)

| English | Български | Бележки |
|---|---|---|
| computer vision | компютърно зрение | |
| object detection | откриване на обекти | |
| detector | детектор | |
| georeferencing | геопривързване | ГИС терминът; ⚠ не „геореференциране“ |
| pose (x, y, alt, yaw) | поза (позиция и ориентация) | |
| bounding box (detection) | ограждаща кутия | в контекст на детекция |
| training / fine-tuning | обучение / дообучаване | |
| inference | извод (inference) | |
| sim-to-real transfer | пренос от симулация към реална среда | |
| domain gap | разлика между симулирана и реална среда (domain gap) | описателно при първа употреба |
| accuracy / precision / recall | точност / прецизност / пълнота | утвърдените преводи |

---

*Незавършено: терминът за „bay“ извън паркинг контекста, преводите за
конкретни метрики на детектора (mAP, IoU) — ще се добавят със стартирането на
vision етапа.*
