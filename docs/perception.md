# Perception Engine

## Status

Foundation (Phase 0), Preprocessing (Phase 3), Coordinate Transformation (Phase 4), and Obstacle
Clustering (Phase 5) implemented. Classification through clearance (Phases 6-10) are not
implemented yet.

| Stage | Package | Status |
|---|---|---|
| Data models, config, logging, `LiDARDataSource` | `models`, `common`, `datasources` | ✅ Phase 0 |
| Preprocessing (validation, range filtering, outlier detection, noise/temporal filtering) | `preprocessing` | ✅ Phase 3 -- see [preprocessing.md](preprocessing.md) |
| Coordinate transformation (polar → Cartesian) | `coordinates` | ✅ Phase 4 -- see [coordinates.md](coordinates.md) |
| Obstacle clustering (DBSCAN) | `clustering` | ✅ Phase 5 -- see [clustering.md](clustering.md) |
| Shape classification | `objects` | ⏳ Phase 6 |
| Object tracking (nearest-neighbour, Kalman) | `tracking` | ⏳ Phase 7 |
| Occupancy grid | `mapping` | ⏳ Phase 8 |
| Collision-risk / TTC | `collision` | ⏳ Phase 9 |
| Clearance analysis | `clearance` | ⏳ Phase 10 |
| End-to-end orchestration | `pipeline` | ⏳ assembled incrementally as the above land |

## Pipeline so far

```text
LiDARDataSource.read_scan()
        |
     ScanFrame                       (raw; models.scan.ScanFrame)
        |
  Preprocessor.process()             (perception.preprocessing, Phase 3)
        |
   PreprocessedScan                  (clean; models.preprocessing.PreprocessedScan)
        |
  CoordinateTransformer.transform()  (perception.coordinates, Phase 4)
        |
    CartesianScan                    (x/y; models.coordinates.CartesianScan)
        |
  DBSCANClusterer.cluster()          (perception.clustering, Phase 5)
        |
    ClusteredScan                    (obstacle candidates; models.clustering.ClusteredScan)
        |
  (Phase 6) shape classification -- not implemented yet
```

See [data-model.md](data-model.md) for the full field-level model reference,
[preprocessing.md](preprocessing.md) for the Phase 3 write-up,
[coordinates.md](coordinates.md) for the Phase 4 write-up, and
[clustering.md](clustering.md) for the Phase 5 write-up (DBSCAN, parameter selection, cluster
representation, noise/free-space handling, 0/360 handling, performance, usage examples).

## Design principle carried through every stage

Each stage is independent of both `simulator` (so it runs unchanged against real hardware once
Phase 16 lands) and of every *later* stage (so, e.g., preprocessing has zero awareness that
clustering or Unity will ever exist). Cross-package integration tests that need both a data
producer and a consumer (e.g. "run preprocessing against simulator scenarios") live in whichever
package already depends on the other -- see [testing.md](testing.md) and
[architecture.md](architecture.md).
