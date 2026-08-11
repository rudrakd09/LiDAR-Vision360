# Perception Engine

## Status

Foundation (Phase 0), Preprocessing (Phase 3), Coordinate Transformation (Phase 4), Obstacle
Clustering (Phase 5), and Geometric Object Classification (Phase 6) implemented. Tracking through
clearance (Phases 7-10) are not implemented yet.

| Stage | Package | Status |
|---|---|---|
| Data models, config, logging, `LiDARDataSource` | `models`, `common`, `datasources` | ✅ Phase 0 |
| Preprocessing (validation, range filtering, outlier detection, noise/temporal filtering) | `preprocessing` | ✅ Phase 3 -- see [preprocessing.md](preprocessing.md) |
| Coordinate transformation (polar → Cartesian) | `coordinates` | ✅ Phase 4 -- see [coordinates.md](coordinates.md) |
| Obstacle clustering (DBSCAN) | `clustering` | ✅ Phase 5 -- see [clustering.md](clustering.md) |
| Geometric shape classification | `objects` | ✅ Phase 6 -- see [object-classification.md](object-classification.md) |
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
  GeometricClassifier.classify()     (perception.objects, Phase 6)
        |
    ClassifiedScan                   (labeled objects; models.classification.ClassifiedScan)
        |
  (Phase 7) tracking -- not implemented yet
```

See [data-model.md](data-model.md) for the full field-level model reference,
[preprocessing.md](preprocessing.md) for the Phase 3 write-up,
[coordinates.md](coordinates.md) for the Phase 4 write-up,
[clustering.md](clustering.md) for the Phase 5 write-up, and
[object-classification.md](object-classification.md) for the Phase 6 write-up (features, rule
scoring, thresholds, confidence, explainability, evaluation metrics, known failure cases).

## Design principle carried through every stage

Each stage is independent of both `simulator` (so it runs unchanged against real hardware once
Phase 16 lands) and of every *later* stage (so, e.g., preprocessing has zero awareness that
clustering or Unity will ever exist). Cross-package integration tests that need both a data
producer and a consumer (e.g. "run preprocessing against simulator scenarios") live in whichever
package already depends on the other -- see [testing.md](testing.md) and
[architecture.md](architecture.md).
