# Perception Engine

## Status

Foundation (Phase 0), Preprocessing (Phase 3), Coordinate Transformation (Phase 4), Obstacle
Clustering (Phase 5), Geometric Object Classification (Phase 6), Object Tracking (Phase 7),
2D Occupancy Grid Mapping (Phase 8), and Collision/Risk Engine (Phase 9) implemented. Clearance
engine through hardware integration (Phases 10, 16) are not implemented yet.

| Stage | Package | Status |
|---|---|---|
| Data models, config, logging, `LiDARDataSource` | `models`, `common`, `datasources` | ✅ Phase 0 |
| Preprocessing (validation, range filtering, outlier detection, noise/temporal filtering) | `preprocessing` | ✅ Phase 3 -- see [preprocessing.md](preprocessing.md) |
| Coordinate transformation (polar → Cartesian) | `coordinates` | ✅ Phase 4 -- see [coordinates.md](coordinates.md) |
| Obstacle clustering (DBSCAN) | `clustering` | ✅ Phase 5 -- see [clustering.md](clustering.md) |
| Geometric shape classification | `objects` | ✅ Phase 6 -- see [object-classification.md](object-classification.md) |
| Object tracking (nearest-neighbour, Kalman) | `tracking` | ✅ Phase 7 -- see [tracking.md](tracking.md) |
| 2D occupancy grid mapping | `mapping` | ✅ Phase 8 -- see [mapping.md](mapping.md) |
| Collision/risk engine (TTC, SAFE/WARNING/CRITICAL) | `collision` | ✅ Phase 9 -- see [collision.md](collision.md) |
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
  ObjectTracker.update()             (perception.tracking, Phase 7)
        |
    TrackedScan                      (persistent IDs + velocity; models.tracking.TrackedScan)
        |
  (Phase 9) collision engine -- not implemented yet

  (in parallel with clustering onward -- see mapping.md "Architecture"):
    CartesianScan
        |
  OccupancyGridMapper.update()       (perception.mapping, Phase 8)
        |
    OccupancyGrid                    (persistent FREE/OCCUPIED/UNKNOWN map; models.mapping.OccupancyGrid)

  CollisionRiskEngine.evaluate()     (perception.collision, Phase 9; consumes TrackedScan + VehicleState)
        |
    CollisionAssessment              (per-object SAFE/WARNING/CRITICAL risk; models.collision.CollisionAssessment)
```

**Mapping (Phase 8) consumes `CartesianScan` directly**, not `TrackedScan` -- it is a parallel
consumer of Phase 4's output alongside clustering, not a strictly-linear next stage after
tracking, even though it is documented/numbered as "Phase 8" and typically run after tracking in
end-to-end pipeline scripts. See [mapping.md](mapping.md) "Architecture" for why.

**Collision (Phase 9) consumes `TrackedScan` directly**, resuming the linear chain after tracking
-- its primary inputs are tracked-object position/velocity/geometry, vehicle geometry, and vehicle
state, deliberately **not** the occupancy grid (Phase 8), which remains independent, optional
supporting information for a future phase. See [collision.md](collision.md) "Architecture".

See [data-model.md](data-model.md) for the full field-level model reference,
[preprocessing.md](preprocessing.md) for the Phase 3 write-up,
[coordinates.md](coordinates.md) for the Phase 4 write-up,
[clustering.md](clustering.md) for the Phase 5 write-up,
[object-classification.md](object-classification.md) for the Phase 6 write-up (features, rule
scoring, thresholds, confidence, explainability, evaluation metrics, known failure cases),
[tracking.md](tracking.md) for the Phase 7 write-up (association, track lifecycle, Kalman filter,
velocity/movement estimation, prediction, known limitations), [mapping.md](mapping.md) for the
Phase 8 write-up (occupancy grid, log-odds model, ray traversal, decay, ground-truth evaluation,
known limitations -- **this is a 2D LiDAR occupancy map, not a true 3D map**), and
[collision.md](collision.md) for the Phase 9 write-up (vehicle model, safety zones, projected
path, TTC, collision prediction, risk classification, known limitations -- **this is a prototype
collision-awareness system, not a certified automotive safety system**).

## Design principle carried through every stage

Each stage is independent of both `simulator` (so it runs unchanged against real hardware once
Phase 16 lands) and of every *later* stage (so, e.g., preprocessing has zero awareness that
clustering or Unity will ever exist). Cross-package integration tests that need both a data
producer and a consumer (e.g. "run preprocessing against simulator scenarios") live in whichever
package already depends on the other -- see [testing.md](testing.md) and
[architecture.md](architecture.md).
