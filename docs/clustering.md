# Obstacle Clustering

## Status

Not yet implemented. Planned for **Phase 5** (`perception/src/clustering/`).

Will document the clustering approach (DBSCAN as the initial implementation, evaluated against
simple Euclidean/distance-based segmentation), parameters (`eps`, `min_samples`, distance
thresholds), and the per-cluster features produced (point count, centroid, width, depth, minimum
distance, angular width, bounding box, shape features feeding `models.objects.DetectedObject`).
