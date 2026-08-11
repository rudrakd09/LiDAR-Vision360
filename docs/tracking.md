# Object Tracking

## Status

Not yet implemented. Planned for **Phase 7** (`perception/src/tracking/`).

Will document the frame-to-frame association strategy (starting with nearest-neighbour/centroid
matching, then a Kalman filter for position/velocity/direction estimation), how disappearing and
reappearing objects are handled, and how `track_id`/`velocity`/`direction` on
`models.objects.DetectedObject` are populated.
