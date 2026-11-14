# Temporal-subset localization review

The same 100 raw query scans, eight initializations, source preprocessing and registration gates are used for every exact exported PCD. This is full-map repeat-observation stability, not a temporally separated test or an absolute-accuracy measurement. Temporal filtering uses accepted scan observations beyond the original map insertion subset; its evidence can be close in time to a query even when raw query IDs are excluded. See the independent temporal-evidence audit for the actual nearest-query timing.

| Artifact | Points | Accepted /800 | All-eight stable /100 | Conditional spread p95 (m) | Eligible spread queries |
|---|---:|---:|---:|---:|---:|
| original | 49783 | 283 | 11 | 0.9861187894802192 | 40 |
| occupancy_supported | 30165 | 332 | 29 | 0.04200981642031981 | 45 |
| temporal_conservative | 47679 | 286 | 16 | 1.0274677771370655 | 40 |
| temporal_static | 39614 | 271 | 18 | 0.019428822935786686 | 38 |

The filter uses 3526 accepted scans in 296 epochs. All 100 queries are excluded by raw ID, but each has a filter-evidence scan within 0.06812 s (median 0.05040 s). These observations are strongly correlated.

Stability requires all eight registrations accepted and agreeing within 0.15 m / 3 degrees. Failures remain in the denominator. Spread is conditional on at least two accepted starts; the eight starts on a scan are correlated trials.

| Candidate / comparison | Accepted gains / losses | Stable-query gains / losses | Newly zero-success scan IDs |
|---|---:|---:|---|
| temporal_conservative / against_original | 26 / 23 | 6 / 1 | [2314, 5163, 5341, 6408, 6587, 10681] |
| temporal_conservative / against_occupancy_supported | 62 / 108 | 3 / 16 | [7298, 7654, 7833, 9079, 9257, 9434, 9613, 9791, 9969, 10147, 10503, 10681] |
| temporal_static / against_original | 36 / 48 | 9 / 2 | [2314, 5163, 5341, 6408, 6587, 6943, 8723, 14953] |
| temporal_static / against_occupancy_supported | 60 / 121 | 5 / 16 | [6943, 7298, 7654, 7833, 9079, 9257, 9434, 9613, 9791, 9969, 10147, 10503, 13173, 14953] |

These two controls answer different questions: original measures the change from the unfiltered map; occupancy-supported measures the change from the previously available cleaned map. A cleaner static-inspection artifact may lose localization support. Removal due to insufficient temporal evidence does not establish free space, and stationary people or long-lived movable objects can remain. This test uses only exact LiDAR PCD targets; it does not validate the matched occupancy grid or free-space safety. Raw query scans may contain transient returns, so lower acceptance does not by itself establish that removed geometry was static or that the map is wrong. Camera landmarks do not enter these localization targets. See the filter report for evidence categories and their distinct meanings.

Per-query trials, paired IDs, actual PCD hashes and complete evaluation provenance remain in each candidate directory. Neither localization controls nor primary outputs are overwritten.
