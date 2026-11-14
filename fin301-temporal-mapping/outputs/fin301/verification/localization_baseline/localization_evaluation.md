# Held-out raw-scan localization stability

**Internal evaluation only. No independent pose ground truth or absolute accuracy measurement.**

Trajectory: `outputs\fin301\baseline\trajectory.csv`. Frozen protocol SHA-256: `d31faa51f300366ac228352aa90c90151b886a1651b43e7122355dbb90e2e57a`.

Reference representation: `all_returns`; parameters: `{'voxel_m': 0.07}`.

Queries are raw scans excluded from every odometry pose. Reference points come only from baseline map-insertion scan IDs, with a five-second temporal exclusion and at least 0.5 m estimated sensor-origin separation. Queries and reference IDs are identical between methods; each method supplies its own map poses. Seeds remain the same baseline estimate plus deterministic perturbations; no deviation from that estimate is scored as truth.

| Internal measure | Separated reference map | Earlier-pass reference only |
|---|---:|---:|
| Queries | 100 | 20 |
| Accepted registrations | 707 | 124 |
| Initialization attempts | 800 | 160 |
| Acceptance fraction | 0.88375 | 0.775 |
| All eight starts agree within 0.15 m / 3 deg | 0.67 | 0.55 |
| Held-out line residual (m): median / p95 | 0.01858982963947788 / 0.07798170360048734 | 0.029933033960839565 / 0.07485259746335264 |
| Per-query maximum pairwise translation (m): median / p95 | 1.1192026056946179e-11 / 0.5284455780170724 | 9.35220338358857e-11 / 1.7053753066384037e-07 |
| Per-query maximum pairwise yaw (deg): median / p95 | 2.3571428678261027e-10 / 3.445197189687724 | 8.844287673546156e-10 / 1.2246185178043118e-06 |

Each query uses eight starts: four signed XY/yaw directions at 0.25 m / 5 deg and four at 0.5 m / 10 deg. Pose spread is measured only between successful solutions for the same scan. A stable case requires all eight registrations to be accepted and agree. Rejections remain in the denominator; residual statistics alone use accepted runs.

Earlier-pass tests use only references at least 30 seconds earlier, with an estimated intervening departure of at least 3 m. This is retrospective consistency against an estimated earlier map, not independent first-pass ground truth. No camera 3D landmarks are projected into the 2D reference map.

Selection: do not select a candidate solely for a lower residual or visually cleaner map. Require no material degradation in acceptance and multi-start agreement, together with improved spread or earlier-pass consistency; inspect multi-basin failures. Shared baseline seeds and baseline-based spatial selection are a declared prior and can favor that prior. External checks are still required before claiming absolute localization accuracy.
