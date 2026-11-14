# Held-out raw-scan localization stability

**Internal evaluation only. No independent pose ground truth or absolute accuracy measurement.**

Trajectory: `outputs\fin301\result\trajectory.csv`. Frozen protocol SHA-256: `d31faa51f300366ac228352aa90c90151b886a1651b43e7122355dbb90e2e57a`.

Reference representation: `all_returns`; parameters: `{'voxel_m': 0.07}`.

Queries are raw scans excluded from every odometry pose. Reference points come only from baseline map-insertion scan IDs, with a five-second temporal exclusion and at least 0.5 m estimated sensor-origin separation. Queries and reference IDs are identical between methods; each method supplies its own map poses. Seeds remain the same baseline estimate plus deterministic perturbations; no deviation from that estimate is scored as truth.

| Internal measure | Separated reference map | Earlier-pass reference only |
|---|---:|---:|
| Queries | 100 | 20 |
| Accepted registrations | 712 | 124 |
| Initialization attempts | 800 | 160 |
| Acceptance fraction | 0.89 | 0.775 |
| All eight starts agree within 0.15 m / 3 deg | 0.71 | 0.55 |
| Held-out line residual (m): median / p95 | 0.014739530100239671 / 0.07096728620465693 | 0.02576940639352863 / 0.07485245267448802 |
| Per-query maximum pairwise translation (m): median / p95 | 9.28478245390941e-14 / 0.009100080037929375 | 3.0359405436528116e-10 / 0.02033529731630204 |
| Per-query maximum pairwise yaw (deg): median / p95 | 1.360581657329815e-12 / 0.17609705819182003 | 1.8296617888321099e-09 / 0.4089866316364907 |

Each query uses eight starts: four signed XY/yaw directions at 0.25 m / 5 deg and four at 0.5 m / 10 deg. Pose spread is measured only between successful solutions for the same scan. A stable case requires all eight registrations to be accepted and agree. Rejections remain in the denominator; residual statistics alone use accepted runs.

Earlier-pass tests use only references at least 30 seconds earlier, with an estimated intervening departure of at least 3 m. This is retrospective consistency against an estimated earlier map, not independent first-pass ground truth. No camera 3D landmarks are projected into the 2D reference map.

Selection: do not select a candidate solely for a lower residual or visually cleaner map. Require no material degradation in acceptance and multi-start agreement, together with improved spread or earlier-pass consistency; inspect multi-basin failures. Shared baseline seeds and baseline-based spatial selection are a declared prior and can favor that prior. External checks are still required before claiming absolute localization accuracy.

## Paired comparison

```json
{
  "queries_summary": {
    "acceptance_rate_difference": 0.006249999999999978,
    "all_starts_agree_fraction_difference": 0.039999999999999925,
    "heldout_point_to_line_rmse_m": {
      "baseline": {
        "count": 707,
        "median": 0.01858982963947788,
        "p95": 0.07798170360048734,
        "max": 0.09589793106164224
      },
      "candidate": {
        "count": 712,
        "median": 0.014739530100239671,
        "p95": 0.07096728620465693,
        "max": 0.0952094354168078
      }
    },
    "query_max_pairwise_translation_m": {
      "baseline": {
        "count": 91,
        "median": 1.1192026056946179e-11,
        "p95": 0.5284455780170724,
        "max": 1.443884326629834
      },
      "candidate": {
        "count": 92,
        "median": 9.28478245390941e-14,
        "p95": 0.009100080037929375,
        "max": 1.0460621388416806
      }
    },
    "query_max_pairwise_yaw_deg": {
      "baseline": {
        "count": 91,
        "median": 2.3571428678261027e-10,
        "p95": 3.445197189687724,
        "max": 10.37381118755997
      },
      "candidate": {
        "count": 92,
        "median": 1.360581657329815e-12,
        "p95": 0.17609705819182003,
        "max": 3.6300220484418384
      }
    }
  },
  "crosspass_queries_summary": {
    "acceptance_rate_difference": 0.0,
    "all_starts_agree_fraction_difference": 0.0,
    "heldout_point_to_line_rmse_m": {
      "baseline": {
        "count": 124,
        "median": 0.029933033960839565,
        "p95": 0.07485259746335264,
        "max": 0.07485259762374304
      },
      "candidate": {
        "count": 124,
        "median": 0.02576940639352863,
        "p95": 0.07485245267448802,
        "max": 0.07485245283491677
      }
    },
    "query_max_pairwise_translation_m": {
      "baseline": {
        "count": 17,
        "median": 9.35220338358857e-11,
        "p95": 1.7053753066384037e-07,
        "max": 3.17536159817957e-07
      },
      "candidate": {
        "count": 17,
        "median": 3.0359405436528116e-10,
        "p95": 0.02033529731630204,
        "max": 0.056466315092432925
      }
    },
    "query_max_pairwise_yaw_deg": {
      "baseline": {
        "count": 17,
        "median": 8.844287673546156e-10,
        "p95": 1.2246185178043118e-06,
        "max": 1.4129045414680377e-06
      },
      "candidate": {
        "count": 17,
        "median": 1.8296617888321099e-09,
        "p95": 0.4089866316364907,
        "max": 1.45658866763199
      }
    }
  },
  "selection_rule": "Prefer an experiment only if acceptance and eight-start agreement do not materially worsen and spread or earlier-pass consistency improves across the same queries. Lower fitting residual alone is insufficient; inspect ambiguous and rejected cases. These checks do not establish physical accuracy."
}
```
