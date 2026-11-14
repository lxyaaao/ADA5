# Held-out raw-scan localization stability

**Internal evaluation only. No independent pose ground truth or absolute accuracy measurement.**

Trajectory: `outputs\fin301\baseline\trajectory.csv`. Frozen protocol SHA-256: `d31faa51f300366ac228352aa90c90151b886a1651b43e7122355dbb90e2e57a`.

Queries are raw scans excluded from every odometry pose. Reference points come only from baseline map-insertion scan IDs, with a five-second temporal exclusion and at least 0.5 m estimated sensor-origin separation. Queries and reference IDs are identical between methods; each method supplies its own map poses. Seeds remain the same baseline estimate plus deterministic perturbations; no deviation from that estimate is scored as truth.

| Internal measure | Separated reference map | Earlier-pass reference only |
|---|---:|---:|
| Queries | 100 | 20 |
| Accepted registrations | 620 | 96 |
| Initialization attempts | 800 | 160 |
| Acceptance fraction | 0.775 | 0.6 |
| All eight starts agree within 0.15 m / 3 deg | 0.61 | 0.4 |
| Held-out line residual (m): median / p95 | 0.023795140569638645 / 0.08025412578385885 | 0.04527433332609078 / 0.09385472997230633 |
| Per-query maximum pairwise translation (m): median / p95 | 9.935092393758723e-11 / 0.8501119335280098 | 2.3397941483976237e-11 / 0.552511654737204 |
| Per-query maximum pairwise yaw (deg): median / p95 | 1.4988729700185191e-09 / 5.393937443970476 | 5.294709135165835e-10 / 0.0388354706090824 |

Each query uses eight starts: four signed XY/yaw directions at 0.25 m / 5 deg and four at 0.5 m / 10 deg. Pose spread is measured only between successful solutions for the same scan. A stable case requires all eight registrations to be accepted and agree. Rejections remain in the denominator; residual statistics alone use accepted runs.

Earlier-pass tests use only references at least 30 seconds earlier, with an estimated intervening departure of at least 3 m. This is retrospective consistency against an estimated earlier map, not independent first-pass ground truth. No camera 3D landmarks are projected into the 2D reference map.

Selection: do not select a candidate solely for a lower residual or visually cleaner map. Require no material degradation in acceptance and multi-start agreement, together with improved spread or earlier-pass consistency; inspect multi-basin failures. Shared baseline seeds and baseline-based spatial selection are a declared prior and can favor that prior. External checks are still required before claiming absolute localization accuracy.

## Paired comparison

```json
{
  "queries_summary": {
    "acceptance_rate_difference": -0.10875000000000001,
    "all_starts_agree_fraction_difference": -0.06000000000000005,
    "heldout_point_to_line_rmse_m": {
      "baseline": {
        "count": 707,
        "median": 0.01858982963947788,
        "p95": 0.07798170360048734,
        "max": 0.09589793106164224
      },
      "candidate": {
        "count": 620,
        "median": 0.023795140569638645,
        "p95": 0.08025412578385885,
        "max": 0.09948520088475744
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
        "count": 81,
        "median": 9.935092393758723e-11,
        "p95": 0.8501119335280098,
        "max": 1.5041626673377853
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
        "count": 81,
        "median": 1.4988729700185191e-09,
        "p95": 5.393937443970476,
        "max": 14.381765018625252
      }
    }
  },
  "crosspass_queries_summary": {
    "acceptance_rate_difference": -0.17500000000000004,
    "all_starts_agree_fraction_difference": -0.15000000000000002,
    "heldout_point_to_line_rmse_m": {
      "baseline": {
        "count": 124,
        "median": 0.029933033960839565,
        "p95": 0.07485259746335264,
        "max": 0.07485259762374304
      },
      "candidate": {
        "count": 96,
        "median": 0.04527433332609078,
        "p95": 0.09385472997230633,
        "max": 0.09939733437782206
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
        "count": 13,
        "median": 2.3397941483976237e-11,
        "p95": 0.552511654737204,
        "max": 1.373657896055903
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
        "count": 13,
        "median": 5.294709135165835e-10,
        "p95": 0.0388354706090824,
        "max": 0.045786497340843606
      }
    }
  },
  "selection_rule": "Prefer an experiment only if acceptance and eight-start agreement do not materially worsen and spread or earlier-pass consistency improves across the same queries. Lower fitting residual alone is insufficient; inspect ambiguous and rejected cases. These checks do not establish physical accuracy."
}
```
