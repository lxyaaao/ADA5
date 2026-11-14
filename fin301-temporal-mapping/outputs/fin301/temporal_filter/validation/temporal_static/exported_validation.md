# Exported PCDs: full-map repeat-observation stability

**This is not the temporally excluded reference test and does not measure absolute localization accuracy.**

The exact exported XYZ coordinates are used as targets. Every method uses the same frozen100 raw query scans and eight baseline-frame initial perturbations. Queries retain the original0.06m source preprocessing. The exported target is transformed into each frozen seed frame, cropped to14m and aggregated at0.07m, with unchanged ICP/acceptance gates. No map is regenerated and no camera geometry enters.

The reconstructed production insertion list contains780 scans and shares zero IDs with the queries. However,87/100 queries have a map insertion within5s. The full map includes nearby observations of the same scene; temporal/cross-pass claims from the separate evaluator do not apply here.

| Exact exported artifact | Points | Accepted /800 | Stable /100 | Conditional translation spread p95(m) | Accepted line-fit median(m) |
|---|---:|---:|---:|---:|---:|
| original | 49783 | 283 | 11 | 0.9861187894802192 | 0.011494415222214411 |
| temporal_static | 39614 | 271 | 18 | 0.019428822935786686 | 0.011684514872425252 |

Stable requires all eight registrations to pass and agree within0.15m and3degrees. Rejections stay in the denominator. Conditional spread includes only queries with at least two accepted solutions; fit residual is not pose error against truth.

| Candidate vs original | Accepted gains /losses | Stable-query gains /losses |
|---|---:|---:|
| temporal_static | 36 /48 | 9 /2 |

This reused development diagnostic supports inspecting artifact-specific stability, not automatic promotion. The original map remains unchanged. Interpret alongside the separate temporally excluded reference-only tests and spatial feature loss; no independent physical-reference measurement was provided.
