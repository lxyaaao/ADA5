# Temporal-filter method review before candidate evaluation

The requested target is a map of repeatedly observed structure rather than a geometric density cleanup. No localization result for either new temporal preset was available when this review was written.

The proposed evidence uses the current one-loop corrected poses and the 3,526 accepted raw LiDAR scans, while the original PCD coordinates remain fixed. A hit requires a nearby measured endpoint. A free observation requires valid beams across a lateral uncertainty neighborhood to pass beyond the candidate point with a range margin. A closer return is an occlusion; an invalid return is unknown. Neither is evidence that the candidate is absent.

Evidence is combined into two-second epochs to reduce repeated votes from adjacent frames. A hit takes precedence over a free observation within an epoch. Raw conflicting counts should remain inspectable. Epoch grouping reduces repetition but does not create statistically independent passes. Actual first and last supporting timestamps, rather than epoch-number differences, must implement duration thresholds.

The fixed proposal uses at least three free epochs across at least four seconds and a free fraction of at least 0.70 for a strong conflict. Repeated structure requires at least three hit epochs across at least ten seconds and a free fraction of at most 0.30. The fraction denominator includes only hit/free epochs. Unknown and occluded observations must be excluded.

The conservative preset removes strong contradictions and retains insufficient evidence. The static-only preset retains repeatedly supported points, so it also removes insufficiently observed structures. Those removals must have separate labels. Removed points do not declare free space. No preset may move, interpolate, or invent coordinates.

Important limitations:

- A person or movable object stationary for the recorded interval can remain. This is persistence classification, not semantic person recognition or proof of permanent structure.
- Residual pose error, range error, voxel averaging and beam discretization can also produce contradiction. A removed point is not automatically a moving object.
- Thin obstacles, depth discontinuities, missing beams and the 360-degree angular boundary require conservative treatment.
- Structure observed only once can be real. The strict output deliberately trades that coverage for repeated support.
- The 3,526 raw IDs are unique, sorted and identical to the recorded odometry timestamp set; no main or earlier-pass query IDs occur in it. Nonetheless, all 100 query scans have an evidence scan within 0.06812 seconds (median 0.05040 seconds). This is strong temporal correlation.

The evaluation will reuse the frozen 100 raw query scans, eight seeds per scan, registration algorithm and gates. Exact exported candidate subsets will be compared with both the original PCD (283 accepted starts / 11 stable queries) and the previous occupancy-supported PCD (332 / 29). This is full-map repeat-observation stability. It is not temporal holdout or surveyed physical accuracy.

The review will report point retention, accepted starts, all-eight stable queries, paired gains/losses and locations that newly lose all accepted starts. Conditional spread and fit residual will retain their denominators. The user's requested static-inspection output can be useful even if it is not the strongest localization target; that tradeoff must be explicit rather than silently reverting every artifact to the original map.
