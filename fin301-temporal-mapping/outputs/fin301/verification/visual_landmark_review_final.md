# Final independent camera-map review

The final component contains **29 camera-derived points from two six-view windows**. The combined PCD contains 49,783 planar LiDAR points plus these 29 camera points, for 49,812 total points. File composition and every camera observation were independently checked.

The checker uses integer-timestamp interpolation, recorded rigid transforms and explicit Brown-Conrady projection, without importing the estimator's triangulation routine. Recomputed residuals agree with saved evidence within 8.86e-13 pixels. Maximum per-landmark reprojection residual is 1.92624 pixels; minimum endpoint parallax is 2.01833 degrees. Camera depths span 2.465–10.554 m. Both accepted windows satisfy the baseline, image support, optical-flow and six-view geometric gates. The independently recomputed undistorted world bearing has positive z in every accepted view; no arbitrary world-height clipping is used.

The contact sheet covers both windows, and the representative endpoint images were inspected. Visible markers lie on overhead lights, ceiling structure or upper glass edges. No marker visibly overlaps a person or the reflective floor in these inspected images. Glass reflections and coherent motion can nevertheless remain consistent over multiple views. The recorded calibration and planar vehicle-attitude model have not been physically validated.

These are **six-view in-sample construction checks**, not untouched predictive validation. Camera poses and metric scale come from the LiDAR trajectory. The camera component has not demonstrated an improvement in localization accuracy. Physical-reference validation remains incomplete; no reference provided.

The earlier 93-point, four-view baseline is preserved separately. A fresh run with `--views 4 --allow-downward-rays` reproduced its PCD byte for byte and every shared CSV evidence field exactly. Its frozen extra-frame result—only 10/93 points within 2 pixels in both later views—applies to that older baseline, not to an untouched test of the final six-view component.

Evidence: `visual_landmark_review_final.json`, `visual_landmark_review_final_contact.png`, `visual_landmark_review_heldout.json`, and the final result's `visual_landmark_evidence.csv`. Re-run numerical checks with `python outputs/fin301/verification/visual_landmark_review_final.py`; a new run resets the visual-inspection status until photographs are reviewed again.
