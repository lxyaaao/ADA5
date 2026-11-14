# Independent visual-landmark review: fin301 baseline

This review concerns the frozen **93-candidate, four-view baseline**, preserved under `outputs/fin301/baseline`, with source hashes in the JSON files. It is not an approval of absolute localization accuracy. The current estimator trajectory remains LiDAR-derived; the camera candidates have not demonstrated pose-accuracy improvement.

## Numerical and file consistency

All 93 candidate XYZ points from six accepted image windows were checked without importing the estimator's triangulation routine. Independent integer-time interpolation, rigid transforms and explicit Brown-Conrady projection reproduce the saved four-view evidence within 1.42e-12 pixels. All checked depths, baselines, parallax, forward/backward errors and projection residuals satisfy the recorded acceptance gates. The exported components concatenate correctly: 53,700 planar LiDAR points plus 93 camera candidates give 53,793 XYZ points. These checks establish implementation consistency, not physical truth.

A real image-input defect was found and corrected by the implementation agent: 2,912 valid source JPEGs have one zero padding byte after their EOI marker, while 2,965 end directly at EOI. A strict last-two-byte test rejected valid padded files. Only original image IDs 2764 and 3045 lack EOI and fail both independent decoders. The targeted padding fix retained the geometric gates and increased the camera baseline from 14 to 93 candidates.

## Physical interpretation and visual inspection

The recorded camera origin is (0.205788, 0, -0.08749) m in the laser frame. Negative world z is therefore valid in general; a z>=0 clipping rule would be unjustified.

The contact sheet covers all six accepted image groups. Most markers lie on ceiling lights, structural edges, curtains or upper surfaces. No marker visibly overlapped people in these sampled first views, but this does not establish that all features are static. Candidate landmark 89, from frames 5544/5547/5550/5553, marks the reflective floor at image y approximately344-359 and triangulates to z=-2.078394 m. It is likely a virtual reflection point despite passing all four training views. The point should not be represented as a verified physical surface measurement.

A supported conservative alternative is to retain only upward-looking world rays, derived from undistorted camera bearings and the recorded rotation. This defines an explicit upper-view structural scope without inventing a floor height. It retains 91 of the 93 baseline candidates, including legitimately negative z when the camera lies below the scan plane, and excludes the suspected floor reflection. It does not detect window reflections above the horizon and is not a complete semantic static-object mask.

## Frozen fifth/sixth-view predictive evaluation

The candidate XYZ and LiDAR trajectory were frozen. Images approximately0.3s and0.6s after each fourth training view were not used in triangulation. Features were propagated with forward/backward LK error <=0.8px, and predicted pixels were compared with the observed image coordinates. No camera calibration, pose, point or threshold was fitted to these extra views.

| Subset | Training candidates | Tracked in view5 / view6 | Pass <=2px in both extra views | View5 error median / p95 | View6 error median / p95 |
|---|---:|---:|---:|---:|---:|
| All candidates | 93 | 83 / 70 | 10 | 2.189 / 4.833 px | 4.264 / 9.586 px |
| Upward world rays | 91 | 81 / 68 | 10 | 2.192 / 4.866 px | 4.264 / 9.588 px |

The 10 predictive passes occur in three windows, with 2, 7 and 1 point respectively. This does not support calling the original93 points robust generalizable localization landmarks. Reflection candidate89 passes the fifth view at1.666px but fails the sixth at5.198px. A reflection could in principle remain geometrically consistent for many views, so even six-view agreement cannot by itself establish a physical surface.

If these extra views are used to select or refit an output, they become part of acceptance/training and must no longer be reported as an untouched held-out evaluation of that selected output. No independent absolute trajectory reference was supplied.

## Signed-error evidence

Sixth-view residuals show substantial shared direction within some windows even when optical-flow forward/backward error is small:

- Group790: median observed-minus-predicted (du,dv)=(+4.29,+5.69)px; all vertical residuals positive; median LK FB error0.046px.
- Group5524: (+3.61,+4.61)px; all residual components positive; median FB error0.084px.
- Group5544: (-2.58,-3.25)px; all vertical residuals negative; median FB error0.023px.

This is inconsistent with treating all discrepancy as random LK loss or correcting it with one constant pixel offset. Planar attitude assumptions, calibration error, landmark-depth uncertainty and coherent tracking error are plausible explanations; these observations do not identify a unique cause. No calibration optimization was performed.

## Qualitative context for proposed LiDAR closures

Keyframes262->422 and264->424 both view a wood/glass counter region; later frames expose a neighboring gray pillar and trash bin. Keyframes104->137 view the curtained room/large pillar near white bar furniture. The photographs are compatible with neighborhood revisits, but repeated panels, glass, low texture and viewpoint differences do not uniquely establish identical locations. The latter pair has only about3.10m intervening excursion. No positive visual metric loop was established; LiDAR temporal and held-out checks remain necessary.

## Artifacts

- `visual_landmark_review.json`: every-point numerical/file checks and explicit limitations.
- `visual_landmark_review_contact.png`: sampled first views from all six baseline groups.
- `visual_landmark_review_heldout.json`: all candidate predictions, LK checks, per-window signed residuals and frozen-source hashes.
- `visual_landmark_review_payloads.json`: original JPEG padding/truncation evidence.
- `visual_landmark_review_sixview_experiment.py`: separately authorized six-view in-sample experiment; it does not modify the core or baseline result. Its output must be labeled in-sample, not held out.

Baseline reproduction with the current configurable core: `python -m indoor_mapping.visual_landmarks --cache outputs/fin301/cache --trajectory outputs/fin301/baseline/trajectory_arrays.npz --calibration outputs/fin301/audit/calibration.json --output <separate-output-directory> --views 4 --allow-downward-rays`. Final construction defaults to six views and upward world rays.
