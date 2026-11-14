# Offline Viewer Implementation Plan

**Goal:** Deliver a self-contained interactive viewer of the final fin301 PCD, YAML/PGM and trajectory.

**Architecture:** Plain browser JavaScript, WebGL renderer plus Canvas overlay, DOM controls, all source data embedded as base64. Python assembles the offline artifact. Node runs pure parsing/math tests; browser checks exercise the actual file.

**Constraints:** No network dependencies or map modifications. Coordinates stay metric in the existing map frame. Grid image top rows must map to high local y. Camera points do not become scan-plane obstacles.

- [ ] Parsers: `viewer/io.js`, `tests/viewer_io.test.cjs`. Expose `MapIO.parsePCD(ArrayBuffer)`, `parseMapYaml(string)`, `parsePGM(ArrayBuffer)`, `parseTrajectory(string)`. Test malformed data, ASCII/binary XYZ offsets, P5 whitespace-valued pixels, YAML yaw, exact timestamp strings.
- [ ] Renderer: `viewer/renderer.js`, `tests/viewer_renderer.test.cjs`. Expose `MapRenderer.ViewerRenderer(canvas,overlay)` with layers/camera/projection/measurement methods agreed in agent messages. Test world/screen roundtrips, yaw and grid corners.
- [ ] Assembly: `scripts/build_viewer.py`, `viewer/README.md`. Embed CSS/JS and `<script id="bundled-files" type="application/json">` with dataset, files `{name,base64}`, metadata. Export HTML, original maps, launcher and ZIP with hashes.
- [ ] UI integration: `viewer/index.html`, `viewer/style.css`, `viewer/app.js`. Chinese controls, map layers, navigation, measurement, timeline playback, import/export and errors. Protect same-frame assumptions during import.
- [ ] Verify: run Node tests and build script; inspect actual browser rendering and operate each feature; verify bundled map bytes; produce screenshots and concise launcher instructions.

No git repository is present, so files remain in the shared workspace without a synthetic commit.
