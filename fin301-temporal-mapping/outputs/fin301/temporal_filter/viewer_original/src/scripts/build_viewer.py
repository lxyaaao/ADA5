"""Build a self-contained offline map viewer and byte-verified ZIP.

Default: python scripts/build_viewer.py --dataset fin301
The browser artifact needs no Python, server, network, or installation.
Rebuilding requires Python 3.10+ and PyYAML, already in project requirements.
"""
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
from html.parser import HTMLParser
import io
import json
import math
from pathlib import Path, PureWindowsPath
import re
import sys
import zipfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ("index.html", "style.css", "io.js", "renderer.js", "app.js")
CORE_MAP_FILES = ("lidar_scan_map.pcd", "visual_landmarks.pcd", "map.pcd",
                  "map.yaml", "trajectory.csv")
TOKENS = {"{{STYLE}}": "style.css", "{{IO_JS}}": "io.js",
          "{{RENDERER_JS}}": "renderer.js", "{{APP_JS}}": "app.js"}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_inside(directory, relative):
    """Resolve a local artifact without following a reference outside its root."""
    name = str(relative).replace("\\", "/")
    require(name and not Path(name).is_absolute()
            and not PureWindowsPath(name).is_absolute()
            and not PureWindowsPath(name).drive
            and ":" not in name and ".." not in Path(name).parts,
            f"Artifact reference must be a relative local path: {relative!r}")
    root = Path(directory).resolve()
    path = (root / name).resolve()
    require(path.is_relative_to(root), f"Artifact reference escapes result directory: {relative!r}")
    require(path.is_file(), f"Required artifact missing: {path}")
    return path.read_bytes()


def parse_project_pcd(data, name):
    """Check the project's fixed XYZ ASCII export, independent of browser import."""
    stream = io.StringIO(data.decode("ascii"))
    header = {}
    for line in stream:
        fields = line.strip().split()
        if not fields or fields[0].startswith("#"):
            continue
        header[fields[0].upper()] = fields[1:]
        if fields[0].upper() == "DATA":
            break
    for key, expected in (("FIELDS", ["x", "y", "z"]), ("SIZE", ["4"] * 3),
                          ("TYPE", ["F"] * 3), ("COUNT", ["1"] * 3),
                          ("DATA", ["ascii"])):
        require(header.get(key) == expected, f"{name}: expected project XYZ float32 ASCII PCD ({key})")
    points = []
    for number, line in enumerate(stream, 1):
        if not line.strip():
            continue
        tokens = line.split()
        require(len(tokens) == 3, f"{name}: invalid XYZ row {number}")
        point = tuple(float(token) for token in tokens)
        require(all(math.isfinite(value) for value in point), f"{name}: non-finite XYZ row {number}")
        points.append(point)
    count = int(header["POINTS"][0])
    require(count == len(points) == int(header["WIDTH"][0]) * int(header["HEIGHT"][0]),
            f"{name}: PCD header/data point count mismatch")
    return points


def collect_data(result):
    result = Path(result).resolve()
    files = {name: read_inside(result, name) for name in CORE_MAP_FILES}
    config = yaml.safe_load(files["map.yaml"].decode("utf-8-sig"))
    require(isinstance(config, dict), "map.yaml must contain a mapping")
    image_name = config.get("image")
    require(isinstance(image_name, str), "map.yaml image must be a local filename")
    image_name = image_name.replace("\\", "/")
    require(Path(image_name).suffix.lower() in (".pgm", ".png", ".jpg", ".jpeg"),
            "map.yaml must reference a supported PGM, PNG or JPEG image")
    files[image_name] = read_inside(result, image_name)
    resolution = float(config["resolution"])
    origin = config["origin"]
    require(math.isfinite(resolution) and resolution > 0, "map.yaml resolution must be positive")
    require(isinstance(origin, list) and len(origin) == 3
            and all(math.isfinite(float(value)) for value in origin), "map.yaml origin must be finite [x,y,yaw]")
    files["map_metadata.json"] = read_inside(result, "map_metadata.json")
    metadata = json.loads(files["map_metadata.json"])
    metrics_path = result / "metrics.json"
    metrics = {}
    if metrics_path.is_file():
        files["metrics.json"] = read_inside(result, "metrics.json")
        metrics = json.loads(files["metrics.json"])
    lidar = parse_project_pcd(files["lidar_scan_map.pcd"], "lidar_scan_map.pcd")
    camera = parse_project_pcd(files["visual_landmarks.pcd"], "visual_landmarks.pcd")
    combined = parse_project_pcd(files["map.pcd"], "map.pcd")
    require(lidar and all(point[2] == 0 for point in lidar), "LiDAR component must be a nonempty z=0 scan plane")
    require(len(combined) == len(lidar) + len(camera), "Combined/component PCD count mismatch")
    max_difference = max((abs(a - b) for p, q in zip(combined, lidar + camera)
                          for a, b in zip(p, q)), default=0.)
    require(max_difference <= 3.01e-6, "Combined PCD does not match ordered LiDAR/camera components")
    require(metadata.get("unit") == "metre" and metadata.get("frame_id") == "map",
            "Map metadata must identify the metric local map frame")
    counts = {"lidar_point_count": len(lidar), "visual_point_count": len(camera), "points": len(combined)}
    for key, count in counts.items():
        require(metadata.get(key) == count, f"Map metadata count mismatch: {key}")
        metric_key = "pcd_points" if key == "points" else key
        if metric_key in metrics:
            require(metrics[metric_key] == count, f"Metrics count mismatch: {metric_key}")
    rows = list(csv.DictReader(io.StringIO(files["trajectory.csv"].decode("utf-8-sig"))))
    require(bool(rows), "trajectory.csv has no samples")
    stamps = [int(row["timestamp_ns"]) for row in rows]
    require(all(b > a for a, b in zip(stamps, stamps[1:])), "Trajectory timestamps must increase")
    require(stamps[0] == int(metadata["origin_timestamp_ns"]), "Trajectory/map origin timestamp mismatch")
    require(all(abs(float(rows[0][key])) <= 1e-8 for key in ("lidar_x", "lidar_y", "lidar_yaw")),
            "Trajectory must start at the first LiDAR pose origin")
    mount = metadata["base_to_lidar_se2"]
    maximum_mount_error = 0.
    for row in rows:
        values = [float(row[key]) for key in ("x", "y", "yaw", "lidar_x", "lidar_y", "lidar_yaw")]
        require(all(math.isfinite(value) for value in values), "Trajectory contains non-finite pose values")
        x, y, yaw, lx, ly, lyaw = values
        predicted = (x + math.cos(yaw) * mount[0] - math.sin(yaw) * mount[1],
                     y + math.sin(yaw) * mount[0] + math.cos(yaw) * mount[1])
        angle_error = math.atan2(math.sin(yaw + mount[2] - lyaw), math.cos(yaw + mount[2] - lyaw))
        maximum_mount_error = max(maximum_mount_error, abs(lx - predicted[0]), abs(ly - predicted[1]), abs(angle_error))
    require(maximum_mount_error < 1e-6, "Trajectory base and LiDAR frames are inconsistent with recorded mount")
    validation = {**counts, "trajectory_samples": len(rows), "trajectory_start_ns": str(stamps[0]),
                  "trajectory_end_ns": str(stamps[-1]), "yaml_image": image_name,
                  "grid_resolution_m": resolution, "grid_origin_xy_yaw": origin,
                  "max_component_difference_m": max_difference,
                  "max_base_lidar_transform_error": maximum_mount_error}
    return files, {"map_metadata": metadata, "metrics": metrics}, validation


def browser_safe_numbers(value):
    """Keep nanosecond timestamps exact after JSON.parse in a browser."""
    if isinstance(value, int) and abs(value) > 2**53 - 1:
        return str(value)
    if isinstance(value, list):
        return [browser_safe_numbers(item) for item in value]
    if isinstance(value, dict):
        return {key: browser_safe_numbers(item) for key, item in value.items()}
    return value


def assemble_html(source, dataset, files, metadata):
    source = Path(source)
    missing = [name for name in (*ASSETS, "README.md") if not (source / name).is_file()]
    require(not missing, "Viewer assets are not ready; missing: " + ", ".join(missing))
    assets = {name: (source / name).read_bytes() for name in (*ASSETS, "README.md")}
    template = assets["index.html"].decode("utf-8-sig")
    for token in (*TOKENS, "{{BUNDLE_JSON}}"):
        require(template.count(token) == 1, f"index.html must contain exactly one {token} placeholder")
    require(re.search(r'<script\b[^>]*\bid=[\"\']bundled-files[\"\']', template),
            "index.html must provide script id=\"bundled-files\"")
    bundle = {"dataset": dataset, "files": [{"name": name, "base64": base64.b64encode(data).decode("ascii")}
                                                for name, data in files.items()],
              "metadata": browser_safe_numbers(metadata)}
    serialized = json.dumps(bundle, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    serialized = (serialized.replace("&", "\\u0026").replace("<", "\\u003c").replace(">", "\\u003e")
                  .replace("\u2028", "\\u2028").replace("\u2029", "\\u2029"))
    replacements = {"{{BUNDLE_JSON}}": serialized}
    for token, name in TOKENS.items():
        content = assets[name].decode("utf-8-sig")
        require(not re.search(r'</(?:script|style)\b', content, re.I), f"{name}: embedded closing script/style tag is unsafe")
        replacements[token] = content
    # One pass avoids interpreting placeholder-looking strings inside source assets/data.
    html = re.sub(r'\{\{(?:STYLE|IO_JS|RENDERER_JS|APP_JS|BUNDLE_JSON)\}\}',
                  lambda match: replacements[match.group()], template)
    require(not re.search(r'<script\b[^>]*\bsrc\s*=', html, re.I), "Standalone viewer cannot have external script src")
    require(not re.search(r'<link\b[^>]*\brel\s*=\s*[\"\']?stylesheet', html, re.I),
            "Standalone viewer cannot depend on linked stylesheets")
    return html.encode("utf-8"), assets


def verify_embedded_files(html, dataset, originals):
    class BundleParser(HTMLParser):
        def __init__(self):
            super().__init__(convert_charrefs=False)
            self.active = False
            self.matches = 0
            self.parts = []

        def handle_starttag(self, tag, attrs):
            attributes = dict(attrs)
            if tag == "script" and attributes.get("id") == "bundled-files":
                require(attributes.get("type") == "application/json", "bundled-files must use application/json")
                self.active = True
                self.matches += 1

        def handle_endtag(self, tag):
            if tag == "script":
                self.active = False

        def handle_data(self, data):
            if self.active:
                self.parts.append(data)

    parser = BundleParser()
    parser.feed(html.decode("utf-8"))
    require(parser.matches == 1, "HTML must have exactly one bundled-files JSON element")
    bundle = json.loads("".join(parser.parts))
    require(bundle["dataset"] == dataset, "Embedded dataset mismatch")
    entries = bundle["files"]
    require(len(entries) == len(originals) == len({entry["name"] for entry in entries}), "Embedded file count mismatch")
    for entry in entries:
        require(entry["name"] in originals, f"Unexpected embedded artifact: {entry['name']}")
        require(base64.b64decode(entry["base64"], validate=True) == originals[entry["name"]],
                f"HTML embedded bytes differ: {entry['name']}")


def write_atomic(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_bytes(content)
    temporary.replace(path)


def verify_package(archive, members):
    with zipfile.ZipFile(archive) as handle:
        require(handle.testzip() is None, "ZIP CRC verification failed")
        require(len(handle.namelist()) == len(set(handle.namelist())), "ZIP has duplicate entries")
        require(set(handle.namelist()) == set(members), "ZIP member set mismatch")
        for name, expected in members.items():
            require(handle.read(name) == expected, f"ZIP bytes differ: {name}")


def add_filtered_clouds(files, metadata, validation, paths, report=None):
    """Bundle optional, exact-subset PCDs without replacing original map data."""
    if not paths:
        require(report is None, "A filtering report requires at least one filtered PCD")
        return
    original = parse_project_pcd(files["lidar_scan_map.pcd"], "lidar_scan_map.pcd")
    source_points = set(original)
    variants = {}
    for path in paths:
        path = Path(path)
        name = path.name
        require(name.lower().endswith(".pcd") and name.lower() not in {key.lower() for key in files},
                f"Filtered cloud must be a separate PCD filename: {name}")
        data = path.read_bytes()
        points = parse_project_pcd(data, name)
        require(0 < len(points) < len(original), f"{name}: expected a nonempty filtered subset")
        require(len(set(points)) == len(points) and all(point in source_points for point in points),
                f"{name}: every retained XYZ must exactly match an original LiDAR point")
        files[name] = data
        variants[name] = {"kind": "filtered", "source": "lidar_scan_map.pcd",
                          "points": len(points), "removed_points": len(original) - len(points),
                          "removed_percent": 100 * (1 - len(points) / len(original)),
                          "label": "Filtered: " + name.removeprefix("lidar_scan_map_clean_").removesuffix(".pcd").replace('_', ' '),
                          "note": "Exact subset of original scan points. Original occupancy grid and trajectory retained. Filtering does not establish accuracy."}
    if report is not None:
        report = Path(report)
        require(report.suffix.lower() == '.json' and report.name.lower() not in {key.lower() for key in files},
                "Filtering report requires a separate JSON filename")
        content = report.read_bytes()
        json.loads(content)
        files[report.name] = content
    metadata["pointcloud_variants"] = variants
    validation["filtered_variants"] = variants
    validation["original_grid_unchanged"] = True


def build(result, source, output, archive, dataset, filtered_pcds=(), filter_report=None, display_title=None):
    result, output = Path(result).resolve(), Path(output).resolve()
    require(not output.is_relative_to(result), "Viewer output must be separate from original result artifacts")
    files, metadata, validation = collect_data(result)
    if display_title:
        metadata['viewer_title'] = str(display_title)
    original_names = tuple(files)
    add_filtered_clouds(files, metadata, validation, filtered_pcds, filter_report)
    html, assets = assemble_html(source, dataset, files, metadata)
    verify_embedded_files(html, dataset, files)
    members = {"viewer.html": html, "README.md": assets["README.md"],
               "open_viewer.cmd": b'@echo off\r\nstart "" "%~dp0viewer.html"\r\n'}
    members.update({"maps/" + name: data for name, data in files.items()})
    members.update({"src/viewer/" + name: data for name, data in assets.items()})
    members["src/scripts/build_viewer.py"] = Path(__file__).read_bytes()
    manifest = {"schema_version": 1, "dataset": dataset, "entrypoint": "viewer.html",
                "offline": True, "map_files_are_original_bytes": not bool(filtered_pcds),
                "original_map_files_preserved": list(original_names),
                "metadata_integer_encoding": "Integers outside JavaScript safe range become decimal strings only in embedded JSON; original JSON files are unchanged.",
                "validation": validation, "hash_algorithm": "SHA-256",
                "manifest_note": "Each packaged file except this manifest is listed; no recursive self-hash.",
                "files": [{"path": name, "bytes": len(data), "sha256": sha256(data)}
                          for name, data in sorted(members.items())]}
    members["viewer_manifest.json"] = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    for name, data in members.items():
        write_atomic(output / name, data)
    for name, data in files.items():
        if name in original_names:
            require(read_inside(result, name) == data, f"Source artifact changed during build: {name}")
        require((output / "maps" / name).read_bytes() == data, f"Copied artifact differs: {name}")
    archive = Path(archive).resolve()
    require(archive.name != "Project_2_2_fin301.zip", "The original project delivery ZIP must remain unchanged")
    require(not archive.is_relative_to(result), "ZIP output must be separate from original result artifacts")
    archive.parent.mkdir(parents=True, exist_ok=True)
    temporary = archive.with_name(archive.name + ".tmp")
    with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as handle:
        for name, data in sorted(members.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            handle.writestr(info, data, compresslevel=9)
    verify_package(temporary, members)
    temporary.replace(archive)
    return {"status": "PASS", "html": str(output / "viewer.html"), "zip": str(archive),
            "html_bytes": len(html), "zip_bytes": archive.stat().st_size,
            "packaged_files": len(members), "original_map_bytes_verified": True, "validation": validation}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="fin301", help="Simple dataset name; default fin301")
    parser.add_argument("--result", type=Path, help="Original result directory")
    parser.add_argument("--source", type=Path, default=ROOT / "viewer", help="Viewer template/source directory")
    parser.add_argument("--output", type=Path, help="Standalone viewer directory")
    parser.add_argument("--zip", dest="archive", type=Path, help="Standalone delivery ZIP")
    parser.add_argument("--filtered-pcd", type=Path, action="append", default=[],
                        help="Additional exact-subset LiDAR PCD; repeat to compare variants")
    parser.add_argument("--filter-report", type=Path, help="Optional JSON filtering evidence bundled with the variants")
    parser.add_argument("--title", help="Display title for this dataset or experiment")
    parser.add_argument("--validate-data-only", action="store_true", help="Check data and frames without requiring UI assets or writing output")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.dataset):
        parser.error("dataset must be a simple name of letters, numbers, _ or -")
    result = args.result or ROOT / "outputs" / args.dataset / "result"
    output = args.output or ROOT / "outputs" / args.dataset / "viewer"
    archive = args.archive or ROOT / "deliverables" / f"{args.dataset}_viewer.zip"
    try:
        if args.validate_data_only:
            _, _, validation = collect_data(result)
            report = {"status": "PASS", "data_only": True, "validation": validation}
        else:
            report = build(result, args.source, output, archive, args.dataset,
                           args.filtered_pcd, args.filter_report, args.title)
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError, zipfile.BadZipFile) as error:
        print(f"Viewer build failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
