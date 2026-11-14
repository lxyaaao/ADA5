#!/usr/bin/env bash
# Run from WSL/Linux; all generated build and verification files stay in project.
set -eo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"
mkdir -p outputs/verification
echo "ROS 2 artifact smoke test"
date -u +'%Y-%m-%dT%H:%M:%SZ'
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=174
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 - <<'PY'
import importlib
import sys
print('Python:', sys.version)
required_missing = []
for name in ['rclpy', 'numpy', 'scipy', 'cv2', 'yaml', 'PIL']:
    try:
        module = importlib.import_module(name)
        print('AVAILABLE', name, getattr(module, '__version__', '(ROS/system package)'))
    except ImportError as exc:
        print('MISSING', name, str(exc))
        if name in ('rclpy', 'numpy', 'yaml', 'PIL'):
            required_missing.append(name)
if required_missing:
    raise SystemExit('Required artifact dependencies missing: ' + ', '.join(required_missing))
PY
PUBLISHER_MODE=module
if command -v colcon >/dev/null 2>&1; then
  colcon --log-base outputs/ros_log build --paths "$PROJECT_ROOT" \
    --build-base outputs/ros_build --install-base outputs/ros_install \
    --packages-select indoor_mapping --event-handlers console_direct+
  source outputs/ros_install/setup.bash
  PUBLISHER_MODE=installed
else
  echo 'colcon unavailable: testing source-module publisher; package build remains untested'
fi
timeout 660s python3 tests/ros_runtime_smoke.py --output outputs/result --publisher-mode "$PUBLISHER_MODE" "$@"
