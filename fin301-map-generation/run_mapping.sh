#!/usr/bin/env bash
set -euo pipefail
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${MAP_PYTHON:-python3}" "$package_root/src/pipeline.py" "$@"
