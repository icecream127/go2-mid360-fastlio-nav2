#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${1:-$(cd "$script_dir/../../../.." && pwd)}"
target="$workspace/src/fast_gicp"
revision=0e7ec1441c99f7be453db2ea216d5de029387417
if [[ ! -e "$target" ]]; then
  git -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=30 clone https://github.com/koide3/fast_gicp.git "$target"
  git -C "$target" checkout --detach "$revision"
elif [[ ! -d "$target/.git" ]] || [[ "$(git -C "$target" rev-parse HEAD)" != "$revision" ]]; then
  echo "Existing $target is not the pinned version; preserve it and resolve manually." >&2
  exit 1
fi
# CPU FastGICP uses system PCL/Eigen/OpenMP; CUDA submodules are not needed.
echo "fast_gicp ready: $target ($revision)"
