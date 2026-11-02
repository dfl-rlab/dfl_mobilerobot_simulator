#!/usr/bin/env bash
# Apply integration files to a disposable workspace, never to the submodules.
set -euo pipefail
integration_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${1:?Usage: apply_overlays.bash /path/to/workspace}"
workspace="$(realpath -m "$workspace")"
case "$workspace/" in
    "$integration_root/upstream/"*) echo 'Refusing to modify upstream submodules.' >&2; exit 1 ;;
esac
for name in rl_sar rl_sar_zoo; do
    test -d "$workspace/src/$name" || { echo "Missing $workspace/src/$name" >&2; exit 1; }
    cp -a "$integration_root/overlays/$name/." "$workspace/src/$name/"
done
while IFS= read -r -d '' manifest; do
    ln -sfn package.ros2.xml "$(dirname "$manifest")/package.xml"
done < <(find "$workspace/src" -name package.ros2.xml -print0)
echo "Applied DDDMR overlays to $workspace"
