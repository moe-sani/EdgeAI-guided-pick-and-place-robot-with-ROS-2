#!/usr/bin/env bash
# Apply our small patches to the upstream clones in src/ (safe to re-run).
# Run after `vcs import src < ros2_ws.repos`, before building.
set -euo pipefail
cd "$(dirname "$0")"
for patch in patches/*.patch; do
    repo="src/$(basename "$patch" .patch)"
    if git -C "$repo" apply --reverse --check "$PWD/$patch" 2>/dev/null; then
        echo "already applied: $patch"
    elif git -C "$repo" apply --check "$PWD/$patch"; then
        git -C "$repo" apply "$PWD/$patch" && echo "applied: $patch"
    else
        echo "FAILED (does not apply cleanly): $patch" >&2
        exit 1
    fi
done
