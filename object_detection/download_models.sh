#!/usr/bin/env bash
# Fetch the deployed OAK-D YOLO blobs into object_detection/.
#
# The blobs are published as GitHub Release assets rather than committed, so a
# clone stays small and the repository does not redistribute AGPL-3.0-derived
# Ultralytics weights under its own MIT licence (see NOTICE).
#
#   bash object_detection/download_models.sh            # fetch what's missing
#   bash object_detection/download_models.sh --force    # re-fetch even if present
#
# Overrides:
#   TB4_MODELS_REPO=owner/name   # where the Release lives  (default below)
#   TB4_MODELS_TAG=models-v1     # which Release tag to pull
#   TB4_ROOT=/path/to/checkout   # defaults to this script's parent directory
set -euo pipefail

REPO=${TB4_MODELS_REPO:-scan33scan33/turtlebot4-glassbox}
TAG=${TB4_MODELS_TAG:-models-v1}

# Default to the checkout this script lives in, so it works from any cwd.
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=${TB4_ROOT:-$(dirname -- "$HERE")}
DEST="$HERE"
FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

# name <TAB> sha256   (sha256sum of the file as published)
read -r -d '' MODELS <<'EOF' || true
yolov5mu_416_5shave.blob	17177475c105bc8e067c67d1dbe497ad3c10f6ac9a3fd82ee0860716e484f5a5
yolov8s_416_fixed_6shave.blob	096ff790a30b57f682c68ea29bf3e28567fd1f0526895f09c661cb9719b30e63
EOF

if ! command -v curl >/dev/null 2>&1; then
    echo "error: curl is required but not installed." >&2
    exit 1
fi

mkdir -p "$DEST"
rc=0

verify() {  # verify <file> <expected-sha256>; 0 = matches
    [ -f "$1" ] || return 1
    command -v sha256sum >/dev/null 2>&1 || return 0   # can't check, assume ok
    [ "$(sha256sum "$1" | cut -d' ' -f1)" = "$2" ]
}

while IFS=$'\t' read -r name want; do
    [ -n "$name" ] || continue
    out="$DEST/$name"

    if [ "$FORCE" -eq 0 ] && verify "$out" "$want"; then
        echo "ok       $name (already present, checksum matches)"
        continue
    fi
    if [ "$FORCE" -eq 0 ] && [ -f "$out" ]; then
        echo "stale    $name (checksum mismatch) — re-downloading"
    fi

    url="https://github.com/$REPO/releases/download/$TAG/$name"
    echo "fetching $name"
    echo "         $url"
    # Download beside the target so a failed transfer never leaves a partial
    # blob that depthai would try to load.
    tmp="$out.part"
    if ! curl -fL --retry 3 --retry-delay 2 --progress-bar -o "$tmp" "$url"; then
        rm -f "$tmp"
        echo "error    could not download $name from $REPO ($TAG)" >&2
        rc=1
        continue
    fi

    if verify "$tmp" "$want"; then
        mv -f "$tmp" "$out"
        echo "ok       $name"
    else
        got=$(sha256sum "$tmp" 2>/dev/null | cut -d' ' -f1)
        rm -f "$tmp"
        echo "error    $name failed checksum" >&2
        echo "         expected $want" >&2
        echo "         got      ${got:-<none>}" >&2
        rc=1
    fi
done <<< "$MODELS"

if [ "$rc" -ne 0 ]; then
    cat >&2 <<MSG

One or more blobs could not be fetched. If you forked or renamed the repo, point
this at your own Release:

    TB4_MODELS_REPO=<owner>/<name> bash object_detection/download_models.sh

You can also build the blobs yourself — see object_detection/README.md and
object_detection/YOLOV8S_SWAP.md for the ultralytics + luxonis/tools export steps.
MSG
    exit "$rc"
fi

echo
echo "Models are in $DEST — you can now run object_detection/run_oakd.sh."
