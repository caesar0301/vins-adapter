#!/usr/bin/env bash
# Build the offline VINS adapter in Docker and copy it out of the image.
#
#   ./vins-adapter/build_vins_adapter.sh [options]
#
#   --image NAME      image name for the build (default: vins-adapter)
#   --tag TAG         image tag (default: noetic-jammy)
#   --out DIR         copy target (default: vins-adapter/vins_adapter/)
#   --jobs N          parallel build jobs inside the container
#                     (default: all CPUs visible inside the container --
#                     the VM's vCPUs, not the host's)
#   --platform PLAT   docker platform (default: linux/amd64 -- the ROS jammy
#                     packages used here are amd64-only)
#   --remote HOST     build on a remote (ideally native-amd64 Linux) docker
#                     daemon over ssh instead of the local one; the context is
#                     streamed and the binary copied back automatically
#   --force           docker build --no-cache (otherwise cached layers are
#                     reused and a matching ref does not rebuild)
#   --check           only verify the copied adapter, do not build
#   --skip-tests      do not run tests/regression tests after the build
#                     (they run automatically on every build; also runnable
#                     standalone: python3 tests/run_tests.py --sources-only)
#
# VINS-Fusion is taken from the local checkout at vins-adapter/VINS-Fusion
# (no network clone at build time); update that checkout to build a different
# snapshot.
#
# Why Docker
# ----------
# The adapter must link VINS-Fusion (GPLv3) and its estimator still touches
# roscpp, so the build needs a full ROS1 + Ceres + OpenCV toolchain. Building
# inside an ubuntu:22.04 container keeps the host clean and makes the build
# reproducible; the finished binary is copied out of the image in step 4.
#
# ROS1 on Ubuntu 22.04
# --------------------
# Official ROS1 Noetic targets Ubuntu 20.04 only and the third-party jammy
# apt ports are gone, so the image installs ROS1 from RoboStack (conda-forge).
# The adapter runs without a roscore: the estimator's ROS publishers are
# never registered, so publish() calls no-op.
#
# The adapter contract
# --------------------
#   vins_adapter <config.yaml> <output.tum>
#
#       Reads the flat config yaml (camera intrinsics, T_cam0_body /
#       T_cam1_body, left_dir, right_dir, optional imu_csv and
#       frame_times_csv -- keys documented in adapter/vins_adapter.cpp), runs
#       the estimator, and writes a TUM trajectory:
#           timestamp tx ty tz qx qy qz qw        (seconds, one line per frame)
#       Exit non-zero with a message on stderr when the run fails.
#
#       One process per episode, stateless across runs: a fresh Estimator is
#       constructed per invocation so no sliding-window / marginalisation
#       state leaks between recordings. Poses are emitted once the estimator
#       reaches NON_LINEAR (window initialized) -- short or near-static
#       episodes legitimately produce fewer rows than frames.
#
# Where the binary can live
# -------------------------
# By default it is copied to vins-adapter/vins_adapter/ (binary + bundled
# shared libs + run_vins_adapter.sh launcher). The copy is self-contained: it
# only needs glibc on the target Linux host/worker (or run it inside the built
# image), not a ROS install. A self-contained tar.gz of that directory is also
# produced at vins-adapter/vins_adapter.tar.gz for shipping to workers, with an
# md5 checksum beside it (vins_adapter.tar.gz.md5, md5sum -c compatible);
# verify after transfer with: md5sum -c vins_adapter.tar.gz.md5.
# Unpack it anywhere and run ./run_vins_adapter.sh <config.yaml> <output.tum>.
# It is a Linux ELF: do not run it on macOS directly.
#
# Two conversion traps when bridging VINS-Fusion's own output
# -----------------------------------------------------------
#   vio.csv is:  ts_ns, px,py,pz, qw,qx,qy,qz, vx,vy,vz     (qw FIRST, ns)
#   TUM wants:   ts_s,  tx,ty,tz, qx,qy,qz,qw               (seconds, q LAST)
# The adapter converts both, exactly once, at the boundary.
#
# License: GPLv3 (see LICENSE in this directory).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE="vins-adapter"
TAG="noetic-jammy"
OUT="$SCRIPT_DIR/vins_adapter"
JOBS=""
PLATFORM="linux/amd64"
REMOTE=""
FORCE=0
CHECK=0
SKIP_TESTS=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)    IMAGE="$2"; shift 2 ;;
    --tag)      TAG="$2"; shift 2 ;;
    --remote)   REMOTE="$2"; shift 2 ;;
    --out)      OUT="$2"; shift 2 ;;
    --jobs)     JOBS="$2"; shift 2 ;;
    --platform) PLATFORM="$2"; shift 2 ;;
    --force)    FORCE=1; shift ;;
    --check)    CHECK=1; shift ;;
    --skip-tests) SKIP_TESTS=1; shift ;;
    -h|--help)  sed -n '2,90p' "$0" | sed 's/^#\{1,2\} \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
done

log() { printf '\033[1m[vins-build]\033[0m %s\n' "$*"; }
die() { printf '\033[31m[vins-build] error:\033[0m %s\n' "$*" >&2; exit 1; }

BIN="$OUT/vins_adapter"

verify() {
  [[ -x "$BIN" ]] || die "no adapter at $BIN"
  log "copied: $BIN"
  log "contract:  vins_adapter <config.yaml> <output.tum>"
  # Standalone check 1: every shared dep must resolve to the bundled
  # $ORIGIN/lib or host glibc -- a "not found" here means a lib was missed
  # during bundling and the copy would break on a clean host.
  if command -v ldd >/dev/null; then
    if ldd "$BIN" 2>&1 | grep -q 'not found'; then
      ldd "$BIN" 2>&1 | grep 'not found' >&2
      die "unresolved shared deps (bundling incomplete)"
    fi
    log "standalone: all shared deps resolved ($(ls "$OUT/lib" 2>/dev/null | wc -l) bundled libs in lib/)"
  fi
  # Standalone check 2: run it. No args = usage error, exit 2; this proves
  # the binary starts and links on this host with no ROS/conda installed.
  local rc=0
  "$BIN" 2>/dev/null || rc=$?
  [[ "$rc" -eq 2 ]] || die "smoke run failed (exit $rc, expected 2) -- binary does not run standalone on this host"
  log "standalone: smoke run OK"
  if [[ -f "$OUT/BUILDINFO.json" ]]; then
    log "provenance: $(tr -d '\n' < "$OUT/BUILDINFO.json")"
  fi
  # Bundle integrity: if a tarball and its checksum ship together, the shipped
  # artifact is what was last built (catches transfer corruption / stale pairs).
  local tarball="$OUT.tar.gz"
  if [[ -f "$tarball" && -f "$tarball.md5" ]]; then
    local expected actual
    expected="$(awk '{print $1}' "$tarball.md5")"
    actual="$(_md5 "$tarball")"
    [[ "$actual" == "$expected" ]] || die "bundle checksum mismatch: $tarball.md5 says $expected, tarball is $actual"
    log "bundle: $tarball md5 verified ($actual)"
  fi
}

# Portable md5 (md5sum on Linux, md5 on macOS build hosts).
_md5() {
  if command -v md5sum >/dev/null; then
    md5sum "$1" | awk '{print $1}'
  elif command -v md5 >/dev/null; then
    md5 -q "$1"
  else
    die "need md5sum or md5 to checksum the bundle"
  fi
}

if [[ "$CHECK" -eq 1 ]]; then
  verify
  exit 0
fi

command -v docker >/dev/null || die "docker is required on the build host"
[[ -f "$SCRIPT_DIR/VINS-Fusion/vins_estimator/CMakeLists.txt" ]] \
  || die "local VINS-Fusion checkout not found: $SCRIPT_DIR/VINS-Fusion"

# Optional: build on a remote native-amd64 docker daemon over ssh (the amd64
# emulation path on Apple Silicon is ~10-30x slower and memory hungry). The
# local build context is streamed to the remote daemon; the binary is copied
# back over the same connection.
if [[ -n "$REMOTE" ]]; then
  case "$REMOTE" in
    *://*) DOCKER_HOST="$REMOTE" ;;
    *)     DOCKER_HOST="ssh://$REMOTE" ;;
  esac
  export DOCKER_HOST
  log "using remote docker daemon: $DOCKER_HOST"
  docker version --format 'connected: {{.Server.Os}}/{{.Server.Arch}} {{.Server.Version}}' \
    || die "cannot reach docker daemon at $DOCKER_HOST"
fi

# 1-3. docker build: conda-forge base + ROS1 noetic + the local VINS-Fusion
# checkout with our adapter.
log "building image $IMAGE:$TAG ($PLATFORM, -j${JOBS:-auto})"
BUILD_ARGS=(
  --platform "$PLATFORM"
  -f "$SCRIPT_DIR/Dockerfile"
  --build-arg "JOBS=$JOBS"
  -t "$IMAGE:$TAG"
)
if [[ "$FORCE" -eq 1 ]]; then
  BUILD_ARGS+=(--no-cache)
fi
docker build "${BUILD_ARGS[@]}" "$SCRIPT_DIR"

# 4. copy the built binary (and its bundled shared libs) out of the image to
# the local vins_adapter dir.
log "extracting /out/vins_adapter from the image"
CID="$(docker create --platform "$PLATFORM" "$IMAGE:$TAG")"
cleanup() { docker rm -f "$CID" >/dev/null 2>&1 || true; }
trap cleanup EXIT
# Start from a clean copy: replace only the artifacts this script generates so
# no stale binary/libs survive into the new self-contained bundle.
rm -rf "$OUT/lib" "$BIN" "$OUT/BUILDINFO.json"
mkdir -p "$OUT"
docker cp "$CID:/out/vins_adapter" "$BIN"
docker cp "$CID:/out/lib" "$OUT/lib"
docker rm -f "$CID" >/dev/null
trap - EXIT
chmod 0755 "$BIN"

# Launcher wrapper shipped inside the bundle/tarball: execs the binary, which
# resolves its bundled libraries via $ORIGIN/lib (no ROS/conda install needed).
cat > "$OUT/run_vins_adapter.sh" <<'EOF'
#!/usr/bin/env bash
# vins_adapter launcher: usage ./run_vins_adapter.sh <config.yaml> <output.tum>
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/vins_adapter" "$@"
EOF
chmod 0755 "$OUT/run_vins_adapter.sh"

IMAGE_ID="$(docker image inspect -f '{{.Id}}' "$IMAGE:$TAG")"
VINS_COMMIT="$(git -C "$SCRIPT_DIR/VINS-Fusion" rev-parse HEAD 2>/dev/null || echo unknown)"
VINS_REMOTE="$(git -C "$SCRIPT_DIR/VINS-Fusion" remote get-url origin 2>/dev/null || echo "local snapshot: $SCRIPT_DIR/VINS-Fusion")"
cat > "$OUT/BUILDINFO.json" <<JSON
{
  "target": "vins_adapter",
  "built_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "source_repo": "$VINS_REMOTE",
  "source_commit": "$VINS_COMMIT",
  "docker_image": "$IMAGE:$TAG",
  "docker_image_id": "$IMAGE_ID",
  "docker_platform": "$PLATFORM",
  "license": "GPLv3",
  "note": "built locally, never redistributed"
}
JSON

verify

# Self-contained tarball: binary + bundled libs + launcher + provenance. Drop
# it on any Linux x86_64 host with only glibc and run ./run_vins_adapter.sh.
TARBALL="$OUT.tar.gz"
tar -czf "$TARBALL" -C "$(dirname "$OUT")" "$(basename "$OUT")"
MD5="$(_md5 "$TARBALL")"
# md5sum output format, so receivers can verify with: md5sum -c <file>.md5
printf '%s  %s\n' "$MD5" "$(basename "$TARBALL")" > "$TARBALL.md5"
log "bundle: $TARBALL ($(du -h "$TARBALL" | cut -f1), md5 $MD5)"
log "bundle checksum: $TARBALL.md5"

# Regression tests: source guards + functional runs against the adapter that
# was just built (see tests/run_tests.py for what is covered).
if [[ "$SKIP_TESTS" -eq 0 && -f "$SCRIPT_DIR/tests/run_tests.py" ]]; then
  log "running regression tests"
  python3 "$SCRIPT_DIR/tests/run_tests.py" --adapter "$OUT/run_vins_adapter.sh" \
    || die "regression tests failed (see above)"
fi
log "done"
