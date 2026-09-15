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
#   --platform PLAT   docker platform (default: auto -- the cpu arch of the
#                     machine that compiles: linux/amd64 on x86_64 hosts,
#                     linux/arm64 on arm64 hosts, or the remote daemon's arch
#                     with --remote; override to cross-build)
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
# Architecture (x86 vs arm)
# -------------------------
# The build target follows the cpu arch of the machine that does the compiling,
# so the resulting ELF runs natively there instead of under qemu emulation:
#   x86_64 host          -> linux/amd64
#   arm64/aarch64 host   -> linux/arm64   (Apple Silicon included: a Rosetta
#                           shell reports x86_64, the sysctl probe below still
#                           resolves it to arm64)
#   --remote HOST        -> the remote daemon's arch, queried with
#                           `docker version --format '{{.Server.Arch}}'`
# Pass --platform linux/<arch> to cross-build (e.g. amd64 on an arm64 mac,
# ~10-30x slower under emulation). The arch is recorded in BUILDINFO.json and
# in the shipped artifact names, so an x86 and an arm bundle can sit side by
# side without overwriting each other:
#   vins_adapter-linux-x86_64.tar.gz   (+ .md5)
#   vins_adapter-linux-aarch64.tar.gz  (+ .md5)
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
# produced at vins-adapter/vins_adapter-linux-<arch>.tar.gz (<arch> = x86_64 or
# aarch64, the build's target cpu arch) for shipping to workers, with an md5
# checksum beside it (vins_adapter-linux-<arch>.tar.gz.md5, md5sum -c
# compatible); verify after transfer with:
#   md5sum -c vins_adapter-linux-x86_64.tar.gz.md5
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
PLATFORM=""            # empty = auto: arch of the host that compiles
REMOTE=""
REMOTE_ARCH=""         # arch reported by a --remote docker daemon
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
    -h|--help)  awk 'NR==1{next} /^#/{sub(/^#+ ?/,""); print; next} {exit}' "$0"; exit 0 ;;
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
  # Standalone check 0: the ELF must be the arch we built for. Catches a stale
  # binary from a previous arch or a mis-set --platform. Only fatal when the
  # header identifies a *different* arch; unreadable headers just warn.
  local elf=""
  if command -v file >/dev/null; then
    elf="$(file -b "$BIN" 2>/dev/null || true)"
    local want=""
    case "$PKG_ARCH" in
      x86_64)   want='x86-64' ;;
      aarch64)  want='aarch64' ;;
      armv7)    want='ARM' ;;
    esac
    if [[ -n "$want" && -n "$elf" ]]; then
      if [[ "$elf" == *"$want"* ]]; then
        log "arch: $PKG_ARCH ELF confirmed"
      else
        case "$elf" in
          *x86-64*|*80386*|*aarch64*|*ARM*)
            die "arch mismatch: $BIN is '$(echo "$elf" | cut -d, -f2-)', expected $PKG_ARCH" ;;
        esac
        log "arch: could not confirm $PKG_ARCH from '$elf'"
      fi
    fi
  fi
  # Standalone checks 1-2 need to *execute* the ELF, which only works when this
  # host is Linux and matches the artifact's arch (not on macOS, not for a
  # cross-arch build).
  local can_run=0
  if [[ "$(uname -s)" == "Linux" && "$PKG_ARCH" == "$(arch_to_pkg "$(detect_host_arch 2>/dev/null || echo unknown)")" ]]; then
    can_run=1
  fi
  # Standalone check 1: every shared dep must resolve to the bundled
  # $ORIGIN/lib or host glibc -- a "not found" here means a lib was missed
  # during bundling and the copy would break on a clean host.
  if [[ "$can_run" -eq 1 ]] && command -v ldd >/dev/null; then
    if ldd "$BIN" 2>&1 | grep -q 'not found'; then
      ldd "$BIN" 2>&1 | grep 'not found' >&2
      die "unresolved shared deps (bundling incomplete)"
    fi
    log "standalone: all shared deps resolved ($(ls "$OUT/lib" 2>/dev/null | wc -l) bundled libs in lib/)"
  fi
  # Standalone check 2: run it. No args = usage error, exit 2; this proves
  # the binary starts and links on this host with no ROS/conda installed.
  if [[ "$can_run" -eq 1 ]]; then
    local rc=0
    "$BIN" 2>/dev/null || rc=$?
    [[ "$rc" -eq 2 ]] || die "smoke run failed (exit $rc, expected 2) -- binary does not run standalone on this host"
    log "standalone: smoke run OK"
  else
    log "standalone: smoke run skipped (host $(uname -s)/$(uname -m) cannot execute a linux/$PKG_ARCH ELF)"
  fi
  if [[ -f "$OUT/BUILDINFO.json" ]]; then
    log "provenance: $(tr -d '\n' < "$OUT/BUILDINFO.json")"
  fi
  # Bundle integrity: if a tarball and its checksum ship together, the shipped
  # artifact is what was last built (catches transfer corruption / stale pairs).
  local tarball="$OUT-linux-$PKG_ARCH.tar.gz"
  [[ -f "$tarball" ]] || tarball="$OUT.tar.gz"   # pre-arch-naming bundle
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

# Architecture naming, three spellings of the same thing:
#   ARCH      docker/go name:  amd64 | arm64
#   PLATFORM  docker platform: linux/amd64 | linux/arm64
#   PKG_ARCH  artifact name:   x86_64 | aarch64   (in tarball/md5/BUILDINFO)

# Arch of the machine running this script. uname lies under macOS Rosetta (an
# x86_64 shell on an arm64 cpu), so probe sysctl there: proc_translated=1 means
# the cpu is arm64 even though uname says x86_64.
detect_host_arch() {
  local m
  m="$(uname -m)"
  case "$m" in
    x86_64|amd64)
      if [[ "$(uname -s)" == "Darwin" \
            && "$(sysctl -n sysctl.proc_translated 2>/dev/null || echo 0)" == "1" ]]; then
        echo arm64
      else
        echo amd64
      fi ;;
    aarch64|arm64) echo arm64 ;;
    *) die "unsupported host cpu arch '$m' -- pass --platform linux/<arch> explicitly" ;;
  esac
}

arch_to_platform() {
  case "$1" in
    amd64|x86_64)    echo linux/amd64 ;;
    arm64|aarch64)   echo linux/arm64 ;;
    arm/v7|armv7|armv7l) echo linux/arm/v7 ;;
    *) echo "linux/$1" ;;
  esac
}

arch_to_pkg() {
  case "$1" in
    amd64|x86_64)         echo x86_64 ;;
    arm64|aarch64)        echo aarch64 ;;
    arm/v7|armv7|armv7l)  echo armv7 ;;
    *) echo "$1" ;;
  esac
}

# What to build for: --platform wins; else the remote daemon's arch; else this
# host's cpu arch.
resolve_arch() {
  case "$PLATFORM" in
    "") [[ -n "$REMOTE_ARCH" ]] && { echo "$REMOTE_ARCH"; return; }
        detect_host_arch ;;
    linux/amd64|linux/x86_64|amd64|x86_64)        echo amd64 ;;
    linux/arm64|linux/aarch64|arm64|aarch64)      echo arm64 ;;
    linux/arm/v7|arm/v7|armv7|armv7l)             echo arm/v7 ;;
    *) die "cannot map --platform '$PLATFORM' to a build arch (known: linux/amd64, linux/arm64, linux/arm/v7)" ;;
  esac
}

# Arch of an already-built copy: BUILDINFO.json first (written by this script),
# then the ELF header. Empty when neither is readable.
detect_copy_arch() {
  local a
  if [[ -f "$OUT/BUILDINFO.json" ]]; then
    a="$(sed -n 's/.*"arch"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' \
          "$OUT/BUILDINFO.json" | head -n1)"
    [[ -n "$a" ]] && { echo "$a"; return; }
  fi
  if command -v file >/dev/null && [[ -f "$BIN" ]]; then
    case "$(file -b "$BIN" 2>/dev/null)" in
      *x86-64*|*80386*)  echo x86_64;  return ;;
      *aarch64*)         echo aarch64; return ;;
      *ARM*)             echo aarch64; return ;;
    esac
  fi
  echo ""
}

if [[ "$CHECK" -eq 1 ]]; then
  # No build: read the arch off the existing copy so --check verifies the
  # matching arch-named tarball.
  PKG_ARCH="$(arch_to_pkg "$(detect_copy_arch)")"
  [[ -n "$PKG_ARCH" ]] || PKG_ARCH="unknown"
  verify
  exit 0
fi

command -v docker >/dev/null || die "docker is required on the build host"
[[ -f "$SCRIPT_DIR/VINS-Fusion/vins_estimator/CMakeLists.txt" ]] \
  || die "local VINS-Fusion checkout not found: $SCRIPT_DIR/VINS-Fusion"

# Optional: build on a remote docker daemon over ssh (native arch, no emulation:
# the amd64-on-arm64 path is ~10-30x slower and memory hungry). The local build
# context is streamed to the remote daemon; the binary is copied back over the
# same connection. The daemon's own arch becomes the build target.
if [[ -n "$REMOTE" ]]; then
  case "$REMOTE" in
    *://*) DOCKER_HOST="$REMOTE" ;;
    *)     DOCKER_HOST="ssh://$REMOTE" ;;
  esac
  export DOCKER_HOST
  log "using remote docker daemon: $DOCKER_HOST"
  REMOTE_ARCH="$(docker version --format '{{.Server.Arch}}' 2>/dev/null || true)"
  [[ -n "$REMOTE_ARCH" ]] || die "cannot reach docker daemon at $DOCKER_HOST"
  log "remote builder arch: $REMOTE_ARCH"
fi

# Resolve the build target from the cpu arch of the machine that compiles:
# local host (Apple Silicon included) or the remote daemon. --platform
# overrides it for cross builds.
ARCH="$(resolve_arch)"
# Re-spell the platform (so a bare --platform amd64 works too, not just
# linux/amd64) and derive the artifact arch used in file names.
PLATFORM="$(arch_to_platform "$ARCH")"
PKG_ARCH="$(arch_to_pkg "$ARCH")"
HOST_ARCH="$(detect_host_arch 2>/dev/null || echo unknown)"
log "arch: this host $(uname -s | tr '[:upper:]' '[:lower:]')/$(arch_to_pkg "$HOST_ARCH") -> build $PLATFORM (artifact arch $PKG_ARCH)"

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
  "arch": "$PKG_ARCH",
  "host_arch": "$(arch_to_pkg "$HOST_ARCH")",
  "license": "GPLv3",
  "note": "built locally, never redistributed"
}
JSON

verify

# Self-contained tarball: binary + bundled libs + launcher + provenance. Drop it
# on any Linux host of the same arch ($PKG_ARCH) with only glibc and run
# ./run_vins_adapter.sh. The arch is in the file name (and in both md5 file name
# and content), so x86_64 and aarch64 bundles can coexist and a receiver can
# tell them apart without unpacking.
TARBALL="$OUT-linux-$PKG_ARCH.tar.gz"
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
