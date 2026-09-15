#!/usr/bin/env python3
"""Regression tests for the offline VINS adapter.

Two layers:

1. Source guards (instant, no build needed): the four fixes that keep the
   adapter running standalone must not be reverted --
     - camodocal YAML written with a ``%YAML:1.0`` header (OpenCV >= 4.13
       rejects bare YAML in FileStorage::FORMAT_AUTO),
     - ``pubTF`` must not construct a TransformBroadcaster (blocks forever in
       the ROS-master XMLRPC retry loop when no roscore exists),
     - ``-DNDEBUG`` on the estimator build (otherwise publish() on the
       never-registered publishers aborts instead of no-oping),
     - estimator state pointers zero-initialized (clearState() used to delete
       garbage on construction -> segfault),
     - ``ESTIMATE_EXTRINSIC`` forced off without IMU (null pre_integrations
       deref in processImage).

2. Functional tests against the built adapter (skipped with --sources-only):
   argv contract, config error paths, undecodable-frame handling, temp-dir
   cleanup, TUM output validity, and clean exit on untrackable input.

Usage:
    python3 tests/run_tests.py [--adapter PATH] [--sources-only]

Exit code 0 = all tests passed, 1 = failures, 2 = nothing to test against.
Standard library only; test frames are hand-written PGM images, so neither
OpenCV nor PIL is required on the test host.
"""

from __future__ import annotations

import argparse
import glob
import os
import random
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ADAPTER_CPP = REPO / "adapter" / "vins_adapter.cpp"
ESTIMATOR_H = REPO / "VINS-Fusion" / "vins_estimator" / "src" / "estimator" / "estimator.h"
VISUALIZATION_CPP = REPO / "VINS-Fusion" / "vins_estimator" / "src" / "utility" / "visualization.cpp"
VINS_CMAKELISTS = REPO / "VINS-Fusion" / "vins_estimator" / "CMakeLists.txt"
DEFAULT_ADAPTER = REPO / "vins_adapter" / "run_vins_adapter.sh"

WIDTH, HEIGHT, FPS, N_FRAMES = 320, 240, 30, 12

failures: list[str] = []
passed = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global passed
    if ok:
        passed += 1
        print(f"  PASS {name}")
    else:
        failures.append(f"{name}: {detail}")
        print(f"  FAIL {name}  {detail}")


def section(title: str) -> None:
    print(f"\n== {title} ==")


# ---------------------------------------------------------------- frame utils


def _png_chunk(typ: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + typ + data + struct.pack(">I", zlib.crc32(typ + data) & 0xFFFFFFFF)


def make_frame(i: int, width: int = WIDTH, height: int = HEIGHT, noise: bool = False) -> bytes:
    """One 8-bit grayscale PNG frame: a bright square sweeping a textured field.

    Hand-rolled PNG (stdlib zlib only -- no OpenCV/PIL needed on the host), in
    the extension set the adapter accepts (.png).
    """
    rng = random.Random(1234)
    rows = []
    for y in range(height):
        row = bytearray(width)
        for x in range(width):
            if noise:
                v = rng.randrange(256)  # untrackable salt noise
            else:
                # Smooth gradient + checker texture: stable corners to track.
                base = (x * 255 // width + y * 255 // height) // 2
                tex = 60 if ((x // 16 + y // 16) % 2 == 0) else 0
                v = min(255, base // 2 + tex)
            sq_x = (i * 6) % (width - 48)
            sq_y = height // 3
            if sq_x <= x < sq_x + 40 and sq_y <= y < sq_y + 40:
                v = 255
            row[x] = v
        rows.append(b"\x00" + bytes(row))  # filter type 0 (None) per row
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", zlib.compress(bytes(b"".join(rows))))
        + _png_chunk(b"IEND", b"")
    )


def write_frames(directory: Path, n: int, noise: bool = False) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for i in range(n):
        (directory / f"{i:06d}.png").write_bytes(make_frame(i, noise=noise))


def write_config(
    path: Path,
    left_dir: Path,
    right_dir: Path | None = None,
    estimate_extrinsic: int = 2,
) -> None:
    """Adapter config in the harness layout (stereo extrinsics = identity)."""
    identity = "- [1.0, 0.0, 0.0, 0.0]\n- [0.0, 1.0, 0.0, 0.0]\n- [0.0, 0.0, 1.0, 0.0]\n"
    lines = [
        f"imu: 0",
        f"num_of_cam: {2 if right_dir else 1}",
        f"estimate_extrinsic: {estimate_extrinsic}",
        "max_features: 100",
        "fx: 300.0",
        "fy: 300.0",
        "cx: 160.0",
        "cy: 120.0",
        "k1: 0.0",
        "k2: 0.0",
        "p1: 0.0",
        "p2: 0.0",
        f"left_dir: {left_dir}",
    ]
    if right_dir:
        lines.append(f"right_dir: {right_dir}")
    lines.append("T_cam0_body:")
    lines.append(identity)
    if right_dir:
        lines.append("T_cam1_body:")
        lines.append(identity)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ------------------------------------------------------------- source guards


def test_source_guards() -> None:
    section("source guards (regression fixes present)")

    src = ADAPTER_CPP.read_text(encoding="utf-8")
    fn_start = src.find("void write_camodocal_pinhole")
    fn_end = src.find("\n}", fn_start)
    body = src[fn_start:fn_end]
    check(
        "camodocal yaml carries the %YAML:1.0 header",
        "%YAML:1.0" in body,
        "OpenCV >= 4.13 FileStorage::FORMAT_AUTO rejects bare YAML -> "
        "'Input file is invalid' abort in generateCameraFromYamlFile",
    )
    check(
        "ESTIMATE_EXTRINSIC forced off without IMU",
        "ESTIMATE_EXTRINSIC = use_imu ? estimate_extrinsic : 0;" in src,
        "null pre_integrations deref in processImage (estimator.cpp "
        "ESTIMATE_EXTRINSIC == 2 branch) for vision-only runs",
    )

    est = ESTIMATOR_H.read_text(encoding="utf-8")
    check(
        "pre_integrations zero-initialized",
        "pre_integrations[(WINDOW_SIZE + 1)] = {};" in est,
        "clearState() deletes uninitialized pointers on construction -> segfault",
    )
    check(
        "tmp_pre_integration initialized",
        "IntegrationBase *tmp_pre_integration = nullptr;" in est,
        "clearState() deletes uninitialized pointer -> segfault",
    )
    check(
        "last_marginalization_info initialized",
        "MarginalizationInfo *last_marginalization_info = nullptr;" in est,
        "clearState() deletes uninitialized pointer -> segfault",
    )

    vis = VISUALIZATION_CPP.read_text(encoding="utf-8")
    pubtf_start = vis.find("void pubTF")
    pubtf_end = vis.find("\n}", pubtf_start)
    pubtf = vis[pubtf_start:pubtf_end]
    ret = pubtf.find("    return;")
    br = pubtf.find("static tf::TransformBroadcaster")
    check(
        "pubTF returns before constructing a TransformBroadcaster",
        ret != -1 and br != -1 and ret < br,
        "TransformBroadcaster ctor -> ros::start() -> endless master XMLRPC "
        "retry loop when no roscore exists (adapter hang)",
    )

    cmake = VINS_CMAKELISTS.read_text(encoding="utf-8")
    check(
        "estimator built with -DNDEBUG",
        "-DNDEBUG" in cmake,
        "without NDEBUG, publish() on the never-registered publishers aborts "
        "(publisher.h ROS_ASSERT_MSG) instead of no-oping",
    )


# ----------------------------------------------------------- functional tests


def run_adapter(adapter: str, config: Path, out: Path, timeout: float = 240.0):
    return subprocess.run(  # noqa: S603 -- fixed argv, no shell
        [adapter, str(config), str(out)],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def tmp_dir_count() -> int:
    return len(glob.glob(os.path.join(tempfile.gettempdir(), "vins_adapter_*")))


def assert_valid_tum(name: str, tum: Path, allow_empty: bool) -> None:
    lines = [ln for ln in tum.read_text(encoding="utf-8").splitlines() if ln.strip()]
    if not lines:
        check(f"{name}: tum empty-or-valid", allow_empty, "empty TUM not allowed for this case")
        return
    stamps = []
    ok = True
    for ln in lines:
        parts = ln.split()
        if len(parts) != 8:
            ok = False
            break
        try:
            stamps.append(float(parts[0]))
        except ValueError:
            ok = False
            break
    check(f"{name}: tum rows are 8 floats", ok)
    check(f"{name}: tum stamps non-decreasing", all(a <= b for a, b in zip(stamps, stamps[1:])))


def test_functional(adapter: str) -> None:
    section(f"functional tests ({adapter})")
    if not (os.path.isfile(adapter) and os.access(adapter, os.X_OK)):
        print("  SKIP adapter not built/found")
        return

    work = Path(tempfile.mkdtemp(prefix="vins_adapter_tests_"))
    try:
        # argv contract: no args is a usage error, exit 2 (smoke-run contract).
        proc = subprocess.run([adapter], capture_output=True, text=True)  # noqa: S603
        check("usage: no args exits 2", proc.returncode == 2, f"exit={proc.returncode}")

        # unreadable config
        proc = run_adapter(adapter, work / "nope.yaml", work / "o.tum")
        check(
            "unreadable config fails with a message",
            proc.returncode != 0 and "cannot read config" in proc.stderr,
            f"exit={proc.returncode} stderr={proc.stderr[:120]!r}",
        )

        # missing left_dir
        cfg = work / "missing_dir.yaml"
        write_config(cfg, work / "does_not_exist")
        proc = run_adapter(adapter, cfg, work / "o.tum")
        check(
            "missing left_dir fails",
            proc.returncode == 1 and proc.stderr.strip() != "",
            f"exit={proc.returncode} stderr={proc.stderr[:120]!r}",
        )

        # empty left dir
        empty = work / "empty_left"
        empty.mkdir()
        cfg = work / "empty_dir.yaml"
        write_config(cfg, empty)
        proc = run_adapter(adapter, cfg, work / "o.tum")
        check("empty left_dir fails", proc.returncode == 1, f"exit={proc.returncode}")

        # bad transform
        frames = work / "mono"
        write_frames(frames, N_FRAMES)
        cfg = work / "bad_transform.yaml"
        write_config(cfg, frames)
        cfg.write_text(cfg.read_text(encoding="utf-8").replace("T_cam0_body:", "T_cam0_missing:"), encoding="utf-8")
        proc = run_adapter(adapter, cfg, work / "o.tum")
        check(
            "missing T_cam0_body fails",
            proc.returncode == 1 and "T_cam0_body" in proc.stderr,
            f"exit={proc.returncode} stderr={proc.stderr[:120]!r}",
        )

        # one undecodable frame mid-sequence: graceful failure, no crash,
        # scratch dir still cleaned up.
        before = tmp_dir_count()
        bad = work / "bad_frame"
        write_frames(bad, 4)
        (bad / "000002.png").write_bytes(b"not an image at all")
        cfg = work / "bad_frame.yaml"
        write_config(cfg, bad)
        proc = run_adapter(adapter, cfg, work / "o.tum")
        check(
            "undecodable frame fails gracefully (exit 1, not a signal)",
            proc.returncode == 1 and "cannot decode" in proc.stderr,
            f"exit={proc.returncode} (-1073/-11 style = crash) stderr={proc.stderr[:120]!r}",
        )
        check(
            "undecodable frame: scratch dir cleaned up",
            tmp_dir_count() <= before,
            f"before={before} after={tmp_dir_count()}",
        )

        # core runs: mono and stereo, vision-only (no IMU). These must exit 0
        # cleanly on trackable frames AND on untrackable ones -- the original
        # segfault reproduced exactly on the noise case, and the
        # ESTIMATE_EXTRINSIC / NDEBUG regressions crashed the stereo path.
        before = tmp_dir_count()
        cases = {
            "mono_vision_only": (work / "mono", None, False),
            "stereo_vision_only": (work / "stereo_l", work / "stereo_r", False),
            "mono_untrackable_noise": (work / "noise", None, True),
        }
        for name, (left, right, noise) in cases.items():
            if noise:
                write_frames(left, 8, noise=True)
            else:
                write_frames(left, N_FRAMES)
            if right:
                write_frames(right, N_FRAMES, noise=noise)
            cfg = work / f"{name}.yaml"
            write_config(cfg, left, right)
            tum = work / f"{name}.tum"
            proc = run_adapter(adapter, cfg, tum)
            check(f"{name}: exits 0", proc.returncode == 0, f"exit={proc.returncode} stderr={proc.stderr[-200:]!r}")
            if proc.returncode == 0:
                assert_valid_tum(name, tum, allow_empty=True)
            check(f"{name}: scratch dir cleaned up", tmp_dir_count() <= before, f"after={tmp_dir_count()}")
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--adapter", default=str(DEFAULT_ADAPTER), help="path to run_vins_adapter.sh")
    ap.add_argument("--sources-only", action="store_true", help="skip the functional (subprocess) tests")
    args = ap.parse_args()

    for path in (ADAPTER_CPP, ESTIMATOR_H, VISUALIZATION_CPP, VINS_CMAKELISTS):
        if not path.is_file():
            print(f"source file missing: {path}", file=sys.stderr)
            return 2

    test_source_guards()
    if not args.sources_only:
        test_functional(args.adapter)

    print(f"\n{passed} passed, {len(failures)} failed")
    for f in failures:
        print(f"  FAILED: {f}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
