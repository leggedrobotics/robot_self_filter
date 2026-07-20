# robot_self_filter optimization worklog

## 2026-07-20 — Goal grounding

- Confirmed repository is on clean `main` at `8910134`.
- Inspected package documentation, build configuration, node callback, filter output path, and mask containment/intersection loops.
- Observed that no automated test or benchmark target is enabled.
- Identified initial measurement candidates rather than assumed fixes: per-cloud PCL conversions, per-point/per-body ray-hit allocation, collision-marker publication on every callback, and per-cloud `INFO` logging.
- Built the package successfully in Release mode with ROS 2 Jazzy using isolated build/install/log directories under `/home/lorenzo/moleworks/.codex_runs/robot_self_filter_baseline_8910134`.
- Baseline build command: `colcon --log-base <run>/log build --base-paths /home/lorenzo/moleworks/ros2_ws/src/robot_self_filter --packages-select robot_self_filter --build-base <run>/build --install-base <run>/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON`.
- The build completed successfully in 37.7 seconds. It emitted one developer warning from PCL's `FindFLANN.cmake` about unset CMake policy `CMP0144` and inherited `FLANN_ROOT=/usr`.
- Ran the baseline test command against the isolated build. Result: `0 tests, 0 errors, 0 failures, 0 skipped`.
- Next: add a deterministic correctness test target and benchmark harness, then capture revision `8910134` results before changing the filtering hot path.

## 2026-07-20 — Jazzy release scope

- User explicitly requested a new release for ROS 2 Jazzy.
- Inspected GitHub and rosdistro state. The only GitHub release is `v1.0.0`, named “Release v1.0.0 - ROS 2 Humble”; it is a source release with clone/build instructions.
- Confirmed `robot_self_filter` is absent from Humble, Iron, Jazzy, Kilted, and Rolling distribution indexes and that no `ros2-gbp/robot_self_filter-release` repository exists.
- Following repository precedent, added a `v1.1.0` Jazzy GitHub source release to the goal. Bloom/rosdistro publication is explicitly not claimed.

## 2026-07-20 — Correctness corpus and baseline

- Added deterministic unit coverage for primitive and mesh geometry, ray/containment boundaries, NaN and empty inputs, collision origins and scaling, prefixed TF frames, organized/inverted output, zero-versus-NaN replacement, metadata preservation, mask-size validation, and Ouster/Hesai/Robosense/Pandar layouts.
- Added an installed-executable ROS smoke test that starts `self_filter`, verifies the default wall clock is retained, publishes a stamped two-point `PointCloud2`, receives the expected one-point filtered output, and verifies clean SIGINT shutdown.
- Added `robot_self_filter_benchmark` with fixed small (20,000), medium (100,000), and large (400,000) workloads, 8 warmups, 21 measured iterations, raw per-iteration CSV output, and deterministic classification checksums.
- Captured the unoptimized `8910134` baseline in `benchmark/results/baseline_8910134.csv`: small 7,250,143.734 points/s, medium 7,425,242.841 points/s, and large 7,422,434.538 points/s.
- Profiling identified `Box::intersectsRay` and per-ray hit-vector allocation as the dominant measured work. `gprof` attributed 50% of sampled time to box ray intersections; the benchmark exercised approximately 6.9 million calls. `perf` was unavailable in this environment.

## 2026-07-20 — Optimizations and reliability fixes

- Reworked oriented-box ray slabs in the box's local basis, cutting projection work while retaining a 20,000-sample randomized equivalence test against the previous algorithm.
- Reused ray-hit storage and added scaled/unscaled per-body bounding-sphere rejection before detailed containment checks.
- Removed per-cloud `INFO` logging, bounded debug output, and avoided collision-marker construction when disabled or unsubscribed. Extended marker publication to all supported point types.
- Preserved PCL sensor metadata and density semantics, value-initialized removed custom-layout points, and rejected mismatched output masks.
- Stopped overriding `use_sim_time`, applied `max_queue_size` to the subscription QoS, added graceful unknown-sensor fallback, and supported prefixed TF link names with suffix-based URDF matching.
- Added mesh topology/index/resource-size validation and RAII file storage. Final review also found and fixed pre-validation and unaligned reads in the binary STL parser, with a valid/truncated-buffer regression test.
- Cleaned direct dependency declarations/exports and added a standalone downstream CMake consumer.

## 2026-07-20 — Final local verification

- Environment: ROS 2 Jazzy on x86_64 Ubuntu 24.04, Intel Core Ultra 7 258V (8 cores), GCC 13.3.0, CMake 3.28.3.
- Final isolated Release build used `colcon --log-base <run>/log build --base-paths <repo> --packages-select robot_self_filter --build-base <run>/build --install-base <run>/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON '-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic -Wconversion -Wshadow'`. It completed without compiler diagnostics.
- Full tests passed: 8 geometry/mesh tests, 11 filter tests, and 1 process-level ROS smoke test; zero failures or skips. `colcon test-result` also reports three CTest wrapper records, yielding 23 XML records in total.
- Final raw benchmark artifact `benchmark/results/optimized_v1.1.0.csv` records small 9,401,054.140 points/s (+29.667%), medium 9,371,619.774 (+26.213%), and large 9,377,044.489 (+26.334%). The geometric mean improvement is 27.395%; every classification checksum is identical to baseline and no case regressed.
- ASan and UBSan passed all exercised paths. LeakSanitizer also passed all 19 C++ unit cases. Leak detection was disabled only for the process-level smoke because ROS Jazzy reports shutdown allocations in `librcl` type-description caches; the smoke passes under ASan/UBSan, passes cleanly without instrumentation, and produced no package-owned sanitizer finding.
- A fresh standalone consumer found installed package version 1.1.0 from the isolated prefix, configured, linked, and ran successfully. `rosdep check`, `ament_xmllint`, `ament_flake8`, and `git diff --check` passed.
- Next: publish through the authorized branch/PR workflow, build and test a source-only archive of the merged commit, tag `v1.1.0`, create the public Jazzy source release, and record remote readback in `RESULT.md`.

## 2026-07-20 — Archive gate and shutdown correction

- Merged the primary implementation through PR #8 as `98f5c9e`, then generated a source-only archive and built it with `AMENT_PREFIX_PATH=/opt/ros/jazzy` and no workspace install paths.
- Repeating the installed-node smoke exposed an intermittent shutdown crash after SIGINT (`malloc_consolidate(): unaligned fastbin chunk detected` or SIGSEGV). This had escaped the initial single-run gate, so release publication remained paused.
- The node constructed an unused TF buffer/listener while the active templated filter already owned the TF listener used by `SelfMask`. Removing that duplicate listener eliminated the competing hidden TF executor/thread without changing the active transform path.
- The corrected strict-warning Release build passes the full 20-check suite and 10 consecutive process-level smoke runs. The corrected sanitizer build passes all C++ unit cases with ASan/UBSan/LSan and five consecutive smoke runs with ASan/UBSan.
- Benchmark outputs and classification checksums remain unchanged because the correction only removes unused node initialization and shutdown state.
- Next: merge the shutdown correction, regenerate the archive from the new `main` commit, and complete the tag/release readback.

## 2026-07-20 — Release complete

- Merged the shutdown correction through PR #9 as `65f5b01ebb7a24ef88b5a79febaed62ae326c510`.
- Generated a fresh archive from that exact commit, built/tested it with only `/opt/ros/jazzy` in `AMENT_PREFIX_PATH`, and passed the full suite plus 10 consecutive smoke runs. Archive SHA-256: `312668cf5d01592e2be4bba72b8cccb7cff71188547cc1100fea64313989c1ff`.
- Reconfirmed the standalone downstream consumer against the archive install and reran XML/Python lint, dependency resolution, and whitespace checks successfully.
- Pushed annotated tag `v1.1.0`; its peeled remote ref is the verified commit. Published the non-draft, non-prerelease latest GitHub release at <https://github.com/leggedrobotics/robot_self_filter/releases/tag/v1.1.0>.
- Recorded the complete proof, exact commands, raw metrics, remote readback, and limitations in `RESULT.md`.
