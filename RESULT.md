# robot_self_filter v1.1.0 result

## Release identity

- Baseline: `8910134`
- Verified release commit: `65f5b01ebb7a24ef88b5a79febaed62ae326c510`
- Annotated tag: `v1.1.0` (tag object `d611dca49ba6120aac4c9a37b765f7c29e5cc060`)
- Release: <https://github.com/leggedrobotics/robot_self_filter/releases/tag/v1.1.0>
- Published: 2026-07-20 09:52:52 UTC, non-draft, non-prerelease, and returned by GitHub's latest-release endpoint
- Delivery: GitHub source release for ROS 2 Jazzy; no bloom/rosdistro or apt availability is claimed

Remote readback showed `origin/main` and peeled `refs/tags/v1.1.0^{}` both at the verified release commit. GitHub's tag API also resolves the annotated tag to that commit.

## Outcome and diff

The release meets the optimization goal: aggregate median throughput improved by 27.395%, every representative case improved by at least 26.213%, and all classification checksums remain identical. The diff from baseline is 22 files, 1,779 insertions, and 141 deletions.

Measured hot spots and reliability defects addressed:

- reduced oriented-box ray projection work while retaining randomized equivalence to the prior slab algorithm;
- reused per-ray hit storage and added per-body scaled/unscaled bounding-sphere culling;
- bounded hot-path diagnostics and skipped unsubscribed collision-marker construction;
- preserved PCL metadata/density and deterministically initialized removed custom-layout fields;
- respected `use_sim_time` and subscription queue overrides, handled unknown sensor types, and matched prefixed TF frames to URDF suffixes;
- validated mesh topology, indices, sizes, and packed binary STL input before reading;
- removed an unused duplicate TF listener that caused intermittent heap corruption/SIGSEGV during rapid SIGINT shutdown;
- corrected direct dependencies/exports and added deterministic unit, smoke, benchmark, and downstream-consumer coverage.

No supported topic, parameter, point layout, geometry classification, or exported library API was deliberately removed. `publish_collision_shapes` is the only new parameter and defaults to the prior enabled behavior.

## Environment

- ROS 2 Jazzy, Ubuntu 24.04, x86_64
- Intel Core Ultra 7 258V, 8 cores / 8 threads
- GCC 13.3.0, CMake 3.28.3
- Identical machine, Release mode, deterministic inputs, 8 warmups, and 21 measured iterations for baseline and optimized benchmark artifacts

## Build and tests

The strict-warning candidate used:

```bash
colcon --log-base <run>/log build \
  --base-paths /home/lorenzo/moleworks/ros2_ws/src/robot_self_filter \
  --packages-select robot_self_filter \
  --build-base <run>/build --install-base <run>/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  '-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic -Wconversion -Wshadow'
colcon --log-base <run>/test-log test \
  --build-base <run>/build --install-base <run>/install \
  --packages-select robot_self_filter --return-code-on-test-failure
```

Result: clean build with no compiler diagnostics; 8 geometry/mesh tests, 11 filter tests, and 1 installed-node ROS smoke test passed with zero failures or skips. `colcon test-result` reports 23 XML records because it includes the three CTest wrappers in addition to the 20 underlying cases.

The smoke test starts the installed `self_filter` executable, confirms wall time is not overridden, publishes a stamped two-point `PointCloud2`, receives the expected stamped one-point output, and verifies clean SIGINT shutdown. Ten consecutive Release smoke runs pass after the duplicate-listener correction.

Coverage includes sphere/box/cylinder/mesh containment and intersections, randomized optimized/legacy box equivalence, boundary points, NaNs, empty inputs, collision origins/scales, prefixed links, organized/unorganized clouds, invert, zero/NaN replacement, mask mismatch, metadata, and Ouster/Hesai/Robosense/Pandar layouts.

## Benchmark verifier

Raw artifacts:

- `benchmark/results/baseline_8910134.csv`
- `benchmark/results/optimized_v1.1.0.csv`

Verifier command:

```bash
python3 test/compare_benchmarks.py \
  benchmark/results/baseline_8910134.csv \
  benchmark/results/optimized_v1.1.0.csv
```

| Case | Points | Baseline points/s | v1.1.0 points/s | Change | Classification checksum |
| --- | ---: | ---: | ---: | ---: | --- |
| small | 20,000 | 7,250,143.734 | 9,401,054.140 | +29.667% | `10260681419905805875` |
| medium | 100,000 | 7,425,242.841 | 9,371,619.774 | +26.213% | `15395350392396729893` |
| large | 400,000 | 7,422,434.538 | 9,377,044.489 | +26.334% | `18411120811840622729` |

Aggregation is the geometric mean of per-case median-throughput ratios: **+27.395%**. The verifier exits zero only when every checksum is identical, no case regresses by more than 5%, every case has at least 10 measured samples, and aggregate improvement is at least 20%.

## Sanitizers, lint, and downstream export

- ASan and UBSan passed all 20 exercised paths. LeakSanitizer passed all 19 C++ unit cases.
- The process-level ROS smoke passed five consecutive ASan/UBSan runs with leak detection disabled. Enabling process leak detection reports approximately 257 kB retained by ROS Jazzy `librcl` type-description caches during shutdown; stacks point into `/opt/ros/jazzy`, not package allocation sites, so this upstream runtime finding is documented rather than hidden as a package fix.
- `ament_xmllint`, `ament_flake8`, `rosdep check --from-paths <repo> --ignore-src`, and `git diff --check` pass.
- A standalone CMake project found installed `robot_self_filter` version 1.1.0 from the release archive prefix, linked its exported targets, ran successfully, and did not use the workspace package overlay.

## Source archive proof

The exact merged release commit was archived with `git archive`, extracted outside the repository, and built/tested after unsetting `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`, `LD_LIBRARY_PATH`, and `PYTHONPATH`, then sourcing only `/opt/ros/jazzy/setup.bash`. The gate asserted `AMENT_PREFIX_PATH=/opt/ros/jazzy` and rejected any workspace overlay path.

```bash
git archive --format=tar --output robot_self_filter-v1.1.0.tar 65f5b01
colcon build --base-paths <archive-source> --packages-select robot_self_filter \
  --build-base <archive-run>/build --install-base <archive-run>/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --build-base <archive-run>/build --install-base <archive-run>/install \
  --packages-select robot_self_filter --return-code-on-test-failure
```

Result: build passed, all 20 underlying checks passed, an additional 10 consecutive process-smoke runs passed, and the verified tar SHA-256 is `312668cf5d01592e2be4bba72b8cccb7cff71188547cc1100fea64313989c1ff`.

## Remaining risks and limitations

- Performance evidence is deterministic and reproducible but synthetic and measured on one CPU; real sensor rates, robot geometry, TF load, and middleware traffic will affect absolute throughput.
- No robot hardware test was performed or required; ROS process-level behavior was exercised locally.
- Mesh-specific scale/padding remains unsupported, as before.
- This is a GitHub source release only. Adding the package to rosdistro and publishing buildfarm binaries remains separate work.
