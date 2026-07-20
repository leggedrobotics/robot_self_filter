# robot_self_filter optimization goal

## Outcome

Make `robot_self_filter` measurably faster, better tested, and more reliable for Moleworks LiDAR workloads while preserving its supported ROS 2 interfaces and filtering semantics, then publish the verified work as the backward-compatible `v1.1.0` GitHub source release for ROS 2 Jazzy.

## Baseline

- Repository: `/home/lorenzo/moleworks/ros2_ws/src/robot_self_filter`
- Starting revision: `8910134` (`main`, clean at goal creation)
- The package builds libraries and the `self_filter` executable, but CMake registers no automated tests or benchmarks; `src/test_filter.cpp` is disabled and is an interactive node rather than a regression test.
- The filtering hot path performs point-by-body containment and ray-intersection work. It currently constructs a hit vector inside the body loop for every ray candidate.
- The node currently emits two `INFO` log messages for every input cloud and republishes collision markers for every cloud.
- No reproducible correctness corpus or performance baseline exists yet.
- The existing `v1.0.0` “ROS 2 Humble” release is a GitHub source release. The package is not present in any ROS distribution index and no bloom release repository exists, so the Jazzy release follows the repository's source-release convention rather than claiming ROS buildfarm binary availability.

## Constraints and non-goals

- Preserve the public ROS 2 topics, parameters, supported point-field layouts, URDF collision-shape behavior, and exported library API unless a change is explicitly justified, documented, and approved.
- Preserve correct handling of organized/unorganized clouds, `invert`, removed-point representation, sensor-distance filtering, transforms, and sphere/box/cylinder/mesh geometry.
- Do not weaken tests, discard difficult benchmark cases, hide failures, or compare different build modes, machines, inputs, or runtime conditions.
- Do not tune Mole robot padding/config values merely to make metrics look better.
- Broad rewrites remain outside this goal. The user's 2026-07-20 request explicitly authorizes the commit, push, tag, and GitHub release actions required for the Jazzy source release; publishing to the ROS buildfarm through bloom/rosdistro remains outside scope.

## Primary verifier

A deterministic benchmark suite, run on the same machine with the same Release build and fixed inputs, must show:

1. at least 20% improvement in aggregate median filtering throughput over revision `8910134` across representative small, medium, and large point clouds; and
2. no individual representative case regresses by more than 5% in median throughput; and
3. optimized and baseline classifications/output clouds are equivalent on the correctness corpus, except for deliberately approved bug fixes captured by regression tests.

Record the exact build command, CPU/runtime context, benchmark command, raw results, aggregation method, and baseline/optimized revisions in `RESULT.md`. Use warm-up runs and at least 10 measured repetitions per case.

## Supporting checks

- Add automated tests that cover geometry containment/intersection and filter output behavior, including boundary points, NaNs/non-finite points, organized clouds, `invert`, zero-versus-NaN replacement, empty clouds, and representative custom LiDAR point layouts.
- Add regression tests for every correctness or reliability defect fixed during the work.
- Run the package's full test suite successfully from a clean build.
- Run a ROS-level smoke test proving that the node starts, accepts a representative `PointCloud2` with valid TF/robot description, publishes a valid filtered cloud, and shuts down cleanly.
- Run available compiler warnings and sanitizer checks over the exercised unit-test paths; resolve goal-related findings or document why they are not actionable.
- Confirm that parameter/config parsing and the package's installed/exported targets still work for a downstream consumer.
- Ensure hot-path diagnostics are bounded or configurable and do not materially distort benchmark results.
- Update the package version and changelog for `1.1.0`, document Jazzy compatibility, and verify the source archive can be built and tested in the current ROS 2 Jazzy environment without relying on an overlay copy of the package.
- Publish a non-prerelease GitHub release tagged `v1.1.0`, targeted at the verified commit, with release notes that accurately state compatibility, test/benchmark evidence, changes, installation from source, and known limitations.

## Iteration loop

1. Build the unmodified baseline and create the deterministic correctness corpus and benchmark harness.
2. Capture baseline correctness, throughput, allocations where practical, and runtime profiles.
3. Select one evidence-backed hot spot or reliability defect at a time.
4. Implement the smallest coherent change, then rerun focused correctness tests and the relevant benchmark.
5. Keep changes only when evidence shows correctness and an aggregate benefit; investigate or revert regressions.
6. Periodically run the full supporting checks and update `WORKLOG.md` with commands, results, failures, and the next action.
7. Finish only after a clean-state reproduction of the primary verifier and all supporting checks.

## Approval gates

Ask before changing public APIs or filtering semantics, modifying robot-specific filter configuration, publishing through bloom/rosdistro, or performing tests on robot hardware. The commit, push, tag, and GitHub source release needed for `v1.1.0` are authorized by the user's explicit release request.

## Blocker standard

A blocker must be an external condition that prevents meaningful progress after safe local alternatives have been exhausted. Record the evidence, preserved partial work, and smallest user or external action needed. Difficulty, a failed experiment, or an unmet performance target is not by itself a blocker.

## Completion proof

Before marking this goal complete, `RESULT.md` must contain:

- final revision/diff summary and the defects or hot spots addressed;
- exact clean build and test commands with passing summaries;
- baseline and optimized raw benchmark artifacts plus the calculated per-case and aggregate changes;
- correctness-equivalence result and any explicitly approved semantic differences;
- ROS smoke-test evidence and downstream export/consumer check;
- sanitizer/warning results;
- remaining risks and limitations.
- the pushed commit SHA, `v1.1.0` tag readback, and public GitHub release URL/readback.
