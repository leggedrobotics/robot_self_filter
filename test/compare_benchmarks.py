#!/usr/bin/env python3

"""Compare deterministic robot_self_filter benchmark CSV files."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import statistics


@dataclass(frozen=True)
class CaseResult:
    points: int
    checksum: str
    median_ns: float

    @property
    def throughput(self) -> float:
        return self.points * 1.0e9 / self.median_ns


def load_results(path: Path) -> dict[str, CaseResult]:
    rows = list(csv.reader(path.read_text(encoding='utf-8').splitlines()))
    if not rows or rows[0] != ['format', 'robot_self_filter_benchmark_v1']:
        raise ValueError(f'{path}: unsupported benchmark format')

    samples: dict[str, list[tuple[int, float, str]]] = {}
    for row in rows:
        if len(row) != 6 or row[0] in {'case', 'format', 'warmup', 'iterations'}:
            continue
        if row[0].startswith('summary_'):
            continue
        case_name = row[0]
        samples.setdefault(case_name, []).append((int(row[1]), float(row[3]), row[5]))

    if not samples:
        raise ValueError(f'{path}: no measured samples')

    results: dict[str, CaseResult] = {}
    for case_name, case_samples in samples.items():
        if len(case_samples) < 10:
            raise ValueError(
                f'{path}: {case_name} has only {len(case_samples)} samples'
            )
        points = {sample[0] for sample in case_samples}
        checksums = {sample[2] for sample in case_samples}
        if len(points) != 1 or len(checksums) != 1:
            raise ValueError(f'{path}: {case_name} metadata changed between samples')
        results[case_name] = CaseResult(
            points=points.pop(),
            checksum=checksums.pop(),
            median_ns=statistics.median(sample[1] for sample in case_samples),
        )
    return results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('baseline', type=Path)
    parser.add_argument('optimized', type=Path)
    parser.add_argument('--min-aggregate-improvement', type=float, default=20.0)
    parser.add_argument('--max-case-regression', type=float, default=5.0)
    args = parser.parse_args()

    baseline = load_results(args.baseline)
    optimized = load_results(args.optimized)
    if baseline.keys() != optimized.keys():
        raise ValueError('baseline and optimized benchmark cases differ')

    speedups: list[float] = []
    failed = False
    print('case,baseline_points_per_second,optimized_points_per_second,change_percent')
    for case_name in sorted(baseline):
        before = baseline[case_name]
        after = optimized[case_name]
        if before.points != after.points:
            raise ValueError(f'{case_name}: point counts differ')
        if before.checksum != after.checksum:
            raise ValueError(f'{case_name}: classification checksums differ')
        speedup = after.throughput / before.throughput
        speedups.append(speedup)
        change_percent = (speedup - 1.0) * 100.0
        print(
            f'{case_name},{before.throughput:.3f},{after.throughput:.3f},{change_percent:.3f}'
        )
        if change_percent < -args.max_case_regression:
            failed = True

    aggregate_speedup = math.prod(speedups) ** (1.0 / len(speedups))
    aggregate_improvement = (aggregate_speedup - 1.0) * 100.0
    print(f'aggregate_geomean_change_percent,{aggregate_improvement:.3f}')
    if aggregate_improvement < args.min_aggregate_improvement:
        failed = True
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
