# Copyright 2026 KAS-lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Compare SUAVE experiment metrics using the paired Wilcoxon signed-rank test."""

# Every input CSV must already carry a `run_idx` column identifying which run
# of that managing system each row is, produced by `sort_results.py` (see the
# sort-suave-results skill). Pairing by row order or position is not
# supported: a CSV without `run_idx`, with duplicate `run_idx` values, or with
# no matching counterpart in the other system raises rather than guessing.

import json
from pathlib import Path

import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from scipy.stats import rankdata
from scipy.stats import wilcoxon

# (CSV column, result key, alternative, exclude pairs with pipeline-not-found)
METRICS = [
    ('time searching pipeline (s)', 'search_time', 'less', True),
    ('distance inspected (m)', 'distance_inspected', 'greater', True),
]


def load_system(managing_system, csv_path):
    """Load one managing system's sorted, run_idx-tagged result CSV."""
    df = pd.read_csv(csv_path)
    if 'run_idx' not in df.columns:
        raise ValueError(
            f"{csv_path} ({managing_system}) has no 'run_idx' column. Sort "
            'it first with sort_results.py (see the sort-suave-results '
            'skill) -- pairing by row position is not supported.')
    if df['run_idx'].duplicated().any():
        duplicates = sorted(df.loc[df['run_idx'].duplicated(), 'run_idx'])
        raise ValueError(
            f'{csv_path} ({managing_system}): duplicate run_idx values '
            f'{duplicates}; refusing to pair on an ambiguous key.')
    if 'pipeline found' in df.columns:
        df = df.copy()
        df['pipeline found'] = (
            df['pipeline found'].astype(str).str.strip().str.lower()
            == 'true')
    return df


def holm_correction(p_values):
    """Return Holm-Bonferroni adjusted p-values, in the input's own order."""
    count = len(p_values)
    order = sorted(range(count), key=lambda i: p_values[i])
    adjusted = [None] * count
    running_max = 0.0
    for rank, index in enumerate(order):
        value = min(1.0, p_values[index] * (count - rank))
        running_max = max(running_max, value)
        adjusted[index] = running_max
    return adjusted


def rank_biserial(diffs):
    """Matched-pairs rank-biserial effect size from paired differences."""
    nonzero = diffs[diffs != 0]
    if len(nonzero) == 0:
        return float('nan')
    ranks = rankdata(np.abs(nonzero))
    positive_rank_sum = ranks[nonzero > 0].sum()
    negative_rank_sum = ranks[nonzero < 0].sum()
    return (
        (positive_rank_sum - negative_rank_sum) /
        (positive_rank_sum + negative_rank_sum))


def paired_test(row_system, col_system, row_df, col_df, column, key,
                alternative, censor_on_not_found):
    """Run one paired Wilcoxon comparison for one metric between two systems."""
    merged = row_df.merge(
        col_df, on='run_idx', suffixes=('_row', '_col'), how='inner')
    declared_pairs = len(merged)
    missing_row = sorted(set(row_df['run_idx']) - set(merged['run_idx']))
    missing_col = sorted(set(col_df['run_idx']) - set(merged['run_idx']))

    value_row = f'{column}_row'
    value_col = f'{column}_col'
    used = merged
    excluded_extra = set()

    if censor_on_not_found and 'pipeline found_row' in used.columns:
        found_both = used['pipeline found_row'] & used['pipeline found_col']
        excluded_extra |= set(used.loc[~found_both, 'run_idx'])
        used = used.loc[found_both]

    diffs = (used[value_row] - used[value_col]).to_numpy()
    n_pos = int((diffs > 0).sum())
    n_neg = int((diffs < 0).sum())
    n_zero = int((diffs == 0).sum())

    result = {
        'row': row_system, 'col': col_system, 'metric': key,
        'alternative': alternative, 'declared_pairs': declared_pairs,
        'n_used': len(used), 'n_pos': n_pos, 'n_neg': n_neg,
        'n_zero': n_zero,
        'median_diff': (
            float(np.median(diffs)) if len(diffs) else float('nan')),
        'missing_row': ';'.join(str(v) for v in missing_row),
        'missing_col': ';'.join(str(v) for v in missing_col),
        'excluded_extra': ';'.join(str(v) for v in sorted(excluded_extra)),
        'statistic': float('nan'), 'p_raw': float('nan'),
        'effect_size': float('nan'), 'note': ''}

    if n_pos + n_neg < 2:
        result['note'] = (
            f'fewer than 2 nonzero paired differences ({n_pos + n_neg}); '
            'test not computed')
        return result

    statistic, p_raw = wilcoxon(
        used[value_row], used[value_col], alternative=alternative,
        zero_method='wilcox', mode='auto')
    result['statistic'] = float(statistic)
    result['p_raw'] = float(p_raw)
    result['effect_size'] = rank_biserial(diffs)
    return result


class WilcoxonAnalysis(Node):
    """Calculate pairwise, run-matched Wilcoxon signed-rank tests."""

    def __init__(self, **kwargs):
        """Load configured, run_idx-tagged datasets."""
        super().__init__('wilcoxon_analysis', **kwargs)

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('filename', 'suave_wilcoxon_analysis')
        self.declare_parameter('data_files', [''])
        self.declare_parameter('correction', 'none')

        self.result_path = Path(self.get_parameter(
            'result_path').get_parameter_value().string_value).expanduser()
        self.filename = self.get_parameter(
            'filename').get_parameter_value().string_value
        self.correction = self.get_parameter(
            'correction').get_parameter_value().string_value

        data_files_param = self.get_parameter(
            'data_files').get_parameter_value().string_array_value
        data_files = [json.loads(entry) for entry in data_files_param]

        self.systems = [
            (entry['managing_system'],
             load_system(entry['managing_system'], entry['data_file']))
            for entry in data_files
        ]

    def perform_analysis(self):
        """Run all pairwise, per-metric comparisons and save the results."""
        results = []
        for row_system, row_df in self.systems:
            for col_system, col_df in self.systems:
                if row_system == col_system:
                    continue
                for column, key, alternative, censor in METRICS:
                    result = paired_test(
                        row_system, col_system, row_df, col_df, column, key,
                        alternative, censor)
                    results.append(result)
                    self._log_result(result)

        self._apply_correction(results)
        self._save_results(results)

    def _apply_correction(self, results):
        for result in results:
            result['correction'] = self.correction

        if self.correction != 'holm':
            for result in results:
                result['p_adjusted'] = result['p_raw']
            return

        valid_indices = [
            i for i, r in enumerate(results) if not np.isnan(r['p_raw'])]
        adjusted = holm_correction([results[i]['p_raw'] for i in valid_indices])
        for position, index in enumerate(valid_indices):
            results[index]['p_adjusted'] = adjusted[position]
        for result in results:
            result.setdefault('p_adjusted', float('nan'))

    def _log_result(self, result):
        if np.isnan(result['p_raw']):
            self.get_logger().info(
                f"{result['row']} vs {result['col']} {result['metric']}: "
                f"{result['note']}")
            return
        verdict = 'SIGNIFICANT' if result['p_raw'] < 0.05 else 'not significant'
        self.get_logger().info(
            f"{result['row']} vs {result['col']} {result['metric']} "
            f"({result['alternative']}): p={result['p_raw']:.5f} "
            f'({verdict}), n_used={result["n_used"]}')

    def _save_results(self, results):
        """Write the tidy per-comparison CSV and compatibility matrices."""
        if not self.result_path.is_dir():
            self.result_path.mkdir(parents=True)

        tidy = pd.DataFrame(results)
        column_order = [
            'row', 'col', 'metric', 'alternative', 'declared_pairs',
            'n_used', 'n_pos', 'n_neg', 'n_zero', 'median_diff', 'statistic',
            'p_raw', 'p_adjusted', 'correction', 'effect_size',
            'missing_row', 'missing_col', 'excluded_extra', 'note',
        ]
        tidy = tidy[[c for c in column_order if c in tidy.columns]]
        tidy.to_csv(
            self.result_path / f'{self.filename}_results.csv', index=False)

        systems = [name for name, _ in self.systems]
        matrix_metrics = {
            'search_time': 'time_search_pipeline',
            'distance_inspected': 'distance_inspected',
        }
        for metric_key, file_suffix in matrix_metrics.items():
            matrix = pd.DataFrame(index=systems, columns=systems)
            for result in results:
                if result['metric'] == metric_key:
                    matrix.loc[result['row'], result['col']] = result['p_raw']
            matrix.to_csv(
                self.result_path / f'{self.filename}_{file_suffix}.csv')


def main(args=None):
    """Run the Wilcoxon analysis node."""
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = WilcoxonAnalysis()
    executor.add_node(lc_node)

    try:
        analysis_task = executor.create_task(lc_node.perform_analysis)
        executor.spin_until_future_complete(analysis_task)
    except KeyboardInterrupt:
        print(' Ctrl+C received. Triggering shutdown...')
    finally:
        if rclpy.ok():
            executor.shutdown()
            lc_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
