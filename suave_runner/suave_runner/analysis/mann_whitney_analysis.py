# Copyright 2025 KAS-lab
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

"""Compare SUAVE experiment metrics using the Mann-Whitney U test."""

#
# Treats each managing system's samples as independent (unpaired) draws. For a
# paired, run-matched alternative, see `wilcoxon_analysis.py`.

import json
from pathlib import Path

import pandas as pd
import rclpy
from rclpy.node import Node
from scipy.stats import mannwhitneyu
from scipy.stats import shapiro


# (CSV column, output-file suffix, description, alternative hypothesis)
METRICS = [
    ('time searching pipeline (s)', 'time_search_pipeline',
     'time to find pipeline', 'less'),
    ('distance inspected (m)', 'distance_inspected',
     'distance inspected', 'greater'),
    ('mean reaction time (s)', 'mean_reaction_time',
     'mean reaction time', 'less'),
]
COMPARISON = {'less': 'lower', 'greater': 'greater'}


class SuaveData:
    """Load metric samples for one managing system."""

    def __init__(self, managing_system, csv_file):
        """Load the compared metric columns from CSV."""
        self.managing_system = managing_system
        self.samples = pd.read_csv(
            csv_file, usecols=[column for column, *_ in METRICS])

    def test_normality(self):
        """Run and print a Shapiro-Wilk normality test for each metric."""
        for column, _, description, _ in METRICS:
            _, p = shapiro(self.samples[column])
            print(
                f'\n{self.managing_system} {description}: '
                f'Shapiro-Wilk p-value = {p:.5f}')
            verdict = 'NOT normal' if p < 0.05 else 'normal'
            print(f'{self.managing_system} {description}: {verdict}')


class MannWhitneyAnalysis(Node):
    """Calculate pairwise Mann-Whitney U tests for experiment metrics."""

    def __init__(self, **kwargs):
        """Load configured datasets and initialize result matrices."""
        super().__init__('analysis', **kwargs)

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('filename', 'suave_statistical_analysis')
        self.declare_parameter('data_files', [''])

        self.result_path = Path(
            self.get_parameter('result_path').value).expanduser()
        self.filename = self.get_parameter('filename').value
        self.data_array = [
            SuaveData(data_file.get('managing_system'),
                      data_file.get('data_file'))
            for data_file in map(
                json.loads, self.get_parameter('data_files').value)
        ]
        systems = [data.managing_system for data in self.data_array]
        self.results = {
            suffix: pd.DataFrame(index=systems, columns=systems)
            for _, suffix, _, _ in METRICS
        }

    def save_data_frames(self):
        """Write pairwise p-value matrices to CSV files."""
        self.result_path.mkdir(parents=True, exist_ok=True)
        for suffix, matrix in self.results.items():
            matrix.to_csv(self.result_path / f'{self.filename}_{suffix}.csv')

    def u_test(self, i, j, data_i, data_j):
        """Compare two managing systems and store their metric p-values."""
        for column, suffix, description, alternative in METRICS:
            _, p = mannwhitneyu(
                data_i.samples[column], data_j.samples[column],
                alternative=alternative)
            self.results[suffix].iloc[i, j] = p
            negation = '' if p < 0.05 else 'NOT '
            print(
                f'Result: {data_i.managing_system} {description} is '
                f'{negation}{COMPARISON[alternative]} than '
                f'{data_j.managing_system}')

    def perform_analysis(self):
        """Run all pairwise comparisons and save their result matrices."""
        for data in self.data_array:
            data.test_normality()
        for i, data_i in enumerate(self.data_array):
            for j, data_j in enumerate(self.data_array):
                if i != j:
                    self.u_test(i, j, data_i, data_j)
        self.save_data_frames()


def main(args=None):
    """Run the statistical analysis node."""
    rclpy.init(args=args)
    node = MannWhitneyAnalysis()
    try:
        node.perform_analysis()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
