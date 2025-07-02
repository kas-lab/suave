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

import pandas as pd
from scipy.stats import shapiro
# from scipy.stats import levene
from scipy.stats import mannwhitneyu

import json
from pathlib import Path

import rclpy
from rclpy.node import Node

class SuaveData:
    def __init__(self, managing_system, csv_file):
        self.managing_system = managing_system

        self.time_search = pd.read_csv(csv_file)['time searching pipeline (s)']
        self.distance_inspect = pd.read_csv(csv_file)['distance inspected (m)']
        self.mean_reaction_time = pd.read_csv(csv_file)['mean reaction time (s)']

    def test_normality_(self, data_name, data):
        # Shapiro-Wilk test for normality
        _, p = shapiro(data)

        print(f"\n{self.managing_system} {data_name}: Shapiro-Wilk p-value = {p:.5f}")

        if p < 0.05:
            print(f"{self.managing_system} {data_name}: NOT normal")
        else:
            print(f"{self.managing_system} {data_name}: normal")

    def test_normality(self):
        self.test_normality_('time searching pipeline', self.time_search)
        self.test_normality_('distance inspected', self.distance_inspect)
        self.test_normality_('mean reaction time', self.mean_reaction_time)
        

class StatisticalAnalysis(Node):
    def __init__(self, **kwargs):
        super().__init__('analysis', **kwargs)
        
        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('filename', 'suave_statistical_analaysis')
        self.declare_parameter('data_files', [''])
        

        self.result_path = Path(self.get_parameter('result_path').get_parameter_value().string_value).expanduser()
        self.filename = self.get_parameter('filename').get_parameter_value().string_value

        data_files_param = self.get_parameter('data_files').get_parameter_value().string_array_value
        try:
            self.data_files = [json.loads(data_file_str) for data_file_str in data_files_param]
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in 'data_files' parameter: {e}")
            return

        self.data_array = [
            SuaveData(data_file.get('managing_system'), data_file.get('data_file'))
            for data_file in self.data_files
        ]
        systems = [data.managing_system for data in self.data_array]
        self.results_matrix_sp = pd.DataFrame(index=systems, columns=systems)
        self.results_matrix_di = pd.DataFrame(index=systems, columns=systems)
        self.results_matrix_mrt = pd.DataFrame(index=systems, columns=systems)

    def save_data_frames(self):
        if self.result_path.is_dir() is False:
            self.result_path.mkdir(parents=True)
        
        self.results_matrix_sp.to_csv(self.result_path / Path(self.filename + '_time_search_pipeline.csv'))
        self.results_matrix_di.to_csv(self.result_path / Path(self.filename + '_distance_inspected.csv'))
        self.results_matrix_mrt.to_csv(self.result_path / Path(self.filename + '_mean_reaction_time.csv'))

    def u_test(self, i, j, data_i, data_j):
        _, p_sp = mannwhitneyu(data_i.time_search, data_j.time_search, alternative="less")
        self.results_matrix_sp.iloc[i, j] = p_sp

        _, p_di = mannwhitneyu(data_i.distance_inspect, data_j.distance_inspect, alternative="greater")
        self.results_matrix_di.iloc[i, j] = p_di
        
        _, p_mrt = mannwhitneyu(data_i.mean_reaction_time, data_j.mean_reaction_time, alternative="less")
        self.results_matrix_mrt.iloc[i, j] = p_mrt

        if p_sp < 0.05:
            print(f"Result: {data_i.managing_system} time to find pipeline is lower than {data_j.managing_system}")
        else:
            print(f"Result: {data_i.managing_system} time to find pipeline is NOT lower than and {data_j.managing_system} time to find pipeline.")
        
        if p_di < 0.05:
            print(f"Result: {data_i.managing_system} distance inspected is greater than {data_j.managing_system}")
        else:
            print(f"Result: {data_i.managing_system} distance inspected is NOT greater than {data_j.managing_system} distance inspected.")
        
        if p_mrt < 0.05:
            print(f"Result: {data_i.managing_system} mean reaction time is lower than {data_j.managing_system}")
        else:
            print(f"Result: {data_i.managing_system} mean reaction time is NOT lower than {data_j.managing_system} mean reaction times.")

    def perform_analysis(self):
        for i, data_i in enumerate(self.data_array):
            for j, data_j in enumerate(self.data_array):
                if i == j:
                    self.results_matrix_sp.iloc[i, j] = None 
                    self.results_matrix_di.iloc[i, j] = None 
                    self.results_matrix_mrt.iloc[i, j] = None 
                    continue
                
                data_i.test_normality()
                data_j.test_normality()
                self.u_test(i, j, data_i, data_j)
        self.save_data_frames()

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = StatisticalAnalysis()
    executor.add_node(lc_node)
    
    try:
        analysis_task = executor.create_task(lc_node.perform_analysis)
        executor.spin_until_future_complete(analysis_task)
    except KeyboardInterrupt:
        print(' Ctrl+C received. Triggering shutdown...')
        lc_node.handle_termination()
        lc_node.shutdown_all_processes()
    finally:
        if rclpy.ok():
            executor.shutdown()
            lc_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()           
