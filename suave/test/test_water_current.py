# Copyright 2026 KAS Lab
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

"""Tests for the water current heading model."""

import math
import random
import statistics

import pytest

import rclpy
from rclpy.parameter import Parameter

from suave.water_current import gauss_markov_step
from suave.water_current import WaterCurrent


@pytest.fixture(scope='module', autouse=True)
def rclpy_runtime():
    """Initialize rclpy for the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**overrides):
    """Create a WaterCurrent node with parameter overrides."""
    node = WaterCurrent()
    node.set_parameters(
        [Parameter(name, value=value) for name, value in overrides.items()])
    return node


def heading_trace(node, times):
    """Return the computed heading at each elapsed time."""
    return [node.compute_heading(t) for t in times]


def test_default_heading_is_small_sinusoid_around_mean():
    """Default heading swings 10 degrees around the mean over 120 s."""
    node = make_node(heading=90.0)
    try:
        assert node.compute_heading(0.0) == pytest.approx(math.radians(90.0))
        assert node.compute_heading(30.0) == pytest.approx(
            math.radians(100.0))
        assert node.compute_heading(90.0) == pytest.approx(math.radians(80.0))
    finally:
        node.destroy_node()


def test_heading_noise_is_reproducible_for_same_seed():
    """Same seed gives the same noisy heading trace."""
    times = [float(t) for t in range(1, 50)]
    params = {
        'heading_amplitude': 0.0,
        'heading_noise_std': 5.0,
        'heading_noise_seed': 7,
    }
    first = make_node(**params)
    second = make_node(**params)
    try:
        trace = heading_trace(first, times)
        assert trace == heading_trace(second, times)
        assert any(abs(value) > 0.0 for value in trace)
    finally:
        first.destroy_node()
        second.destroy_node()


def test_heading_noise_disabled_by_default():
    """Without noise the heading follows the sinusoid only."""
    node = make_node(heading_amplitude=0.0, heading=45.0)
    try:
        for value in heading_trace(node, [1.0, 2.0, 3.0]):
            assert value == pytest.approx(math.radians(45.0))
    finally:
        node.destroy_node()


def test_gauss_markov_step_has_unit_stationary_variance():
    """Exact discretization keeps unit variance for any dt."""
    rng = random.Random(1)
    for dt in (1.0, 5.0):
        state = 0.0
        samples = []
        for _ in range(20000):
            state = gauss_markov_step(state, dt, 10.0, rng)
            samples.append(state)
        assert statistics.pstdev(samples[1000:]) == pytest.approx(
            1.0, abs=0.1)
