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

"""Unit tests for the requirement fulfillment functions."""

import math

import pytest

from suave_requirements.fulfillment import exponential_decay
from suave_requirements.fulfillment import left_shoulder
from suave_requirements.fulfillment import MovingAverage
from suave_requirements.fulfillment import reaction_time_fulfillment
from suave_requirements.fulfillment import right_shoulder
from suave_requirements.fulfillment import search_footprint_fulfillment
from suave_requirements.fulfillment import thruster_availability_fulfillment


@pytest.mark.parametrize('x, expected', [
    (-1.0, 1.0), (0.0, 1.0), (0.25, 0.75), (1.0, 0.0), (3.0, 0.0)])
def test_left_shoulder(x, expected):
    """Left shoulder is flat at 1 below shoulder and 0 above foot."""
    assert left_shoulder(x, foot=1.0, shoulder=0.0) == pytest.approx(expected)


@pytest.mark.parametrize('x, expected', [
    (-1.0, 0.0), (0.0, 0.0), (3.0, 0.25), (12.0, 1.0), (20.0, 1.0)])
def test_right_shoulder(x, expected):
    """Right shoulder is flat at 0 below foot and 1 above shoulder."""
    assert right_shoulder(x, foot=0.0, shoulder=12.0) == pytest.approx(
        expected)


def test_exponential_decay():
    """Exponential decay is 1 at zero and decreases with x."""
    assert exponential_decay(0.0, exponent=0.2) == pytest.approx(1.0)
    assert exponential_decay(5.0, exponent=0.2) == pytest.approx(math.exp(-1))
    assert exponential_decay(-5.0, exponent=0.2) == 1.0


@pytest.mark.parametrize('operational, expected', [
    (6, 1.0), (7, 1.0), (5, 0.0), (0, 0.0), (5.5, 0.5)])
def test_thruster_availability_fulfillment(operational, expected):
    """Fulfillment drops to 0 once one required thruster is missing."""
    assert thruster_availability_fulfillment(
        6, operational, foot=1.0, shoulder=0.0) == \
        pytest.approx(expected)


@pytest.mark.parametrize('area, expected', [
    (0.0, 0.0), (1.33, 1.33 / 12.0), (6.0, 0.5), (12.0, 1.0), (15.0, 1.0)])
def test_search_footprint_fulfillment(area, expected):
    """Larger camera footprints increase fulfillment up to the shoulder."""
    assert search_footprint_fulfillment(
        area, foot=0.0, shoulder=12.0) == pytest.approx(expected)


@pytest.mark.parametrize('reaction_time, expected', [
    (0.0, 1.0), (5.0, math.exp(-1.0)), (10.0, math.exp(-2.0))])
def test_reaction_time_fulfillment(reaction_time, expected):
    """Earlier reactions are more fulfilling."""
    assert reaction_time_fulfillment(
        reaction_time, exponent=0.2) == pytest.approx(expected)


def test_moving_average_initial_and_window():
    """Moving average is undefined until a sample and keeps a window."""
    average = MovingAverage(2)
    assert average.value is None
    average.add(0.2)
    assert average.value == pytest.approx(0.2)
    average.add(0.4)
    average.add(0.8)
    assert average.value == pytest.approx(0.6)


def test_moving_average_rejects_empty_window():
    """A window must hold at least one sample."""
    with pytest.raises(ValueError):
        MovingAverage(0)
