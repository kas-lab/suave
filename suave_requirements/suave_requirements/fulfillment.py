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

"""Fuzzy memberships and fulfillment functions for SUAVE requirements."""

from collections import deque
import math


def _clamp(value):
    """Clamp a membership degree to [0, 1]."""
    return max(0.0, min(1.0, value))


def left_shoulder(x, *, foot, shoulder):
    """Return 1 at or below ``shoulder``, 0 at or above ``foot``."""
    if x <= shoulder:
        return 1.0
    if x >= foot:
        return 0.0
    return _clamp((foot - x) / (foot - shoulder))


def right_shoulder(x, *, foot, shoulder):
    """Return 0 at or below ``foot``, 1 at or above ``shoulder``."""
    if x <= foot:
        return 0.0
    if x >= shoulder:
        return 1.0
    return _clamp((x - foot) / (shoulder - foot))


def exponential_decay(x, *, exponent):
    """Return ``e^(-exponent * x)`` clamped to [0, 1]."""
    return _clamp(math.exp(-exponent * x))


def thruster_availability_fulfillment(
        required, operational, *, foot, shoulder):
    """Return AS CLOSE AS POSSIBLE TO 0 fulfillment of missing thrusters."""
    return left_shoulder(
        required - operational, foot=foot, shoulder=shoulder)


def search_footprint_fulfillment(coverage_area, *, foot, shoulder):
    """Return AS MANY AS POSSIBLE fulfillment of the camera footprint."""
    return right_shoulder(coverage_area, foot=foot, shoulder=shoulder)


def reaction_time_fulfillment(reaction_time, *, exponent):
    """Return AS EARLY AS POSSIBLE fulfillment of a reaction time (s)."""
    return exponential_decay(reaction_time, exponent=exponent)


class MovingAverage:
    """Mean over the last ``window_size`` samples, ``None`` if empty."""

    def __init__(self, window_size):
        """Create an empty window of ``window_size`` samples."""
        if window_size < 1:
            raise ValueError('window_size must be at least 1')
        self._samples = deque(maxlen=window_size)

    def add(self, sample):
        """Add a sample, dropping the oldest one when the window is full."""
        self._samples.append(sample)

    @property
    def value(self):
        """Return the current windowed mean, or ``None`` without samples."""
        if not self._samples:
            return None
        return sum(self._samples) / len(self._samples)
