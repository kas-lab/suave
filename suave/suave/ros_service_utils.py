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

"""Shared ROS service-call helpers."""

import time
from typing import Callable
from typing import Optional


StopRequested = Callable[[], bool]


def call_service_with_timeout(
        node, cli, request, timeout_sec=5.0,
        stop_requested: Optional[StopRequested] = None):
    """Call a service and wait for its response or a stop request."""
    poll_interval = 0.05
    deadline = time.monotonic() + timeout_sec

    while True:
        if stop_requested is not None and stop_requested():
            return None
        remaining = deadline - time.monotonic()
        wait_timeout = min(poll_interval, max(0.0, remaining))
        if cli.wait_for_service(timeout_sec=wait_timeout):
            break
        if remaining <= 0.0:
            node.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None

    future = cli.call_async(request)
    deadline = time.monotonic() + timeout_sec

    while not future.done():
        if stop_requested is not None and stop_requested():
            future.cancel()
            return None
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            future.cancel()
            node.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        time.sleep(min(poll_interval, remaining))

    return future.result()
