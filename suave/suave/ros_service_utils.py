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


def call_service_with_timeout(node, cli, request, timeout_sec=5.0):
    """Call a ROS service and wait for a response, returning None on failure."""
    if cli.wait_for_service(timeout_sec=timeout_sec) is False:
        node.get_logger().error(
            'service not available {}'.format(cli.srv_name))
        return None
    future = cli.call_async(request)
    node.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
    if future.done() is False:
        node.get_logger().error(
            'Future not completed {}'.format(cli.srv_name))
        return None
    return future.result()
