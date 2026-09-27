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

"""The suave docker command group."""

import argparse

from suave_cli import images

DOCKER_EPILOG = """\
examples:
  suave docker build                   build suave-headless:latest
  suave docker run                     start or reuse the 'suave' container
  suave docker shell                   open a sourced shell in it
  suave docker stop --rm               stop and remove it
"""


def register(subparsers, common):
    """Add the docker command group."""
    parser = subparsers.add_parser(
        'docker', parents=[common], help='build images and manage the SUAVE container',
        description='Build SUAVE images and manage the SUAVE container. Host only.',
        epilog=DOCKER_EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='docker_command', metavar='ACTION', required=True)
    images.add_parsers(sub, common)
