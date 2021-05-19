#!/usr/bin/env bash

# Copyright (c) 2021 by Apex.AI Inc. All rights reserved.
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
#
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

REVISION="latest"
IMAGE_NAME="test_network:${REVISION}"
ICEORYX_BUILD=build_network_test
DOCKERFILE_DIR="$(dirname $(realpath -s $0))"
ROS2_WORKSPACE="$(dirname $(realpath -s $0))/../../../"

echo "ROS2_WORKSPACE: ${ROS2_WORKSPACE}"
echo "DOCKERFILE_DIR: ${DOCKERFILE_DIR}"
cd ../../../../../
echo "pwd: $(pwd)"



docker build -f "${DOCKERFILE_DIR}/Dockerfile" --build-arg B_ICEORYX_BUILD="${ICEORYX_BUILD}" --build-arg REVISION="${REVISION}" . -t "${IMAGE_NAME}"
