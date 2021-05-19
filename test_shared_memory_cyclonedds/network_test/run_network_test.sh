#!/bin/bash

# Copyright (c) 2020 by Robert Bosch GmbH. All rights reserved.
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
GIT_DIR="$(git rev-parse --show-toplevel)"

echo $GIT_DIR

build_iceoryx_docker() {
	if [[ "$(docker images -q test_network:latest 2> /dev/null)" == "" ]]; then
		echo "Building base network testing docker image."
		$GIT_DIR/test_shared_memory_cyclonedds/network_test/docker/build_docker.sh
	fi
}

echo "Select an example:"
echo ""
options=("Start Network Test" "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Start Network Test")
			build_iceoryx_docker
			cd "$GIT_DIR/test_shared_memory_cyclonedds/network_test/docker"
			docker-compose -f test_network_communication.yml up
            break
            ;;
        "Quit")
            break
            ;;
        *) echo invalid option;;

    esac
done
