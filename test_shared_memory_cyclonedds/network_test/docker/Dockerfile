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


# ======================================================================================================================== #

FROM ros:rolling as builder
ARG REVISION
ARG B_ICEORYX_BUILD
ENV ICEORYX_BUILD=$B_ICEORYX_BUILD

ADD . /ros2_ws
WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
        libasio-dev \
        libcunit1-dev \
        bison \
        liblog4cxx-dev

RUN colcon build --merge-install --build-base build_network --install-base install_network \
    --packages-up-to test_shared_memory_cyclonedds --cmake-args -DCMAKE_BUILD_TYPE=Debug


# ======================================================================================================================== #

#FROM ubuntu:bionic as roudi

#COPY --from=builder /build_network/* /usr/lib/x86_64-linux-gnu/
# COPY --from=builder /iceoryx/build/dependencies/install/lib/lib* /usr/lib/x86_64-linux-gnu/
# COPY --from=builder /iceoryx/build/install/prefix/bin/* /usr/bin/
# COPY --from=builder /iceoryx/build/install/prefix/etc/* /etc/
# COPY --from=builder /iceoryx/tools/docker/.screenrc /root

#ENTRYPOINT ["/usr/bin/iox-roudi"]
