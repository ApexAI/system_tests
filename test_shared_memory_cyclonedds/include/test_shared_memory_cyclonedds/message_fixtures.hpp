// Copyright 2021, Apex.AI Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_MSGS__SHM_MESSAGE_FIXTURES_HPP_
#define TEST_MSGS__SHM_MESSAGE_FIXTURES_HPP_

#include <cassert>
#include <limits>
#include <memory>
#include <vector>

#include "test_shared_memory_cyclonedds/msg/u_int32.hpp"

std::vector<test_shared_memory_cyclonedds::msg::UInt32::SharedPtr>
create_messages_uint32()
{
  std::vector<test_shared_memory_cyclonedds::msg::UInt32::SharedPtr> messages;
  {
    auto msg = std::make_shared<test_shared_memory_cyclonedds::msg::UInt32>();
    msg->data = 73;
    messages.push_back(msg);
  }
  return messages;
}

#endif  // TEST_MSGS__SHM_MESSAGE_FIXTURES_HPP_
