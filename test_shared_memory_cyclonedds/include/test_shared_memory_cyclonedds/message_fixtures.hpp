// NOLINT: This file starts with a BOM since it contain non-ASCII characters
//
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

#ifndef TEST_SHARED_MEMORY_CYCLONEDDS__MESSAGE_FIXTURES_HPP_
#define TEST_SHARED_MEMORY_CYCLONEDDS__MESSAGE_FIXTURES_HPP_

#include <cassert>
#include <limits>
#include <memory>
#include <vector>

#include "test_shared_memory_cyclonedds/msg/dynamic_array.hpp"
#include "test_shared_memory_cyclonedds/msg/fixed_array.hpp"
#include "test_shared_memory_cyclonedds/msg/fixed_nested.hpp"
#include "test_shared_memory_cyclonedds/msg/u_int32.hpp"
#include "test_shared_memory_cyclonedds/msg/bounded_string.hpp"
#include "test_shared_memory_cyclonedds/msg/unbounded_string.hpp"

std::vector<test_shared_memory_cyclonedds::msg::UInt32::SharedPtr>
create_messages_uint32() {
  std::vector<test_shared_memory_cyclonedds::msg::UInt32::SharedPtr> messages;
  {
    auto msg = std::make_shared<test_shared_memory_cyclonedds::msg::UInt32>();
    msg->data = 73;
    messages.push_back(msg);
  }
  return messages;
}

std::vector<test_shared_memory_cyclonedds::msg::FixedArray::SharedPtr>
create_messages_fixed_array() {
  using Msg = test_shared_memory_cyclonedds::msg::FixedArray;
  std::vector<Msg::SharedPtr> messages;
  {
    auto msg = std::make_shared<Msg>();
    for (uint32_t i = 0; i < Msg::NUMVALUES; ++i) {
      msg->data[i] = i + 42;
    }
    messages.push_back(msg);
  }
  return messages;
}

std::vector<test_shared_memory_cyclonedds::msg::DynamicArray::SharedPtr>
create_messages_dynamic_array() {
  using Msg = test_shared_memory_cyclonedds::msg::DynamicArray;
  std::vector<Msg::SharedPtr> messages;
  {
    auto msg = std::make_shared<Msg>();
    for (uint32_t i = 0; i < 10; ++i) {
      msg->data.push_back(i + 73);
    }
    messages.push_back(msg);
  }
  return messages;
}

std::vector<test_shared_memory_cyclonedds::msg::FixedNested::SharedPtr>
create_messages_fixed_nested() {
  using Msg = test_shared_memory_cyclonedds::msg::FixedNested;
  std::vector<Msg::SharedPtr> messages;
  {
    auto msg = std::make_shared<Msg>();
    msg->value.data = 37;
    for (uint32_t i = 0; i < 4; ++i) {
      msg->array.data[i] = i + 1;
    }
    messages.push_back(msg);
  }
  return messages;
}

std::vector<test_shared_memory_cyclonedds::msg::UnboundedString::SharedPtr>
create_messages_unbounded_string() {
  using Msg = test_shared_memory_cyclonedds::msg::UnboundedString;
  std::vector<Msg::SharedPtr> messages;
  {
    auto msg = std::make_shared<Msg>();
    msg->value.data = "Unbounded";
    messages.push_back(msg);
  }
  return messages;
}

std::vector<test_shared_memory_cyclonedds::msg::BoundedString::SharedPtr>
create_messages_bounded_string() {
  using Msg = test_shared_memory_cyclonedds::msg::BoundedString;
  std::vector<Msg::SharedPtr> messages;
  {
    auto msg = std::make_shared<Msg>();
    msg->value.data = "Bounded";
    messages.push_back(msg);
  }
  return messages;
}

#endif // TEST_SHARED_MEMORY_CYCLONEDDS__MESSAGE_FIXTURES_HPP_
