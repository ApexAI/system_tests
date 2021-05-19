// Copyright 2021, Apex.AI Inc. All rights reserved.
// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_shared_memory_cyclonedds/message_fixtures.hpp"

template<typename T>
void publish(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::string & qos_type,
  std::vector<typename T::SharedPtr> messages,
  size_t number_of_cycles = 100)
{
  auto start = std::chrono::steady_clock::now();

  // default QOS policy is KeepLast
  auto qos = rclcpp::QoS(rclcpp::KeepLast(messages.size()));

  if (qos_type == "qos_keepall_besteffort_transientlocal") {
    qos = rclcpp::QoS(rclcpp::KeepAll());
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  } else if (qos_type == "qos_keeplast_reliable_transientlocal") {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  } else if (qos_type == "qos_keeplast_besteffort_volatile") {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
  }

  auto publisher = node->create_publisher<T>(std::string("test/message/") + message_type, qos);

  rclcpp::WallRate cycle_rate(10);
  rclcpp::WallRate message_rate(100);
  size_t cycle_index = 0;
  // publish all messages up to number_of_cycles times, longer sleep between each cycle
  while (rclcpp::ok() && cycle_index < number_of_cycles) {
    size_t message_index = 0;
    // publish all messages one by one, shorter sleep between each message
    while (rclcpp::ok() && message_index < messages.size()) {
      auto loaned_msg = publisher->borrow_loaned_message();
      auto msg_data = messages[message_index];
      loaned_msg.get() = *msg_data;

      printf("publishing message #%zu\n", message_index + 1);
      publisher->publish(std::move(loaned_msg));
      ++message_index;
      message_rate.sleep();
    }
    ++cycle_index;
    cycle_rate.sleep();
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("published for %f seconds\n", diff.count());
}

int main(int argc, char ** argv)
{
  if (argc != 4) {
    fprintf(stderr, "Wrong number of arguments, pass one message type\n");
    return EXIT_FAILURE;
  }
  rclcpp::init(argc, argv);

  std::string message_type = argv[1];
  std::string qos_type = argv[2];
  std::string test_namespace = argv[3];
  auto node = rclcpp::Node::make_shared(
    std::string("test_shm_publisher_") + message_type, test_namespace);

  if (message_type == "UInt32") {
    publish<test_shared_memory_cyclonedds::msg::UInt32>(
      node, message_type, qos_type,
      create_messages_uint32());
  } else if (message_type == "FixedArray") {
    publish<test_shared_memory_cyclonedds::msg::FixedArray>(
      node, message_type, qos_type,
      create_messages_fixed_array());
  } else if (message_type == "DynamicArray") {
    publish<test_shared_memory_cyclonedds::msg::DynamicArray>(
      node, message_type, qos_type,
      create_messages_dynamic_array());
  } else if (message_type == "FixedNested") {
    publish<test_shared_memory_cyclonedds::msg::FixedNested>(
      node, message_type, qos_type,
      create_messages_fixed_nested());
    else if (message_type == "UnboundedString") {
      publish<test_shared_memory_cyclonedds::msg::UnboundedString>(
          node, message_type, qos_type, create_messages_unbounded_string());
    }
    else if (message_type == "BoundedString") {
      publish<test_shared_memory_cyclonedds::msg::BoundedString>(
          node, message_type, qos_type, create_messages_bounded_string());
    }
    else {
      fprintf(stderr, "Unknown message argument '%s'\n", message_type.c_str());
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
