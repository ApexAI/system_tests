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
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_shared_memory_cyclonedds/message_fixtures.hpp"

template<typename T>
rclcpp::SubscriptionBase::SharedPtr subscribe(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::string & qos_type,
  std::vector<typename T::SharedPtr> & expected_messages,
  std::vector<bool> & received_messages_indicator)
{
  received_messages_indicator = std::vector<bool>(expected_messages.size(), false);
  auto callback =
    [expected_messages, &received_messages_indicator]
      (const typename T::SharedPtr received_message) -> void
    {
      // find received message in vector of expected messages
      bool known_message = false;
      size_t index = 0;
      for (auto expected_message : expected_messages) {
        if (*received_message == *expected_message) {
          printf("received message #%zu of %zu\n", index + 1, expected_messages.size());
          known_message = true;

          // we may read the same message in another subscribe call depending on QoS
          // (it may not be removed from cache)
          received_messages_indicator[index] = true;
          break;
        }
        ++index;
      }
      if (!known_message) {
        throw std::runtime_error("received message does not match any expected message");
      }

      // shutdown node when all expected messages have been received
      for (auto indicator : received_messages_indicator) {
        if (!indicator) {
          // message not received yet, continue
          return;
        }
      }

      // all expected messages received
      rclcpp::shutdown();
    };

  // default QOS policy is KeepLast
  auto qos = rclcpp::QoS(rclcpp::KeepLast(expected_messages.size()));

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

  auto subscriber =
    node->create_subscription<T>(std::string("test/message/") + message_type, qos, callback);
  return subscriber;
}

int main(int argc, char ** argv)
{
  if (argc != 4) {
    fprintf(stderr, "Wrong number of arguments, pass one message type\n");
    return EXIT_FAILURE;
  }
  rclcpp::init(argc, argv);

  auto start = std::chrono::steady_clock::now();

  std::string message_type = argv[1];
  std::string qos_type = argv[2];
  std::string test_namespace = argv[3];
  auto node = rclcpp::Node::make_shared(
    std::string("test_shm_subscriber_") + message_type, test_namespace);

  // must exist while the callback executes
  std::vector<bool> received_messages_indicator;

  rclcpp::SubscriptionBase::SharedPtr subscriber;
  if (message_type == "UInt32") {
    auto expected_messages = create_messages_uint32();
    subscriber = subscribe<test_shared_memory_cyclonedds::msg::UInt32>(
      node, message_type, qos_type, expected_messages, received_messages_indicator);
  } else if (message_type == "FixedArray") {
    auto expected_messages = create_messages_fixed_array();
    subscriber = subscribe<test_shared_memory_cyclonedds::msg::FixedArray>(
      node, message_type, qos_type, expected_messages, received_messages_indicator);
  } else if (message_type == "DynamicArray") {
    auto expected_messages = create_messages_dynamic_array();
    subscriber = subscribe<test_shared_memory_cyclonedds::msg::DynamicArray>(
      node, message_type, qos_type, expected_messages, received_messages_indicator);
  } else {
    fprintf(stderr, "Unknown message argument '%s'\n", message_type.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("subscribed for %f seconds\n", diff.count());

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
