cmake_minimum_required(VERSION 3.5)

project(test_shared_memory_cyclonedds)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND MSVC)
  # /bigobj is needed to avoid error C1128:
  #   https://msdn.microsoft.com/en-us/library/8578y171.aspx
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)

  ament_index_get_resource(interface_files "rosidl_interfaces" "test_msgs")
  string(REPLACE "\n" ";" interface_files "${interface_files}")

  set(message_files "")
  set(service_files "")
  set(action_files "")
  foreach(interface_file ${interface_files})
    get_filename_component(interface_ns "${interface_file}" DIRECTORY)
    get_filename_component(interface_ns "${interface_ns}" NAME)
    string_ends_with("${interface_file}" ".msg" is_message)
    if(is_message AND interface_ns STREQUAL "msg")
      list(APPEND message_files "${interface_file}")
      continue()
    endif()
    string_ends_with("${interface_file}" ".srv" is_service)
    if(is_service AND interface_ns STREQUAL "srv")
      list(APPEND service_files "${interface_file}")
      continue()
    endif()
    string_ends_with("${interface_file}" ".idl" is_action)
    if(is_action AND interface_ns STREQUAL "action")
      list(APPEND action_files "${interface_file}")
      continue()
    endif()
  endforeach()

  set(other_message_files
    "msg/UInt32.msg"
  )
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${other_message_files}
    DEPENDENCIES test_msgs
    SKIP_INSTALL
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  # Provides PYTHON_EXECUTABLE_DEBUG
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations2)
  foreach(rmw_implementation ${rmw_implementations2})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  function(custom_test target with_message_argument)
    if(with_message_argument)
      # adding test for each message type
      foreach(message_file ${message_files})
        get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
        ament_add_test(
          "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
          COMMAND "$<TARGET_FILE:${target}>" "${TEST_MESSAGE_TYPE}"
          TIMEOUT 15
          GENERATE_RESULT_FOR_RETURN_CODE_ZERO
          APPEND_LIBRARY_DIRS "${append_library_dirs}"
          ENV
          RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
          RMW_IMPLEMENTATION=${rmw_implementation})
        set_tests_properties(
          "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
          PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}>"
        )
      endforeach()
    else()
      ament_add_test(
        "${target}${target_suffix}"
        COMMAND "$<TARGET_FILE:${target}>"
        TIMEOUT 15
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        APPEND_LIBRARY_DIRS "${append_library_dirs}"
        ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation})
      set_tests_properties(
        "${target}${target_suffix}"
        PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}>"
      )
    endif()
  endfunction()

  # shm needed
  function(custom_executable target)
    add_executable(${target} ${ARGN})
    rosidl_target_interfaces(${target}
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
    ament_target_dependencies(${target}
      "rclcpp"
      "rclcpp_action"
    )
  endfunction()

  macro(multi_targets)
    # test publish / subscribe messages
    if(rmw_implementation1 STREQUAL rmw_implementation2)
      set(suffix "${suffix}__${rmw_implementation1}")
    else()
      set(suffix "${suffix}__${rmw_implementation1}__${rmw_implementation2}")
    endif()

    set(SKIP_TEST "")

    # TODO(wjwwood): Connext and Fast-RTPS do not currently communicate over pub/sub
    set(rmw_implementation1_is_fastrtps FALSE)
    set(rmw_implementation2_is_fastrtps FALSE)
    if(rmw_implementation1 MATCHES "(.*)fastrtps(.*)")
      set(rmw_implementation1_is_fastrtps TRUE)
    endif()
    if(rmw_implementation2 MATCHES "(.*)fastrtps(.*)")
      set(rmw_implementation2_is_fastrtps TRUE)
    endif()
    if(
      WIN32 AND
      NOT "${rmw_implementation1_is_fastrtps}" STREQUAL "${rmw_implementation2_is_fastrtps}"
    )
      set(SKIP_TEST "SKIP_TEST")
    endif()

    set(rmw_implementation1_is_connext FALSE)
    if(rmw_implementation1 MATCHES "(.*)connext(.*)")
      set(rmw_implementation1_is_connext TRUE)
    endif()
    set(rmw_implementation2_is_connext FALSE)
    if(rmw_implementation2 MATCHES "(.*)connext(.*)")
      set(rmw_implementation2_is_connext TRUE)
    endif()

    # TODO(asorbini) Skip tests between rmw_connext_cpp and rmw_connextdds
    # since they are not meant to coexist (and used incompatible typecodes by default).
    set(rmw_implementation1_is_connextdds FALSE)
    if(rmw_implementation1 MATCHES "(.*)connextdds")
      set(rmw_implementation1_is_connextdds TRUE)
    endif()
    set(rmw_implementation2_is_connextdds FALSE)
    if(rmw_implementation2 MATCHES "(.*)connextdds")
      set(rmw_implementation2_is_connextdds TRUE)
    endif()

    set(rmw_implementation1_is_connext_cpp FALSE)
    if(rmw_implementation1 MATCHES "(.*)connext(.*)_cpp")
      set(rmw_implementation1_is_connext_cpp TRUE)
    endif()
    set(rmw_implementation2_is_connext_cpp FALSE)
    if(rmw_implementation2 MATCHES "(.*)connext(.*)_cpp")
      set(rmw_implementation2_is_connext_cpp TRUE)
    endif()

    if((rmw_implementation1_is_connextdds AND rmw_implementation2_is_connext_cpp) OR
      (rmw_implementation1_is_connext_cpp AND rmw_implementation2_is_connextdds)
    )
      set(SKIP_TEST "SKIP_TEST")
    endif()

    set(rmw_implementation1_is_cyclonedds FALSE)
    set(rmw_implementation2_is_cyclonedds FALSE)
    if(rmw_implementation1 MATCHES "(.*)cyclonedds(.*)")
      set(rmw_implementation1_is_cyclonedds TRUE)
    endif()
    if(rmw_implementation2 MATCHES "(.*)cyclonedds(.*)")
      set(rmw_implementation2_is_cyclonedds TRUE)
    endif()

    set(PUBLISHER_RMW ${rmw_implementation1})
    set(SUBSCRIBER_RMW ${rmw_implementation2})
    set(TEST_MESSAGE_TYPES "")
    foreach(message_file ${message_files})
      get_filename_component(message_type "${message_file}" NAME_WE)
      # TODO(dirk-thomas) WStrings published by FastRTPS can't be received
      # correctly by Connext on macOS
      if(
        "${message_type}" STREQUAL "WStrings" AND
        rmw_implementation1_is_fastrtps AND
        rmw_implementation2_is_connext AND
        APPLE
      )
        continue()
      endif()
      # TODO(dirk-thomas) Connext and CycloneDDS don't interoperate for WString
      if(
        "${message_type}" STREQUAL "WStrings" AND
        (
          (rmw_implementation1_is_connext AND rmw_implementation2_is_cyclonedds) OR
          (rmw_implementation1_is_cyclonedds AND rmw_implementation2_is_connext)
        )
      )
        continue()
      endif()
      list(APPEND TEST_MESSAGE_TYPES "${message_type}")
    endforeach()

    # shm needed
    message("configuring file")
    configure_file(
      test_shm_publisher_subscriber.py.in
      test_shm_publisher_subscriber${suffix}.py.configured
      @ONLY
    )
    message("generating file")
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_shm_publisher_subscriber${suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_shm_publisher_subscriber${suffix}.py.configured"
    )

    # shm needed
    list(LENGTH TEST_MESSAGE_TYPES length)
    math(EXPR timeout "${length} * 15")
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test_shm_publisher_subscriber${suffix}_$<CONFIG>.py"
      TARGET test_shm_publisher_subscriber${suffix}
      PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT ${timeout}
      ${SKIP_TEST})
    if(TEST test_shm_publisher_subscriber${suffix})
      set_tests_properties(
        test_shm_publisher_subscriber${suffix}
        PROPERTIES DEPENDS "test_shm_publisher_cpp__${rmw_implementation1};test_shm_subscriber_cpp__${rmw_implementation2}"
      )
    endif()

  endmacro()

  # executables publisher / subscriber
  custom_executable(test_shm_publisher_cpp
    "test_shm_publisher.cpp")
  custom_executable(test_shm_subscriber_cpp
    "test_shm_subscriber.cpp")

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")


  # finding gtest once in the highest scope
  # prevents finding it repeatedly in each local scope
  ament_find_gtest()

  #call_for_each_rmw_implementation(targets)

endif()  # BUILD_TESTING

ament_auto_package()
