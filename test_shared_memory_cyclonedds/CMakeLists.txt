cmake_minimum_required(VERSION 3.5)

project(test_shared_memory_cyclonedds)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(SKIP_SINGLE_RMW_TESTS
  "Skip tests involving only a single RMW implementation" OFF)
option(SKIP_MULTI_RMW_TESTS
  "Skip tests involving only multiple RMW implementations" OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING AND (CMAKE_SYSTEM_NAME MATCHES Linux OR CMAKE_SYSTEM_NAME MATCHES Darwin))
  find_package(ament_cmake REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)

  ament_index_get_resource(interface_files "rosidl_interfaces" "test_msgs")
  string(REPLACE "\n" ";" interface_files "${interface_files}")

  set(message_files "")
  set(service_files "")
  set(action_files "")

  # add datatypes to be used for testing
  list(APPEND message_files
    "msg/UInt32.msg"
    "msg/FixedArray.msg"
    "msg/DynamicArray.msg"
    "msg/FixedNested.msg"
    "msg/UnboundedString.msg"
    "msg/BoundedString.msg"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${message_files}
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


  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")

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

    ## pub/sub API tests
    configure_file(
      test/test_shm_publisher_subscriber.py.in
      test/test_shm_publisher_subscriber${suffix}.py.configured
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_publisher_subscriber${suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_publisher_subscriber${suffix}.py.configured"
    )

    list(LENGTH TEST_MESSAGE_TYPES length)
    math(EXPR timeout "${length} * 15")
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_publisher_subscriber${suffix}_$<CONFIG>.py"
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

    ## loan API tests
    configure_file(
      test/test_shm_loan_publisher_subscriber.py.in
      test/test_shm_loan_publisher_subscriber${suffix}.py.configured
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_loan_publisher_subscriber${suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_loan_publisher_subscriber${suffix}.py.configured"
    )

    list(LENGTH TEST_MESSAGE_TYPES length)
    math(EXPR timeout "${length} * 15")
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_loan_publisher_subscriber${suffix}_$<CONFIG>.py"
      TARGET test_shm_loan_publisher_subscriber${suffix}
      PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT ${timeout}
      ${SKIP_TEST})
    if(TEST test_shm_loan_publisher_subscriber${suffix})
      set_tests_properties(
        test_shm_loan_publisher_subscriber${suffix}
        PROPERTIES DEPENDS "test_shm_loan_publisher_cpp__${rmw_implementation1};test_shm_subscriber_cpp__${rmw_implementation2}"
      )
    endif()

    ## serialized API tests
    configure_file(
      test/test_shm_serialized_publisher_subscriber.py.in
      test/test_shm_serialized_publisher_subscriber${suffix}.py.configured
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_serialized_publisher_subscriber${suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_serialized_publisher_subscriber${suffix}.py.configured"
    )

    list(LENGTH TEST_MESSAGE_TYPES length)
    math(EXPR timeout "${length} * 15")
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_shm_serialized_publisher_subscriber${suffix}_$<CONFIG>.py"
      TARGET test_shm_serialized_publisher_subscriber${suffix}
      PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT ${timeout}
      ${SKIP_TEST})
    if(TEST test_shm_serialized_publisher_subscriber${suffix})
      set_tests_properties(
        test_shm_serialized_publisher_subscriber${suffix}
        PROPERTIES DEPENDS "test_shm_serialized_publisher_cpp__${rmw_implementation1};\
                            test_shm_serialized_subscriber_cpp__${rmw_implementation2}"
      )
    endif()


    ## network tests
    configure_file(
      test/test_network_publisher.py.in
      test/test_network_publisher${suffix}.py.configured
      @ONLY
    )

    configure_file(
      test/test_network_subscriber.py.in
      test/test_network_subscriber${suffix}.py.configured
      @ONLY
    )

    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_network_publisher${suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_network_publisher${suffix}.py.configured"
    )

    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_network_subscriber${suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_network_subscriber${suffix}.py.configured"
    )

    list(LENGTH TEST_MESSAGE_TYPES length)
    math(EXPR timeout "${length} * 15")

    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_network_publisher${suffix}_$<CONFIG>.py"
      TARGET test_network_publisher${suffix}
      PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT ${timeout}
      ${SKIP_TEST})
    if(TEST test_network_publisher${suffix})
      set_tests_properties(
        test_network_publisher${suffix}
        PROPERTIES DEPENDS "test_shm_publisher_cpp__${rmw_implementation1}"
      )
    endif()

    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_network_subscriber${suffix}_$<CONFIG>.py"
      TARGET test_network_subscriber${suffix}
      PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT ${timeout}
      ${SKIP_TEST})
    if(TEST test_network_subscriber${suffix})
      set_tests_properties(
        test_network_subscriber${suffix}
        PROPERTIES DEPENDS "test_shm_subscriber_cpp__${rmw_implementation2}"
      )
    endif()


  endmacro()

  macro(configure_template _client_library1 _client_library2)
    set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
    set(_client_library1 "${_client_library1}")
    set(_client_library2 "${_client_library2}")
    set(TEST_PUBLISHER_RCL "${_client_library1}")
    set(TEST_SUBSCRIBER_RCL "${_client_library2}")

    if(_client_library1 STREQUAL _client_library2)
      set(suffix "__${_client_library1}")
    else()
      set(suffix "__${_client_library1}__${_client_library2}")
    endif()

    if(_client_library1 STREQUAL "rclpy")
      #set(TEST_PUBLISHER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/publisher_py.py")
    elseif(_client_library1 STREQUAL "rclcpp")
      set(TEST_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_shm_publisher_cpp>")
      set(TEST_LOAN_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_shm_loan_publisher_cpp>")
      set(TEST_SER_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_shm_serialized_publisher_cpp>")
    endif()

    if(_client_library2 STREQUAL "rclpy")
      #set(TEST_SUBSCRIBER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/subscriber_py.py")
    elseif(_client_library2 STREQUAL "rclcpp")
      set(TEST_SUBSCRIBER_EXECUTABLE "$<TARGET_FILE:test_shm_subscriber_cpp>")
      set(TEST_SER_SUBSCRIBER_EXECUTABLE "$<TARGET_FILE:test_shm_serialized_subscriber_cpp>")
    endif()
  endmacro()


  macro(multi_client_libraries)
    # TODO(mikaelarguedas) Aggregate available rcl from ament index ?
    set(client_libraries "")
    #list(APPEND client_libraries "rclpy")
    list(APPEND client_libraries "rclcpp")
    foreach(client_library1 ${client_libraries})
      foreach(client_library2 ${client_libraries})
        configure_template("${client_library1}" "${client_library2}")
        multi_targets()
      endforeach()
    endforeach()
  endmacro()

  custom_executable(test_shm_publisher_cpp
    "test/test_shm_publisher.cpp")
  target_include_directories(test_shm_publisher_cpp PRIVATE
    include)
  custom_executable(test_shm_subscriber_cpp
    "test/test_shm_subscriber.cpp")
  target_include_directories(test_shm_subscriber_cpp PRIVATE
    include)
  custom_executable(test_shm_loan_publisher_cpp
    "test/test_shm_loan_publisher.cpp")
  target_include_directories(test_shm_loan_publisher_cpp PRIVATE
    include)
  custom_executable(test_shm_serialized_publisher_cpp
    "test/test_shm_serialized_publisher.cpp")
  target_include_directories(test_shm_serialized_publisher_cpp PRIVATE
    include)
  custom_executable(test_shm_serialized_subscriber_cpp
    "test/test_shm_serialized_subscriber.cpp")
  target_include_directories(test_shm_serialized_subscriber_cpp PRIVATE
    include)

  macro(targets)
    set(rmw_implementation1 "${rmw_implementation}")
    set(target_suffix1 "${target_suffix}")

    foreach(rmw_implementation2 ${rmw_implementations2})
      if(
        rmw_implementation1 STREQUAL rmw_implementation2 AND
        NOT SKIP_SINGLE_RMW_TESTS OR
        NOT rmw_implementation1 STREQUAL rmw_implementation2 AND
        NOT SKIP_MULTI_RMW_TESTS
      )
        multi_client_libraries()
      endif()
    endforeach()
  endmacro()


  # finding gtest once in the highest scope
  # prevents finding it repeatedly in each local scope
  ament_find_gtest()

  call_for_each_rmw_implementation(targets)
endif()  # BUILD_TESTING

ament_auto_package()
