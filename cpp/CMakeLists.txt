cmake_minimum_required(VERSION 3.16)
project(g1motor VERSION 0.4.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Library: headers and sources live together in g1motor/.
add_library(g1motor
  g1motor/protocol.cc
  g1motor/serial_port.cc
  g1motor/motor_bus.cc
  g1motor/joint.cc)
target_include_directories(g1motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)
if(MSVC)
  target_compile_definitions(g1motor PUBLIC NOMINMAX)
  target_compile_options(g1motor PRIVATE /W4)
else()
  target_compile_options(g1motor PRIVATE -Wall -Wextra)
endif()

option(G1MOTOR_BUILD_EXAMPLES "Build the example programs" ON)
if(G1MOTOR_BUILD_EXAMPLES)
  foreach(example scan zero_torque position_hold sine)
    add_executable(${example} examples/${example}.cc)
    target_link_libraries(${example} PRIVATE g1motor)
  endforeach()
endif()

include(CTest)
if(BUILD_TESTING)
  # protocol_test and joint_test need no hardware and run under ctest.
  foreach(test protocol_test joint_test)
    add_executable(${test} tests/${test}.cc)
    target_link_libraries(${test} PRIVATE g1motor)
    add_test(NAME ${test} COMMAND ${test})
  endforeach()
  # hardware_test needs motors on a bus without a controller; run it by hand.
  add_executable(hardware_test tests/hardware_test.cc)
  target_link_libraries(hardware_test PRIVATE g1motor)
endif()

install(TARGETS g1motor ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY g1motor DESTINATION include FILES_MATCHING PATTERN "*.h")
