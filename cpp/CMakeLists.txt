cmake_minimum_required(VERSION 3.16)
project(g1motor VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(g1motor src/protocol.cc src/serial_port.cc src/motor_bus.cc)
target_include_directories(g1motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(MSVC)
  target_compile_definitions(g1motor PUBLIC _USE_MATH_DEFINES NOMINMAX)
  target_compile_options(g1motor PRIVATE /W4)
else()
  target_compile_options(g1motor PRIVATE -Wall -Wextra)
endif()

option(G1MOTOR_BUILD_EXAMPLES "Build the example programs" ON)
if(G1MOTOR_BUILD_EXAMPLES)
  foreach(example scan position_hold)
    add_executable(g1motor_${example} examples/${example}.cc)
    target_link_libraries(g1motor_${example} PRIVATE g1motor)
  endforeach()
endif()

include(CTest)
if(BUILD_TESTING)
  # protocol_test needs no hardware and runs under ctest.
  add_executable(g1motor_protocol_test tests/protocol_test.cc)
  target_link_libraries(g1motor_protocol_test PRIVATE g1motor)
  add_test(NAME protocol_test COMMAND g1motor_protocol_test)
  # hardware_test needs motors on a bus without a controller; run it by hand.
  add_executable(g1motor_hardware_test tests/hardware_test.cc)
  target_link_libraries(g1motor_hardware_test PRIVATE g1motor)
endif()

install(TARGETS g1motor ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
