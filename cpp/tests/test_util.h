/// @file test_util.h
/// @brief Minimal check macro and helpers shared by the test executables.

#ifndef G1MOTOR_TESTS_TEST_UTIL_H_
#define G1MOTOR_TESTS_TEST_UTIL_H_

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace g1motor_test {

/// Number of failed checks in this executable.
inline int g_failures = 0;

/// @brief Converts a hex string such as "feee00" to bytes.
inline std::vector<std::uint8_t> hex(const std::string& text) {
  std::vector<std::uint8_t> bytes;
  for (std::size_t i = 0; i + 1 < text.size(); i += 2) {
    bytes.push_back(static_cast<std::uint8_t>(
        std::strtoul(text.substr(i, 2).c_str(), nullptr, 16)));
  }
  return bytes;
}

/// @brief Prints the result and returns the process exit code.
inline int finish(const char* name) {
  if (g_failures == 0) {
    std::printf("%s: all checks passed\n", name);
    return 0;
  }
  std::printf("%s: %d check(s) failed\n", name, g_failures);
  return 1;
}

}  // namespace g1motor_test

/// Records a failure, with the line and expression, if cond is false.
#define CHECK(cond)                                         \
  do {                                                      \
    if (!(cond)) {                                          \
      std::printf("FAILED line %d: %s\n", __LINE__, #cond); \
      ++g1motor_test::g_failures;                           \
    }                                                       \
  } while (0)

#endif  // G1MOTOR_TESTS_TEST_UTIL_H_
