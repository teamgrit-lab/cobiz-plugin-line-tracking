#pragma once

#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace test_framework {

struct TestCase {
  std::string name;
  std::function<void()> body;
};

inline std::vector<TestCase> &registry() {
  static std::vector<TestCase> tests;
  return tests;
}

class Registration {
public:
  Registration(std::string name, std::function<void()> body) {
    registry().push_back({std::move(name), std::move(body)});
  }
};

inline void require(bool condition, const char *expression, const char *file,
                    int line) {
  if (!condition) {
    std::ostringstream message;
    message << file << ':' << line << ": requirement failed: " << expression;
    throw std::runtime_error(message.str());
  }
}

inline void near(double actual, double expected, double tolerance,
                 const char *file, int line) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    std::ostringstream message;
    message << file << ':' << line << ": expected " << expected << " +/- "
            << tolerance << ", got " << actual;
    throw std::runtime_error(message.str());
  }
}

} // namespace test_framework

#define LT_CONCAT_INNER(left, right) left##right
#define LT_CONCAT(left, right) LT_CONCAT_INNER(left, right)
#define LT_TEST(name)                                                          \
  static void LT_CONCAT(test_body_, __LINE__)();                               \
  static test_framework::Registration LT_CONCAT(test_registration_, __LINE__)( \
      name, LT_CONCAT(test_body_, __LINE__));                                  \
  static void LT_CONCAT(test_body_, __LINE__)()
#define LT_REQUIRE(expression)                                                 \
  test_framework::require(static_cast<bool>(expression), #expression,          \
                          __FILE__, __LINE__)
#define LT_NEAR(actual, expected, tolerance)                                   \
  test_framework::near((actual), (expected), (tolerance), __FILE__, __LINE__)
#define LT_THROWS(expression)                                                  \
  do {                                                                         \
    bool thrown = false;                                                       \
    try {                                                                      \
      (void)(expression);                                                      \
    } catch (const std::exception &) {                                         \
      thrown = true;                                                           \
    }                                                                          \
    LT_REQUIRE(thrown);                                                        \
  } while (false)
