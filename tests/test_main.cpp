#include "test_framework.hpp"

#include <exception>
#include <iostream>

int main() {
  int failures = 0;
  for (const auto &test : test_framework::registry()) {
    try {
      test.body();
      std::cout << "[PASS] " << test.name << '\n';
    } catch (const std::exception &error) {
      ++failures;
      std::cerr << "[FAIL] " << test.name << ": " << error.what() << '\n';
    }
  }
  std::cout << test_framework::registry().size() - failures << '/'
            << test_framework::registry().size() << " tests passed\n";
  return failures == 0 ? 0 : 1;
}
