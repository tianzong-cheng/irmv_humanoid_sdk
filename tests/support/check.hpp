#pragma once

#include <iostream>

#define CHECK(condition)                                     \
  do {                                                       \
    if (!(condition)) {                                      \
      std::cerr << __FILE__ << ':' << __LINE__               \
                << ": CHECK failed: " << #condition << '\n'; \
      return 1;                                              \
    }                                                        \
  } while (false)
