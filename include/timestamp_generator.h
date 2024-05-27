#ifndef TIMESTAMP_GENERATOR_H
#define TIMESTAMP_GENERATOR_H
#include <stdint.h>

long long generate_timestamp(uint64_t count) {
  static const long long GENERAL_BASIC_TIME =
      1640966400000000;  // beijing time : 2022.01.01 00:00:00; us

  long long stamp =
      GENERAL_BASIC_TIME + count * 10000 + (long long) 8 * 3600 * 1000000;
  return stamp;
}
#endif