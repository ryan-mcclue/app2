// SPDX-License-Identifier: zlib-acknowledgement
#pragma once

#include <stdint.h>
#include <string.h>
#include <errno.h>

#define INTERNAL static
#define GLOBAL static

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint8_t u8;
typedef unsigned int uint;
typedef float r32;

#if defined(APP_DEV_BUILD)
  INTERNAL inline void
  bp(void)
  {
    return;
  }
  #define BP() bp()
  INTERNAL inline void
  ebp(void)
  {
    char *err_msg = strerror(errno);
    return;
  }
  #define EBP() ebp()
#else
  #define BP()
  #define EBP()
#endif
