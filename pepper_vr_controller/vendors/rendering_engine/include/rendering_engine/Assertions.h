#include <stdio.h>

#define DEBUG

#ifdef DEBUG
    #define RE_ENABLE_ASSERTS
#endif

#ifdef RE_ENABLE_ASSERTS
    #define RE_ASSERT(x, ...) { if(!(x)) {printf("ERROR: Assertion Failed: %s\n", __VA_ARGS__); __builtin_trap(); } }
#else
    #define RE_ASSERT(x, ...)
#endif