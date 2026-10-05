#pragma once
#include <stdint.h>

// Kept in a header so the Arduino builder's auto-generated prototypes
// (inserted above the first function in the .ino) can see this type.

#define GEST_LEN          64
#define GEST_AXES         6
#define WIN_BYTES         (GEST_LEN * GEST_AXES)
#define REPS_PER_GESTURE  10

struct ClassTemplates {
  uint32_t magic;
  uint32_t count;
  int8_t   win[REPS_PER_GESTURE][WIN_BYTES];
};
