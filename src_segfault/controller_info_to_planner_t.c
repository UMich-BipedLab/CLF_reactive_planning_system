#include "controller_info_to_planner_t.h"
#include <stddef.h>
#include <string.h>





void pack_controller_info_to_planner_t(const controller_info_to_planner_t *bus,
  unsigned char bytes[52])
{
  int i0;
  unsigned char y[12];
  float x[3];
  unsigned char b_y[12];
  unsigned char c_y[28];
  float b_x[7];
  for (i0 = 0; i0 < 3; i0++) {
    x[i0] = (float)bus->p_com[i0];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)12 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 3; i0++) {
    x[i0] = (float)bus->p_st_foot[i0];
  }

  memcpy((void *)&b_y[0], (void *)&x[0], (unsigned int)((size_t)12 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 7; i0++) {
    b_x[i0] = (float)bus->pose[i0];
  }

  memcpy((void *)&c_y[0], (void *)&b_x[0], (unsigned int)((size_t)28 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 12; i0++) {
    bytes[i0] = y[i0];
    bytes[i0 + 12] = b_y[i0];
  }

  for (i0 = 0; i0 < 28; i0++) {
    bytes[i0 + 24] = c_y[i0];
  }
}

void unpack_controller_info_to_planner_t(const unsigned char bytes[52],
  controller_info_to_planner_t *bus)
{
  int i;
  float y[3];
  unsigned char x[12];
  float b_y[7];
  unsigned char b_x[28];
  for (i = 0; i < 12; i++) {
    x[i] = bytes[i];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)3 * sizeof(float)));
  for (i = 0; i < 3; i++) {
    bus->p_com[i] = y[i];
  }

  for (i = 0; i < 12; i++) {
    x[i] = bytes[i + 12];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)3 * sizeof(float)));
  for (i = 0; i < 3; i++) {
    bus->p_st_foot[i] = y[i];
  }

  for (i = 0; i < 28; i++) {
    b_x[i] = bytes[i + 24];
  }

  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)7 * sizeof
          (float)));
  for (i = 0; i < 7; i++) {
    bus->pose[i] = b_y[i];
  }
}
