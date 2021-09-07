#include "cassie_to_planner_data_t.h"
#include <stddef.h>
#include <string.h>





void pack_cassie_to_planner_data_t(const cassie_to_planner_data_t *bus, unsigned
  char bytes[24])
{
  int i0;
  unsigned char y[12];
  float x[3];
  unsigned char b_y[12];
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
  for (i0 = 0; i0 < 12; i0++) {
    bytes[i0] = y[i0];
    bytes[i0 + 12] = b_y[i0];
  }
}

void unpack_cassie_to_planner_data_t(const unsigned char bytes[24],
  cassie_to_planner_data_t *bus)
{
  int i;
  float y[3];
  unsigned char x[12];
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
}
