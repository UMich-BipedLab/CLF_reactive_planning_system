#include "planner_info_to_controller_t.h"
#include <stddef.h>
#include <string.h>

void pack_planner_info_to_controller_t(const planner_info_to_controller_t *bus,
  unsigned char bytes[2136])
{
  float x;
  unsigned char y[4];
  int i0;
  unsigned char b_y[12];
  float b_x[3];
  unsigned char c_y[4];
  unsigned char d_y[4];
  unsigned char e_y[4];
  unsigned char f_y[1600];
  float c_x[400];
  unsigned char g_y[240];
  float d_x[60];
  unsigned char h_y[240];
  unsigned char i_y[28];
  float e_x[7];
  x = (float)bus->behavior;
  memcpy((void *)&y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  for (i0 = 0; i0 < 3; i0++) {
    b_x[i0] = (float)bus->velocity[i0];
  }

  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)12 * sizeof
          (unsigned char)));
  x = (float)bus->torso.roll;
  memcpy((void *)&c_y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  x = (float)bus->torso.pitch;
  memcpy((void *)&d_y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  x = (float)bus->torso.yaw;
  memcpy((void *)&e_y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  for (i0 = 0; i0 < 400; i0++) {
    c_x[i0] = (float)bus->terrain[i0];
  }

  memcpy((void *)&f_y[0], (void *)&c_x[0], (unsigned int)((size_t)1600 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 60; i0++) {
    d_x[i0] = (float)bus->waypoints[i0];
  }

  memcpy((void *)&g_y[0], (void *)&d_x[0], (unsigned int)((size_t)240 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 60; i0++) {
    d_x[i0] = (float)bus->footplacement[i0];
  }

  memcpy((void *)&h_y[0], (void *)&d_x[0], (unsigned int)((size_t)240 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 7; i0++) {
    e_x[i0] = (float)bus->pose[i0];
  }

  memcpy((void *)&i_y[0], (void *)&e_x[0], (unsigned int)((size_t)28 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 4; i0++) {
    bytes[i0] = y[i0];
  }

  for (i0 = 0; i0 < 12; i0++) {
    bytes[i0 + 4] = b_y[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    bytes[i0 + 16] = c_y[i0];
    bytes[i0 + 20] = d_y[i0];
    bytes[i0 + 24] = e_y[i0];
  }

  memcpy(&bytes[28], &f_y[0], 1600U * sizeof(unsigned char));
  for (i0 = 0; i0 < 240; i0++) {
    bytes[i0 + 1628] = g_y[i0];
    bytes[i0 + 1868] = h_y[i0];
  }

  for (i0 = 0; i0 < 28; i0++) {
    bytes[i0 + 2108] = i_y[i0];
  }
}





void unpack_planner_info_to_controller_t(const unsigned char bytes[2136],
  planner_info_to_controller_t *bus)
{
  int i;
  float y;
  unsigned char x[4];
  float b_y[3];
  unsigned char b_x[12];
  float c_y;
  float d_y;
  unsigned char c_x[1600];
  float e_y[400];
  unsigned char d_x[240];
  float f_y[60];
  float g_y[7];
  unsigned char e_x[28];
  for (i = 0; i < 4; i++) {
    x[i] = bytes[i];
  }

  memcpy((void *)&y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  bus->behavior = y;
  for (i = 0; i < 12; i++) {
    b_x[i] = bytes[i + 4];
  }

  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)3 * sizeof
          (float)));
  for (i = 0; i < 3; i++) {
    bus->velocity[i] = b_y[i];
  }

  for (i = 0; i < 4; i++) {
    x[i] = bytes[i + 16];
  }

  memcpy((void *)&y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  for (i = 0; i < 4; i++) {
    x[i] = bytes[i + 20];
  }

  memcpy((void *)&c_y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  for (i = 0; i < 4; i++) {
    x[i] = bytes[i + 24];
  }

  memcpy((void *)&d_y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  bus->torso.roll = y;
  bus->torso.pitch = c_y;
  bus->torso.yaw = d_y;
  memcpy(&c_x[0], &bytes[28], 1600U * sizeof(unsigned char));
  memcpy((void *)&e_y[0], (void *)&c_x[0], (unsigned int)((size_t)400 * sizeof
          (float)));
  for (i = 0; i < 400; i++) {
    bus->terrain[i] = e_y[i];
  }

  memcpy(&d_x[0], &bytes[1628], 240U * sizeof(unsigned char));
  memcpy((void *)&f_y[0], (void *)&d_x[0], (unsigned int)((size_t)60 * sizeof
          (float)));
  for (i = 0; i < 60; i++) {
    bus->waypoints[i] = f_y[i];
  }

  memcpy(&d_x[0], &bytes[1868], 240U * sizeof(unsigned char));
  memcpy((void *)&f_y[0], (void *)&d_x[0], (unsigned int)((size_t)60 * sizeof
          (float)));
  for (i = 0; i < 60; i++) {
    bus->footplacement[i] = f_y[i];
  }

  for (i = 0; i < 28; i++) {
    e_x[i] = bytes[i + 2108];
  }

  memcpy((void *)&g_y[0], (void *)&e_x[0], (unsigned int)((size_t)7 * sizeof
          (float)));
  for (i = 0; i < 7; i++) {
    bus->pose[i] = g_y[i];
  }
}
