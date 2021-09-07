#include "waypoint_data_t.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

void pack_waypoint_data_t(const waypoint_data_t *bus, unsigned char bytes[188])
{
  int i0;
  unsigned char y[28];
  float x[7];
  unsigned char b_y[160];
  float b_x[40];


  // copy pose to x
  for (i0 = 0; i0 < 7; i0++) {
    x[i0] = (float)bus->pose[i0];
    // printf("x: %f\n", x[i0]);
  }

  // copy x to y
  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)28 * sizeof
          (unsigned char)));

  // for (i0 = 0; i0 < 28; i0++) {
  //     printf("y: %d\n", y[i0]);
  // }

  // copy waypoints to b_x
  for (i0 = 0; i0 < 40; i0++) {
    b_x[i0] = (float)bus->waypoints[i0];
  }

  // copyt b_x to b_y
  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)160 * sizeof
          (unsigned char)));


  // copy y to bytes
  // i.e., copy pose bytes
  for (i0 = 0; i0 < 28; i0++) {
    bytes[i0] = y[i0];
  }

  // copy b_y to bypes
  // i.e., copy waypoints to bytes
  memcpy(&bytes[28], &b_y[0], 160U * sizeof(unsigned char));

  // for (i0 = 0; i0 < 188; i0++) {
  //     printf("%d ", bytes[i0]);
  // }
  // bytes[0] = 10;
  // bytes[1] = 20;
  // bytes[2] = 10;
  // bytes[3] = 10;
  // bytes[4] = 10;
  // size_t msg_length = strlen((char*) bytes);
  // printf("\nsize: %d \n", (int) msg_length);

}


// void pack_waypoint_data_t(const waypoint_data_t *bus, unsigned char bytes[188])
// {
//   int i0;
//   unsigned char y[28];
//   float x[7];
//   unsigned char b_y[160];
//   float b_x[40];
//   for (i0 = 0; i0 < 7; i0++) {
//     x[i0] = (float)bus->pose[i0];
//   }
//
//   memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)28 * sizeof
//           (unsigned char)));
//   for (i0 = 0; i0 < 40; i0++) {
//     b_x[i0] = (float)bus->waypoints[i0];
//   }
//
//   memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)160 * sizeof
//           (unsigned char)));
//   for (i0 = 0; i0 < 28; i0++) {
//     bytes[i0] = y[i0];
//   }
//
//   memcpy(&bytes[28], &b_y[0], 160U * sizeof(unsigned char));
// }

void unpack_waypoint_data_t(const unsigned char bytes[188], waypoint_data_t *bus)
{
  int i;
  float y[7];
  unsigned char x[28];
  unsigned char b_x[160];
  float b_y[40];
  for (i = 0; i < 28; i++) {
    x[i] = bytes[i];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)7 * sizeof(float)));
  for (i = 0; i < 7; i++) {
    bus->pose[i] = y[i];
  }

  memcpy(&b_x[0], &bytes[28], 160U * sizeof(unsigned char));
  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)40 * sizeof
          (float)));
  for (i = 0; i < 40; i++) {
    bus->waypoints[i] = b_y[i];
  }
}




