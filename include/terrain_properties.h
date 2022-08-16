/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.BrucebotStudio.com/
 */
#ifndef TERRAIN_FRICTION_H
#define TERRAIN_FRICTION_H

#include <string>


namespace bipedlab
{
namespace terrain_properties
{
    // static const char *terrain_types[] =
    // {
    //     "road",                      // 0: road
    //     "sidewalk",                  // 1: sidewalk
    //     "terrain",                   // 2: terrain
    //     "building",                  // 3: bulding
    //     "vegetation",                // 4: vegetation
    //     "walkable",                  // 5: walkable
    // };


    constexpr float terrain_friction[] = 
    {
        1,                  // 0: road
        1,                  // 1: sidewalk
        0.3,                // 2: terrain
        1,                  // 3: bulding
        0.3,                // 4: vegetation
        1,                  // 5: walkable
    };

    constexpr std::string_view  terrain_type[] = 
    {
        "road",                  // 0: road
        "sidewalk",                  // 1: sidewalk
        "terrain",                // 2: terrain
        "building",                  // 3: bulding
        "vegetation",                // 4: vegetation
        "walkable",                  // 5: walkable
    };


// struct EL_TABLE
// {
//    constexpr EL_TABLE() : values()
//    {
//       for (auto i = 0; i < 32; ++i) {
//          values[i] = tan(el[i]*M_PI/180);
//       }
//    }
// 
//    int values[32];
// };
// 
// constexpr EL_TABLE EL_TAN = EL_TABLE();

} // namespace terrain_friction
} // namespace BipedLab

#endif
