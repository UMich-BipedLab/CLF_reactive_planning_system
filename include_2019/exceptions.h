/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \ Modified by Bruce JK Huang
 */

#ifndef OCCUPANCY_GRID_UTILS_EXCEPTION_H
#define OCCUPANCY_GRID_UTILS_EXCEPTION_H

#include <boost/format.hpp>
#include <stdexcept>
#include <geometry_msgs/Point.h>

namespace occupancy_grid_utils
{

using boost::format;

/// Base class for exceptions from this package
struct GridUtilsException: public std::logic_error
{
  GridUtilsException (const format& f) : std::logic_error(f.str()) {};
  GridUtilsException (const char* str) : std::logic_error(str) {};
};


/// Exception for point off map
struct PointOutOfBoundsException: public GridUtilsException
{
  PointOutOfBoundsException (const geometry_msgs::Point& p) :
    GridUtilsException(format("Point %1%, %2% is off grid") % p.x % p.y) {}
};


/// Exception for Cell off map
struct CellOutOfBoundsException: public GridUtilsException
{
  CellOutOfBoundsException (const unsigned x, const unsigned y) :
    GridUtilsException(format("Cell %1%, %2% is off grid") % x % y) {}
};

/// \brief Exception when data field is not a vector of the right size
struct DataSizeException: public GridUtilsException
{
  DataSizeException (const unsigned expected, const unsigned actual) :
    GridUtilsException (format ("Expected data vector to have size %1%, "
                                "but it has size %2%") % expected % actual) {}
};      


} // namespace occupancy_grid_utils

#endif
