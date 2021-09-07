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
/**
 * \file     point.h
 * \author   Bruce JK Huang, Collin Johnson
 *
 * Definition of a simple Point class.
 *
 * Includes, PointHash struct that provides a simple hash for points.
 */

#ifndef POINT_H
#define POINT_H

#include "float_comparision.h"
#include <cmath>
#include <type_traits>

namespace bipedlab
{
/**
 * point3_t<T> represents a simple 3D Cartesian point.
 */
template <typename T>
struct point_t
{
    T x;
    T y;
    T z;

    /**
     * Default constructor for Point.
     */
    point_t(void) : x(0), y(0), z(0) { }

    /**
     * Constructor for Point.
     */
    explicit point_t(T xPos, T yPos, T zPos) : x(xPos), y(yPos), z(zPos)
    {
        // Nothing to do here
    }

    /**
     * Copy constructor for a Point with a different type.
     */
    template <typename U>
    point_t(const point_t<U>& copy) : x(copy.x)
                                  , y(copy.y)
                                  , z(copy.z)
    {
    }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(x, y, z);
    }
};
// typedef point3<T> point3_t;

/**
 * Point<T> represents a simple 2D Cartesian point.
 *
 * NOTE: This struct name violates the naming conventions for the project. This issue is causing me some level of
 *       philosophical duress as I try to rectify the gap in my naming conventions because point_t<T> seems very wrong.
 */
template <typename T>
struct point2d_t
{
    T x;
    T y;

    /**
     * Default constructor for Point.
     */
    point2d_t(void) : x(0), y(0) { }

    /**
     * Constructor for Point.
     */
    explicit point2d_t(T xPos, T yPos) : x(xPos), y(yPos)
    {
        // Nothing to do here
    }

    /**
     * Copy constructor for a Point with a different type.
     */
    template <typename U>
    point2d_t(const point2d_t<U>& copy) : x(copy.x)
                                , y(copy.y)
    {
    }

    explicit point2d_t(const std::pair<T, T> p) : x(p.first), y(p.second)
    {
        // Nothing to do here
    }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(x, y);
    }
};


/**
 * distance_between_points calculates the Cartesian distance between two points.
 */
template <typename T, typename U>
float distance_between_points(T pointAx, T pointAy, U pointBx, U pointBy)
{
    return std::sqrt((pointAx - pointBx) * (pointAx - pointBx) + (pointAy - pointBy) * (pointAy - pointBy));
}

/**
 * distance_between_points calculates the Cartesian distance between two points.
 */
template <typename T, typename U>
float distance_between_points(const point2d_t<T>& pointA, const point2d_t<U>& pointB)
{
    return std::sqrt((pointA.x - pointB.x) * (pointA.x - pointB.x) + (pointA.y - pointB.y) * (pointA.y - pointB.y));
}

/**
 * distance_to_point calculates the radial distance from the origin to the specified point.
 */
template <typename T>
float distance_to_point(const point2d_t<T>& point)
{
    return std::sqrt(point.x * point.x + point.y * point.y);
}

template <typename T, typename U>
float squared_point_distance(const point2d_t<T>& pointA, const point2d_t<U>& pointB)
{
    return (pointA.x - pointB.x) * (pointA.x - pointB.x) + (pointA.y - pointB.y) * (pointA.y - pointB.y);
}

/**
 * point_norm_squared calculates the squared L2 norm of a point (equivalent to the radial distance from the origin.)
 */
template <typename T>
T point_norm_squared(const point2d_t<T>& point)
{
    return point.x * point.x + point.y * point.y;
}

/**
 * point_norm calculates the L2 norm of a point (equivalent to the radial distance from the origin.)
 */
template <typename T>
T point_norm(const point2d_t<T>& point)
{
    return std::sqrt(point_norm_squared(point));
}

/**
 * inner_product_between_points calculates simple inner product between points of the same type.
 */
template <typename T>
T inner_product_between_points(const point2d_t<T>& pointA, const point2d_t<T>& pointB)
{
    return pointA.x * pointB.x + pointA.y * pointB.y;
}

/**
 * angle_between_points finds the angle between two vectors defined by the three points provided as arguments.
 *
 * \param    first               Head of one of the vectors
 * \param    second              Head of the other vector
 * \param    center              Point shared by the two vectors
 * \return   Angle between first and second in the range of [0, PI]. NAN if first == center or second == center.
 */
template <typename T>
float angle_between_points(const point2d_t<T>& first, const point2d_t<T>& second, const point2d_t<T>& center)
{
    // Use the vector form here to get a pretty answer:
    //  cos(theta) = a dot b / ||a||*||b||

    if ((first == center) || (second == center)) {
        return NAN;
    }

    T xA = first.x - center.x;
    T yA = first.y - center.y;
    T xB = second.x - center.x;
    T yB = second.y - center.y;

    double acosTerm = (xA * xB + yA * yB) / (std::sqrt(xA * xA + yA * yA) * std::sqrt(xB * xB + yB * yB));
    if (acosTerm < -1.0) {
        acosTerm = 1.0;
    } else if (acosTerm > 1.0) {
        acosTerm = 1.0;
    }

    return std::acos(acosTerm);
}


/**
 * angle_to_point finds the angle of a vector from head to tail.
 *
 * \param    from                Start of the vector
 * \param    to                  End of the vector
 * \return   Angle of vector atan2(to - from).
 */
template <typename T, typename U>
float angle_to_point(const point2d_t<T>& from, const point2d_t<U>& to)
{
    return std::atan2(to.y - from.y, to.x - from.x);
}


/**
 * rotate applies a rotation matrix to the point, rotating the point by the specified angle.
 *
 * \param    point       Point to be rotated
 * \param    angle       Angle by which to rotate
 * \return   Rotated point.
 */
template <typename T>
point2d_t<T> rotate(const point2d_t<T>& point, float angle)
{
    return point2d_t<T>(point.x * std::cos(angle) - point.y * std::sin(angle),
                    point.x * std::sin(angle) + point.y * std::cos(angle));
}


/**
 * transform applies a transform to the point. The transform first adds the x and y values, then applies
 * a rotation of the specified angle.
 *
 * \param    point       Point to be transformed
 * \param    x           x shift
 * \param    y           y shift
 * \param    angle       Angle to rotate
 * \return   Transformed point.
 */
template <typename T, typename U>
point2d_t<T> transform(const point2d_t<T>& point, U x, U y, U angle)
{
    point2d_t<T> transformed(point.x + x, point.y + y);

    return rotate(transformed, angle);
}


/**
 * homogeneous_transform takes a point, applies a rotation to it, then translates it by the specified (x, y)
 *
 * \param    point       Point to be rotated and translated
 * \param    x           x shift
 * \param    y           y shift
 * \param    angle       Angle to rotate
 * \return   Point after rotation and translation.
 */
template <typename T, typename U>
point2d_t<T> homogeneous_transform(const point2d_t<T>& point, U x, U y, U angle)
{
    point2d_t<T> rotated = rotate(point, angle);
    rotated.x += x;
    rotated.y += y;

    return rotated;
}


// Various useful operator overloads
template <typename T, typename U>
bool operator==(const point2d_t<T>& lhs, const point2d_t<U>& rhs)
{
    // If floating point, then use the fuzzy floating point comparison
    if (std::is_floating_point<T>::value) {
        return absolute_fuzzy_equal(lhs.x, rhs.x) && absolute_fuzzy_equal(lhs.y, rhs.y);
    } else {
        return (lhs.x == rhs.x) && (lhs.y == rhs.y);
    }
}

template <typename T, typename U>
bool operator!=(const point2d_t<T>& lhs, const point2d_t<U>& rhs)
{
    // Go from bottom left to top right
    return !(lhs == rhs);
}

template <typename T, typename U>
bool operator<(const point2d_t<T>& lhs, const point2d_t<U>& rhs)
{
    // Go from bottom left to top right
    return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y));
}

template <class ostream, typename T>
ostream& operator<<(ostream& out, const point2d_t<T>& point)
{
    out << '(' << point.x << ',' << point.y << ')';
    return out;
}

template <typename T>
point2d_t<T> operator-(const point2d_t<T>& rhs)
{
    return point2d_t<T>(-rhs.x, -rhs.y);
}

template <typename T, typename U>
point2d_t<T> operator-(const point2d_t<T>& lhs, const point2d_t<U>& rhs)
{
    return point2d_t<T>(lhs.x - rhs.x, lhs.y - rhs.y);
}

template <typename T, typename U>
point2d_t<T> operator+(const point2d_t<T>& lhs, const point2d_t<U>& rhs)
{
    return point2d_t<T>(lhs.x + rhs.x, lhs.y + rhs.y);
}

template <typename T, typename U>
point2d_t<T> operator*(U lhs, const point2d_t<T>& rhs)
{
    return point2d_t<T>(lhs * rhs.x, lhs * rhs.y);
}

template <typename T, typename U>
point2d_t<T> operator*(const point2d_t<T>& lhs, U rhs)
{
    return point2d_t<T>(lhs.x * rhs, lhs.y * rhs);
}

template <typename T, typename U>
point2d_t<T> operator/(const point2d_t<T>& lhs, U rhs)
{
    return point2d_t<T>(lhs.x / rhs, lhs.y / rhs);
}

template <typename T, typename U>
point2d_t<T>& operator+=(point2d_t<T>& lhs, const point2d_t<U>& rhs)
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;

    return lhs;
}


template <typename T, typename U>
point2d_t<T>& operator-=(point2d_t<T>& lhs, const point2d_t<U>& rhs)
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;

    return lhs;
}

}   // namespace bipedlab

#endif   // POINT_H
