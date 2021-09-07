/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     egocentric_coordinates.h
 * \author   Jong Jin Park
 *
 * Declaration of egocentric polar coordinates, which describes a relation
 * between between two poses on a plane.
 * See [Park-11] (original) and [Park-15] (updated) for further description.
 */

#pragma once

#ifndef EGOCENTRIC_COORDINATES_H
#define EGOCENTRIC_COORDINATES_H

namespace bipedlab
{

/**
 * line_of_sight_t encodes range and orientation of the line of sight from the position of an observer to a target.
 */
struct line_of_sight_t
{
    double range;   // length of the line of sight (radial distance to target)
    double angle;   // orientation of the line of sight in global coordinates
};

/**
 * reduced_egocentric_polar_coords_t encodes a 2D point in egocentric polar coords, relative to a target pose.
 */
struct reduced_egocentric_polar_coords_t
{
    double r;                  // length of the line of sight (radial distance to target)
    double phi;                // angle of target orientation measured from the line of sight
    double lineOfSightAngle;   // in global coords. Useful to keep around.
};

/**
 * egocentric_polar_coords_t encodes a 3D pose in egocentric polar coords relative to a target pose.
 * This coordinate system is useful for rigid body objects on 2D plane fixating on a target.
 * It is originally introduced in [Park-11]. See [Park-15] for updated definition.
 */
struct egocentric_polar_coords_t
{
    double r;                  // length of the line of sight (radial distance to target)
    double delta;              // angle of robot orientation measured from the line of sight
    double phi;                // angle of target orientation measured from the line of sight
    double lineOfSightAngle;   // in global coords. Useful to keep around.
};

// Serialization support -- only need this for egocentric_polar_coords_t
template <class Archive>
void serialize(Archive& ar, egocentric_polar_coords_t& coords)
{
    ar(coords.r, coords.phi, coords.delta, coords.lineOfSightAngle);
}

}   // namespace bipedlab

#endif   // EGOCENTRIC_POLAR_COORDINATES_H

