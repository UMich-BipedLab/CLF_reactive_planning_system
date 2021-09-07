/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     egocentric_coordinates.h
 * \author  Bruce JK Huang
 *
 * Declaration of egocentric polar coordinates, which describes a relation
 * between between two poses on a plane.
 */

#pragma once

#ifndef OMNI_EGO_COORDINATES_H
#define OMNI_EGO_COORDINATES_H


namespace bipedlab
{
/** NOTE
 * -------- org --------    -------- new --------
 *         range                    range
 *         angle                     phi
 *          phi                     delta
 *         delta                    theta
 *    ling_of_sight_angle    ling_of_sight_angle
 *
 */

/**
 * line_of_sight_t encodes range and orientation of the line of sight from the position of an observer to a target.
 */
struct omni_line_of_sight_t
{
    double range;   // length of the line of sight (radial distance to target)
    double phi;   // orientation of the line of sight in global coordinates
};

/**
 * reduced_egocentric_polar_coords_t encodes a 2D point in egocentric polar coords, relative to a target pose.
 */
struct reduced_omni_ego_polar_coords_t
{
    double r;                  // length of the line of sight (radial distance to target)
    double delta;                // angle of target orientation measured from the line of sight
    double line_of_sight_angle;   // in global coords. Useful to keep around.
};

/**
 * egocentric_polar_coords_t encodes a 3D pose in egocentric polar coords relative to a target pose.
 * This coordinate system is useful for rigid body objects on 2D plane fixating on a target.
 * It is originally introduced in [Park-11]. See [Park-15] for updated definition.
 */
struct omni_ego_polar_coords_t
{
    double r;                  // length of the line of sight (radial distance to target)
    double theta;              // angle of robot orientation measured in global coords 
    double delta;                // angle of target orientation measured from the line of sight
    double line_of_sight_angle;   // in global coords. Useful to keep around.
};

// Serialization support -- only need this for egocentric_polar_coords_t
template <class Archive>
void serialize(Archive& ar, omni_ego_polar_coords_t& coords)
{
    ar(coords.r, coords.theta, coords.delta, coords.line_of_sight_angle);
}

}   // namespace bipedlab

#endif   // EGOCENTRIC_POLAR_COORDINATES_H

