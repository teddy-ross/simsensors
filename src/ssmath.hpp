/* 
   Math for simulation

   Copyright (C) 2025 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <math.h>

#include <sstypes.h>
#include <obstacles/wall.hpp>

namespace simsens {

    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
    void rotation_to_euler(const rotation_t & rotation, vec3_t & angles)
    {
        static constexpr double TOL = 2e-3;

        const auto x = rotation.x;
        const auto y = rotation.y;
        const auto z = rotation.z;
        const auto alpha = rotation.alpha;

        const auto s = sin(alpha);
        const auto c = cos(alpha);
        const auto t = 1 - c;

        //  if axis is not already normalised then uncomment this
        // magnitude = sqrt(x*x + y*y + z*z)
        // if (magnitude==0) throw error
        // x /= magnitude
        // y /= magnitude
        // z /= magnitude

        const auto sing = x*y*t + z*s;

        if (sing > (1-TOL)) { // north pole singularity detected
            angles.x = 0;
            angles.y = 2*atan2(x*sin(alpha/2),cos(alpha/2));
            angles.z = M_PI/2;
        }

        else if (sing < -(1-TOL)) { // south pole singularity detected
            angles.x = 0;
            angles.y = -2*atan2(x*sin(alpha/2),cos(alpha/2));
            angles.z = -M_PI/2;
        }

        else {
            angles.x = atan2(x * s - y * z * t , 1 - (x*x + z*z) * t);
            angles.y = atan2(y * s- x * z * t , 1 - (y*y+ z*z ) * t);
            angles.z = asin(x * y * t + z * s);
        }
    }

    // https://gist.github.com/kylemcdonald/6132fc1c29fd3767691442ba4bc84018
    static bool line_segments_intersect(
            const double x1, const double y1,
            const double x2, const double y2,
            const double x3, const double y3,
            const double x4, const double y4,
            double & px, double & py)
    {
        const auto denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

        if (denom != 0) {

            const auto ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
            const auto ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

            // Check if intersection point lies within both line segments (0 <=
            // ua <= 1 and 0 <= ub <= 1)
            if (0 <= ua && ua <= 1 && 0 <= ub && ub <= 1) {

                px = x1 + ua * (x2 - x1);
                py = y1 + ua * (y2 - y1);

                return true;
            }
        }

        return false;
    }

    static double sqr(const double x)
    {
        return x * x;
    }

    static double intersect_with_wall(
            const pose_t robot_pose,
            const double azimuth_angle,
            const double elevation_angle,
            const Wall & wall,
            vec3_t * intersection=nullptr)
    {
        static constexpr double MAX_WORLD_DIM_M = 20; // arbitrary

        // Calculate beam endpoints
        const vec2_t beam_start_xy = {robot_pose.x, robot_pose.y};
        const vec2_t beam_end_xy = {
            robot_pose.x + cos(azimuth_angle) * MAX_WORLD_DIM_M,
            robot_pose.y - sin(azimuth_angle) * MAX_WORLD_DIM_M,
        };


        // Get wall endpoints
        const auto psi = wall.rotation.alpha; // rot.  always 0 0 1 alpha
        const auto len = wall.size.y / 2;
        const auto wall_dx = len * sin(psi);
        const auto wall_dy = len * cos(psi);
        const auto wall_tx = wall.translation.x;
        const auto wall_ty = wall.translation.y;

        // If beam ((x1,y1),(x2,y2)) intersects with with wall
        // ((x3,y3),(x4,y4)) 
        double px=0, py=0;
        if (line_segments_intersect(
                    beam_start_xy.x, beam_start_xy.y,
                    beam_end_xy.x, beam_end_xy.y,
                    wall_tx + wall_dx, wall_ty + wall_dy,
                    wall_tx - wall_dx, wall_ty - wall_dy,
                    px, py)) {

            // Use intersection (px,py) to calculate XY distance to wall
            const auto dx = beam_start_xy.x - px;
            const auto dy = beam_start_xy.y - py;
            const auto xydist = sqrt(dx*dx + dy*dy);

            // Use XY distance, robot Z, and elevation angle to calculate Z
            // offset of intersection on wall w.r.t. robot Z
            const auto dz = -tan(elevation_angle) * xydist;

            // Calculate XYZ distance by including Z offset and wall
            // thickness
            const auto xyzdist = sqrt(dx*dx + dy*dy + dz*dz)
                - wall.size.x / 2;

            // Calculate Z in world coordinates
            const auto pz = robot_pose.z + dz;

            // If Z is below wall and XYZ distance is shorter than
            // current, update current
            if (pz < wall.size.z) {
                if (intersection) {
                    intersection->x = px;
                    intersection->y = py;
                    intersection->z = pz;
                }
                return xyzdist;
            }
        }

        return INFINITY;
    }
};
