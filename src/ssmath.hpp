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

};
