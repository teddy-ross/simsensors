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

namespace simsens {

    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
    void rotation2euler(const rotation_t & rotation, vec3_t & angles)
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
};
