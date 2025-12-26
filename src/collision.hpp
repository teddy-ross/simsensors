/* 
   Collision detection

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

#include <vector>
using namespace::std;

#include <simsensors/src/math.hpp>
#include <simsensors/src/types.h>
#include <simsensors/src/obstacles/wall.hpp>

namespace simsens {

    class CollisionDetector {

        public:

            static bool detect(
                    const vec3_t & robot_location, const vector<Wall *> walls,
                    const bool debug=false)
            {
                for (auto wall : walls) {

                    if (
                            intersect_with_wall_at_azimuth(robot_location, *wall, 0) 
                            || intersect_with_wall_at_azimuth(robot_location, *wall, M_PI/2)
                            || intersect_with_wall_at_azimuth(robot_location, *wall, M_PI) 
                            || intersect_with_wall_at_azimuth(robot_location, *wall, 3*M_PI/2)
                       ) 
                    {
                        if (debug) {
                            printf("collided with wall: %s\n", wall->name);
                        }
                        return true;
                    }
                }

                return false;
            }

        private:

            // Arbitrary limits
            static constexpr double COLLISION_TOLERANCE_M = 0.05;

            static bool intersect_with_wall_at_azimuth(
                    const vec3_t & robot_location,
                    const Wall & wall,
                    const double azimuth)
            {
                return intersect_with_wall(
                        robot_location,
                        azimuth,
                        0, // elevation
                        wall)

                    < COLLISION_TOLERANCE_M;
            }
    };
}
