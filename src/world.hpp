/* 
   Simple world representation for robot simulation

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

#include <simsensors/src/obstacles/wall.hpp>

namespace simsens {

    class World {

        friend class WorldParser;
        friend class Rangefinder;
        friend class CollisionDetector;

        private:

            vector<Wall *> walls;

            pose_t robotPose;

            bool y_inverted;

            vec3_t adjust_location(const vec3_t & loc)
            {
                return {loc.x, yinvert(loc.y), loc.z};
            }

            pose_t adjust_pose(const pose_t & pose)
            {
                return {pose.x, yinvert(pose.y), pose.z,
                        pose.phi, pose.theta, pose.psi};
            }

            double yinvert(const double y)
            {
                return y_inverted ? -y : y;
            }

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

         public:

            bool collided(
                    const vec3_t & robot_location, const bool debug=false)
            {
                const auto robloc = adjust_location(robot_location);

                for (auto wall : walls) {

                    if (
                            intersect_with_wall_at_azimuth(robloc, *wall, 0) 
                            || intersect_with_wall_at_azimuth(robloc, *wall, M_PI/2)
                            || intersect_with_wall_at_azimuth(robloc, *wall, M_PI) 
                            || intersect_with_wall_at_azimuth(robloc, *wall, 3*M_PI/2)
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


            pose_t getRobotPose()
            {
                return robotPose;
            }

            void dump()
            {
                for (auto wall : walls) {
                    wall->dump();
                }
            }

    };
}
