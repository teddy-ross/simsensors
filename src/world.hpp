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

        public:

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

            void report()
            {
                for (auto wall : walls) {
                    wall->dump();
                }
            }

            double yinvert(const double y)
            {
                return y_inverted ? -y : y;
            }

            pose_t getRobotPose()
            {
                return robotPose;
            }

    };
}
