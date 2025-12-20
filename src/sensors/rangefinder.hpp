/* 
   Rangefinder simulator

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

    class SimRangefinder {

        friend class RangefinderVisualizer;
        friend class RobotParser;

        private:

            static constexpr double MAX_WORLD_SIZE_M = 1000; // arbitrary

            int width;
            int height; 
            int min_distance_mm;
            int max_distance_mm;
            double field_of_view_radians;

        public:

            void read(const pose_t & robot_pose, const vector<Wall *> walls,
                    int * distances_mm, double & end_x, double & end_y)
            {
                (void)walls;
                /*
                printf("x=%+3.3fm y=%+3.3fm z=%+3.3fm | ",
                        robot_pose.x, robot_pose.y, robot_pose.z);
                printf("phi=%+3.3f theta=%+3.3f psi=%+3.3f\n",
                        robot_pose.phi, robot_pose.theta, robot_pose.psi);
                walls[0]->dump();
                        */

                const double max_distance_m = this->max_distance_mm / 1000;

                end_x = robot_pose.x + cos(robot_pose.psi) * max_distance_m;
                end_y = robot_pose.y - sin(robot_pose.psi) * max_distance_m;


                // XXX show a diagonal pattern for now
                for (int k=0; k<this->width; ++k) {
                    distances_mm[k*this->height+k] = -1;
                }
            }

            void dump()
            {
                printf("Rangefinder: \n");

                printf("  fov: %3.3f rad\n", field_of_view_radians);
                printf("  width: %d\n", width);
                printf("  height: %d\n", height);
                printf("  min range: %d mm\n", min_distance_mm);
                printf("  max range: %d mm\n", max_distance_mm);

                printf("\n");
            }
    };

}
