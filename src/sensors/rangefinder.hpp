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

            static constexpr float RAD2DEG = 180.0f / M_PI;

            int width;
            int height; 
            int min_distance_mm;
            int max_distance_mm;
            double field_of_view_radians;

            void get_intersection(const pose_t & robot_pose, Wall & wall,
                    vec2_t & point)
            {

                // Get wall endpoints
                const auto psi = wall.rotation.z;
                const auto len = wall.size.y / 2;
                const auto dx = len * sin(psi);
                const auto dy = len * cos(psi);
                const auto tx = wall.translation.x;
                const auto ty = wall.translation.y;
                const vec2_t wall_pt1 = {tx + dx, ty + dy};
                const vec2_t wall_pt2 = {tx - dx, ty - dy};

                // Get rangefinder beam endpoints
                const double max_distance_m = this->max_distance_mm / 1000;
                const vec2_t beam_pt1 = {robot_pose.x, robot_pose.y};
                const vec2_t beam_pt2 = {
                    robot_pose.x + cos(robot_pose.psi) * max_distance_m,
                    robot_pose.y - sin(robot_pose.psi) * max_distance_m
                };

                point.x = beam_pt2.x;
                point.y = beam_pt2.y;
            }

        public:

            void read(const pose_t & robot_pose, const vector<Wall *> walls,
                    int * distances_mm, vec2_t & endpoint)
            {
                get_intersection(robot_pose, *walls[1], endpoint);


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
