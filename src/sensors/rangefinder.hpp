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

        public:

        void read(const pose_t & robot_pose, const vector<Wall *> walls,
                int * distances_mm, vec3_t & endpoint)
        {
            // Get rangefinder beam endpoints
            const double max_distance_m = this->max_distance_mm / 1000;
            const auto x1 = robot_pose.x;
            const auto y1 = robot_pose.y;
            const auto x2 = robot_pose.x + cos(robot_pose.psi) * max_distance_m;
            const auto y2 = robot_pose.y - sin(robot_pose.psi) * max_distance_m;

            endpoint.z = -1;

            for (auto wall : walls) {

                // Get wall endpoints
                const auto psi = wall->rotation.z;
                const auto len = wall->size.y / 2;
                const auto dx = len * sin(psi);
                const auto dy = len * cos(psi);
                const auto tx = wall->translation.x;
                const auto ty = wall->translation.y;
                const auto x3 = tx + dx;
                const auto y3 = ty + dy;
                const auto x4 = tx - dx;
                const auto y4 = ty - dy;

                // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
                const auto denom = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
                if (!eqz(denom)) {
                    const auto px = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / denom;
                    const auto py = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / denom;
                    if (ge(px, x3) && le(px, x4) && ge(py, y4) && le(py, y3)) {
                        endpoint.x = px;
                        endpoint.y = py;
                        endpoint.z = robot_pose.z;
                    }
                }

                // XXX show a diagonal pattern for now
                for (int k=0; k<this->width; ++k) {
                    distances_mm[k*this->height+k] = -1;
                }
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

        private:

        static constexpr double MAX_WORLD_SIZE_M = 1000; // arbitrary

        int width;
        int height; 
        int min_distance_mm;
        int max_distance_mm;
        double field_of_view_radians;

        bool ge(const double a, const double b)
        {
            return eqz(a-b) || a > b;
        }

        bool le(const double a, const double b)
        {
            return eqz(a-b) || b > a;
        }

        bool eqz(const double x)
        {
            return fabs(x) < 0.001; // mm precision
        }

    };

}
