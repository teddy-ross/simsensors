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

#include <sim_math.hpp>

namespace simsens {

    class SimRangefinder {

        friend class RangefinderVisualizer;
        friend class RobotParser;

        public:

        void read(const pose_t & robot_pose, const vector<Wall *> walls,
                int * distances_mm, vec3_t & dbg_endpoint)
        {
            (void)distances_mm;

            // Get rangefinder beam endpoints
            const double max_distance_m = this->max_distance_mm / 1000;
            const simsens::vec2_t beam_start = {robot_pose.x, robot_pose.y};
            const simsens::vec2_t beam_end = {
                beam_start.x + cos(robot_pose.psi) * max_distance_m,
                beam_start.y - sin(robot_pose.psi) * max_distance_m
            };

            dbg_endpoint.z = -1;
            double dist = INFINITY;

            for (auto wall : walls) {

                vec2_t newdbg_endpoint = {};
                const double newdist = distance_to_wall(
                        beam_start, beam_end, *wall, newdbg_endpoint);

                if (newdist < dist) {
                    dbg_endpoint.x = newdbg_endpoint.x;
                    dbg_endpoint.y = newdbg_endpoint.y;
                    dbg_endpoint.z = robot_pose.z;
                    dist = newdist;
                }
            }

            if (dist > max_distance_m) {
                dist = INFINITY;
                dbg_endpoint.z = -1;
            }

            dump();

            //printf("%3.3f\n", dist);
        }

        void dump()
        {
            printf("Rangefinder: \n");
            printf("  fov: %3.3f rad\n", field_of_view_radians);
            printf("  width: %d\n", width);
            printf("  height: %d\n", height);
            printf("  min range: %d mm\n", min_distance_mm);
            printf("  max range: %d mm\n", max_distance_mm);
            printf("  translation: x=%+3.3fm y=%+3.3fm z=%+3.3fm\n",
                    translation.x, translation.y, translation.z);
            printf("  angles: phi=%+3.3frad theta=%+3.3frad psi=%+3.3frad\n",
                    angles.x, angles.y, angles.z);
            printf("\n");
        }

        private:

        static constexpr double MAX_WORLD_SIZE_M = 500; // arbitrary

        int width;
        int height; 
        int min_distance_mm;
        int max_distance_mm;
        double field_of_view_radians;
        vec3_t translation;
        vec3_t angles;

        static double distance_to_wall(
                const vec2_t beam_start,
                const vec2_t beam_end,
                const Wall & wall,
                vec2_t & dbg_endpoint)
        {
            // Get wall dbg_endpoints
            const auto psi = wall.rotation.alpha; // XXX should use rotvec2euler() conversion
            const auto len = wall.size.y / 2;
            const auto dx = len * sin(psi);
            const auto dy = len * cos(psi);
            const auto tx = wall.translation.x;
            const auto ty = wall.translation.y;
            const auto x3 = tx + dx;
            const auto y3 = ty + dy;
            const auto x4 = tx - dx;
            const auto y4 = ty - dy;

            const auto x1 = beam_start.x;
            const auto y1 = beam_start.y;
            const auto x2 = beam_end.x;
            const auto y2 = beam_end.y;

            // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
            const auto denom = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
            if (!iszero(denom)) {
                const auto px = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / denom;
                const auto py = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / denom;

                // Ensure intersection point within wall bounds
                if (ge(px, x3) && le(px, x4) && ge(py, y4) && le(py, y3)) {
                    dbg_endpoint.x = px;
                    dbg_endpoint.y = py;
                    return eucdist(x1, y1, px, py);
                }
            }

            return INFINITY;
        }

        static bool ge(const double a, const double b)
        {
            return iszero(a-b) || a > b;
        }

        static bool le(const double a, const double b)
        {
            return iszero(a-b) || b > a;
        }

        static bool iszero(const double x)
        {
            return fabs(x) < 0.001; // mm precision
        }

        static double eucdist(double x1, double y1, double x2, double y2)
        {
            const auto xd = (x1 - x2);
            const auto yd = (y1 - y2);
            return sqrt(xd*xd + yd*yd);
        }
    };

}
