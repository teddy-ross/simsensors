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
                int * distances_mm, vec3_t & dbg_intersection)
        {
            (void)distances_mm;

            // Get rangefinder position and orientation and w.r.t. vehicle
            vec3_t rangefinder_angles= {};
            rotation_to_euler(rotation, rangefinder_angles);
            const auto psi = robot_pose.psi + rangefinder_angles.z;

            // Beam starts at robot coordinates
            const simsens::vec2_t beam_start = {
                robot_pose.x, 
                robot_pose.y
            };

            // Calculate beam endpoint
            const simsens::vec2_t beam_end = {
                beam_start.x + cos(psi) * max_distance_m,
                beam_start.y - sin(psi) * max_distance_m
            };

            dbg_intersection.z = -1;
            double dist = INFINITY;

            for (auto wall : walls) {

                if (robot_pose.z < wall->size.z) { // XXX should use beam Z instead of robot Z

                    vec2_t newdbg_intersection = {};
                    const double newdist = distance_to_wall(
                            beam_start, beam_end, *wall, newdbg_intersection);

                    if (newdist < dist) {
                        dbg_intersection.x = newdbg_intersection.x;
                        dbg_intersection.y = newdbg_intersection.y;
                        dbg_intersection.z = robot_pose.z;
                        dist = newdist;
                    }
                }
            }

            if (dist > max_distance_m) {
                dist = INFINITY;
                dbg_intersection.z = -1;
            }

            // Subtract sensor offset from distance
            dist -= sqrtl2(this->translation.x, this->translation.y);

            printf("dist=%3.3f\n", dist);
        }

        void dump()
        {
            printf("Rangefinder: \n");
            printf("  fov: %3.3fr\n", field_of_view_radians);
            printf("  width: %d\n", width);
            printf("  height: %d\n", height);
            printf("  min range: %3.3fm\n", min_distance_m);
            printf("  max range: %3.3fm\n", max_distance_m);
            printf("  translation: x=%+3.3fm y=%+3.3fm z=%+3.3fm\n",
                    translation.x, translation.y, translation.z);
            printf("  rotation: x=%+3.3f y=%+3.3f z=%+3.3f alpha=%+3.3fr\n",
                    rotation.x, rotation.y, rotation.z, rotation.alpha);
            printf("\n");
        }

        private:

        static constexpr double MAX_WORLD_SIZE_M = 500; // arbitrary

        int width;
        int height; 
        double min_distance_m;
        double max_distance_m;
        double field_of_view_radians;
        vec3_t translation;
        rotation_t rotation;

        static double distance_to_wall(
                const vec2_t beam_start,
                const vec2_t beam_end,
                const Wall & wall,
                vec2_t & dbg_intersection)
        {
            // Get wall endpoints
            const auto psi = wall.rotation.alpha; // rotation always 0 0 1 alpha
            const auto len = wall.size.y / 2;
            const auto dx = len * sin(psi);
            const auto dy = len * cos(psi);
            const auto tx = wall.translation.x;
            const auto ty = wall.translation.y;

            double px=0, py=0;

            if (line_segments_intersect(
                        beam_start.x, beam_start.y,
                        beam_end.x, beam_end.y,
                        tx + dx, ty + dy,
                        tx - dx, ty - dy,
                        px, py))
            {
                dbg_intersection.x = px;
                dbg_intersection.y = py;

                return eucdist(beam_start.x, beam_start.y, px, py) -
                    wall.size.x / 2; // account for wall thickness
            }

            // No intersection found
            return INFINITY;
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
            return sqrtl2(xd, yd);
        }

        static double sqrtl2(const double a, const double b)
        {
            return sqrt(a*a + b*b);
        }
    };

}
