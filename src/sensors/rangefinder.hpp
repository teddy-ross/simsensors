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

#include <ssmath.hpp>

namespace simsens {

    class SimRangefinder {

        public:

            void read(const pose_t & robot_pose, const vector<Wall *> walls,
                    int * distances_mm,
                    vec3_t * dbg_intersection=nullptr, FILE * logfp=nullptr)
            {
                // Get rangefinder rotation w.r.t. vehicle
                vec3_t rangefinder_angles= {};
                rotation_to_euler(rotation, rangefinder_angles);

                // Use vehicle angles and rangefinder angle to get rangefinder
                // azimuth and elevation angles
                const auto azimuth_angle = robot_pose.psi + rangefinder_angles.z;
                const auto elevation_angle = robot_pose.theta + rangefinder_angles.y;

                // Beam starts at robot coordinates
                const vec2_t beam_start = {robot_pose.x, robot_pose.y};

                // Calculate beam endpoint
                const vec2_t beam_end = {
                    beam_start.x + cos(azimuth_angle) * max_distance_m,
                    beam_start.y - sin(azimuth_angle) * max_distance_m,
                };

                // Run a classic calculate-min loop to get distance to closest wall
                double dist = INFINITY;
                vec3_t intersection = {};
                for (auto wall : walls) {
                    intersect_with_wall(beam_start, beam_end, robot_pose.z,
                            elevation_angle, *wall, dist, intersection);
                }

                // Cut off distance at rangefinder's maximum
                if (dist > max_distance_m) {
                    dist = INFINITY;
                    intersection.z = -1;
                }

                // Subtract sensor offset from distance
                dist -= sqrt(
                        sqr(this->translation.x) +
                        sqr(this->translation.y) +
                        sqr(this->translation.z));

                // Use just one distance for now
                distances_mm[0] = dist == INFINITY ? -1 : dist * 1000;

                // Support debugging
                if (logfp && robot_pose.z > 0.18) {
                    fprintf(logfp, "%d\n", distances_mm[0]);
                }
                if (dbg_intersection) {
                    dbg_intersection->x = intersection.x;
                    dbg_intersection->y = intersection.y;
                    dbg_intersection->z = intersection.z;
                }

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

            int width;
            int height; 
            double min_distance_m;
            double max_distance_m;
            double field_of_view_radians;
            vec3_t translation;
            rotation_t rotation;

            static void intersect_with_wall(
                    const vec2_t beam_start_xy,
                    const vec2_t beam_end_xy,
                    const double robot_z,
                    const double elevation_angle,
                    const Wall & wall,
                    double & mindist,
                    vec3_t & intersection)
            {
                // Get wall endpoints
                const auto psi = wall.rotation.alpha; // rot.  always 0 0 1 alpha
                const auto len = wall.size.y / 2;
                const auto wall_dx = len * sin(psi);
                const auto wall_dy = len * cos(psi);
                const auto wall_tx = wall.translation.x;
                const auto wall_ty = wall.translation.y;

                // If beam ((x1,y1),(x2,y2)) intersects with with wall
                // ((x3,y3),(x4,y4)) 
                double px=0, py=0;
                if (line_segments_intersect(
                            beam_start_xy.x, beam_start_xy.y,
                            beam_end_xy.x, beam_end_xy.y,
                            wall_tx + wall_dx, wall_ty + wall_dy,
                            wall_tx - wall_dx, wall_ty - wall_dy,
                            px, py)) {

                    // Use intersection (px,py) to calculate XY distance to wall
                    const auto dx = beam_start_xy.x - px;
                    const auto dy = beam_start_xy.y - py;
                    const auto xydist = sqrt(dx*dx + dy*dy);

                    // Use XY distance, robot Z, and elevation angle to calculate Z
                    // offset of intersection on wall w.r.t. robot Z
                    const auto dz = -tan(elevation_angle) * xydist;

                    // Calculate XYZ distance by including Z offset and wall
                    // thickness
                    const auto xyzdist = sqrt(dx*dx + dy*dy + dz*dz)
                        - wall.size.x / 2;

                    // Calculate Z in world coordinates
                    const auto pz = robot_z + dz;

                    // If Z is below wall and XYZ distance is shorter than
                    // current, update current
                    if (pz < wall.size.z && xyzdist < mindist) {
                        intersection.x = px;
                        intersection.y = py;
                        intersection.z = pz;
                        mindist = xyzdist;
                    }
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

            static double sqr(const double x)
            {
                return x * x;
            }

            friend class RangefinderVisualizer;
            friend class RobotParser;

    };
}
