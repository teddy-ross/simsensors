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

#include <simsensors/src/math.hpp>

namespace simsens {

    class SimRangefinder {

        public:

            void read(const pose_t & robot_pose, const vector<Wall *> walls,
                    int * distances_mm,
                    FILE * dbg_logfp=nullptr,
                    vec3_t * dbg_intersection=nullptr)
            {
                // Get rangefinder rotation w.r.t. vehicle
                vec3_t rangefinder_angles= {};
                rotation_to_euler(rotation, rangefinder_angles);

                // Run a classic calculate-min loop to get distance to closest wall
                double dist = INFINITY;
                vec3_t intersection = {};
                for (auto wall : walls) {
                    const auto newdist = intersect_with_wall(
                                vec3_t{robot_pose.x, robot_pose.y, robot_pose.z},
                                robot_pose.psi + rangefinder_angles.z, // azimuth
                                robot_pose.theta + rangefinder_angles.y, // elevation
                                *wall,
                                &intersection);

                    if (dbg_intersection!=nullptr && newdist < dist) {
                        dbg_intersection->x = intersection.x;
                        dbg_intersection->y = intersection.y;
                        dbg_intersection->z = intersection.z;
                    }

                    dist = min(dist, newdist);
                }

                // Cut off distance at rangefinder's maximum
                if (dist > max_distance_m) {
                    dist = INFINITY;
                    if (dbg_intersection!=nullptr) {
                        dbg_intersection->z = -1;
                    }
                }

                // Subtract sensor offset from distance
                dist -= sqrt(
                        sqr(this->translation.x) +
                        sqr(this->translation.y) +
                        sqr(this->translation.z));

                // Use just one distance for now
                distances_mm[0] = dist == INFINITY ? -1 : dist * 1000;

                // Support logging
                if (dbg_logfp) {
                    fprintf(dbg_logfp, "%d\n", distances_mm[0]);
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

            friend class RangefinderVisualizer;
            friend class RobotParser;
    };
}
