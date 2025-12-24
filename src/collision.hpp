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

#include <ssmath.hpp>
#include <sstypes.h>
#include <obstacles/wall.hpp>

namespace simsens {

    class CollisionDetector {

        public:

            static bool detect(const vec3_t & robot_position, const vector<Wall *> walls)
            {
                // Beam starts at robot coordinates
                const simsens::vec2_t beam_start = {robot_position.x, robot_position.y};

                // Calculate beam endpoint
                const simsens::vec2_t beam_end = {
                    beam_start.x + MAX_WORLD_SIZE_M,
                    beam_start.y - MAX_WORLD_SIZE_M,
                };

                // Run a classic calculate-min loop to get distance to closest wall
                double dist = INFINITY;
                for (auto wall : walls) {
                    intersect_with_wall(beam_start, beam_end, robot_position.z,
                            *wall, dist);
                }

                return dist < COLLISION_TOLERANCE_M;
            }


        private:

            // Arbitrary limits
            static constexpr double MAX_WORLD_SIZE_M = 500;
            static constexpr double COLLISION_TOLERANCE_M = 0.05;

            static void intersect_with_wall(
                    const vec2_t beam_start_xy,
                    const vec2_t beam_end_xy,
                    const double robot_z,
                    const Wall & wall,
                    double & mindist)
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
                    const auto xydist = sqrt(dx*dx + dy*dy) - wall.size.x / 2;
                        
                    // If Z is below wall and XYZ distance is shorter than
                    // current, update current
                    if (robot_z < wall.size.z && xydist < mindist) {
                        mindist = xydist;
                    }
                }
            }
    };
}
