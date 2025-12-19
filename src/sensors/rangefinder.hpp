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

class SimRangefinder {

    friend class RangefinderVisualizer;
    friend class RobotParser;

    private:

        int width;
        int height; 
        int min_distance_mm;
        int max_distance_mm;
        double field_of_view_radians;

    public:

        void read(const vector<Wall *> walls, int * distances_mm)
        {
            (void)walls;
            (void)distances_mm;
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
