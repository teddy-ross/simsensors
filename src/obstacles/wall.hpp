/* 
   Simple class for wall obstacles

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

#include <stdio.h>

#include <simsensors/src/types.h>

namespace simsens {

    class Wall {

        public:

            vec3_t translation;
            rotation_t rotation;
            vec3_t size;
            char name[100];

            Wall()
            {
                rotation.x = 0;
                rotation.y = 0;
                rotation.z = 1;
                rotation.alpha = 0;
            }

            void dump()
            {
                printf("Wall: \n");
                printf("  translation: x=%+3.3fm y=%+3.3fm z=%+3.3fm\n",
                        translation.x, translation.y, translation.z);
                printf("  rotation: x=%+3.3f y=%+3.3f z=%+3.3f alpha=%+3.3fr\n",
                        rotation.x, rotation.y, rotation.z, rotation.alpha);
                printf("  size: x=%3.3fm y=%3.3fm z=%3.3fm\n",
                        size.x, size.y, size.z);
                printf("\n");
            }
    };

}
