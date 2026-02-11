/* 
   Simple robot representation for simulators

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

#include <map>

#include <simsensors/src/sensors/rangefinder.hpp>

namespace simsens {

    class Robot {

        public:

            std::map<char *, Rangefinder *> rangefinders;

            void dump()
            {
                for (auto rangefinder : rangefinders) {
                    rangefinder.second->dump();
                }
            }
    };

}
