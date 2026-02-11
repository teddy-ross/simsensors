/* 
   Simple VRML parser for Webots .proto files

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

#include <simsensors/src/parsers/webots/utils.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/math.hpp>
#include <simsensors/src/robot.hpp>

namespace simsens {

    class RobotParser {

        public:

            static void parse(const string robot_file_name, Robot & robot)
            {
                ifstream file(robot_file_name);

                if (file.is_open()) {

                    string line;

                    Rangefinder * rangefinder = nullptr;

                    while (getline(file, line)) {

                        if (ParserUtils::string_contains(line, "RangeFinder {")) {
                            rangefinder = new Rangefinder();
                        }

                        if (rangefinder) {

                            ParserUtils::try_parse_double(line, "fieldOfView",
                                    rangefinder->field_of_view_radians);

                            ParserUtils::try_parse_int(line, "width",
                                    rangefinder->width);

                            ParserUtils::try_parse_int(line, "height",
                                    rangefinder->height);

                            ParserUtils::try_parse_double(
                                    line, "minRange",
                                    rangefinder->min_distance_m);
                            ParserUtils::try_parse_double(
                                    line, "maxRange", 
                                    rangefinder->max_distance_m);

                            ParserUtils::try_parse_vec3(line, "translation",
                                    rangefinder->translation);

                            ParserUtils::try_parse_rotation(line, "rotation",
                                        rangefinder->rotation);

                            ParserUtils::try_parse_name(line,
                                    rangefinder->name);

                            if (ParserUtils::string_contains(line, "}") ||
                                    ParserUtils::string_contains(line, "children")) {
                                robot.rangefinders.push_back(rangefinder);
                                rangefinder = nullptr;
                            }
                        }
                    }
                }

                else {
                    fprintf(stderr, "Unable to open file %s for input\n",
                            robot_file_name.c_str());
                }
            }
    };

}
