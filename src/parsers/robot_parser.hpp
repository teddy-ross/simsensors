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

#include <parsers/utils.hpp>
#include <sensors/rangefinder.hpp>

class RobotParser {

    public:

        vector<SimRangefinder *> rangefinders;

        void parse(const string robot_file_name)
        {
            ifstream file(robot_file_name);

            if (file.is_open()) {

                string line;

                SimRangefinder * _rangefinder = nullptr;

                while (getline(file, line)) {

                    if (ParserUtils::string_contains(line, "RangeFinder {")) {
                        _rangefinder = new SimRangefinder();
                    }

                    if (_rangefinder) {

                        double tmp = 0;

                        ParserUtils::try_parse_double(line, "fieldOfView",
                                _rangefinder->field_of_view_radians);

                        ParserUtils::try_parse_int(line, "width",
                                _rangefinder->width);

                        ParserUtils::try_parse_int(line, "height",
                                _rangefinder->height);

                        if (ParserUtils::try_parse_double(
                                    line, "minRange", tmp)) {
                            _rangefinder->min_distance_mm = 1000 * tmp;
                        }

                        if (ParserUtils::try_parse_double(
                                    line, "maxRange", tmp)) {
                            _rangefinder->max_distance_mm = 1000 * tmp;
                        }

                        if (ParserUtils::string_contains(line, "}")) {
                            rangefinders.push_back(_rangefinder);
                            _rangefinder = nullptr;
                        }
                    }
                }
            }

            else {
                fprintf(stderr, "Unable to open file %s for input\n",
                       robot_file_name.c_str());
            }
        }

        void report()
        {
            for (auto _rangefinder : rangefinders) {
                _rangefinder->dump();
            }
        }

    private:

};

