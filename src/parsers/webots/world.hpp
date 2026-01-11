/* 
   Simple VRML parser for Webots .wbt world files

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

#include <simsensors/src/math.hpp>
#include <simsensors/src/parsers/webots/utils.hpp>
#include <simsensors/src/obstacles/wall.hpp>

namespace simsens {

    class WorldParser {

        public:

            vector<Wall *> walls;

            pose_t robotPose;

            void parse(
                    const string world_file_name,
                    const string robot_name="")
            {
                ifstream file(world_file_name);

                if (file.is_open()) {

                    string line;

                    Wall * _wall = nullptr;

                    bool in_robot = false;

                    while (getline(file, line)) {

                        if (ParserUtils::string_contains(line, "Wall {")) {
                            _wall = new Wall();
                        }

                        if (_wall) {

                            ParserUtils::try_parse_vec3(line, "translation",
                                    _wall->translation);
                            ParserUtils::try_parse_rotation(line, "rotation",
                                    _wall->rotation);
                            ParserUtils::try_parse_vec3(line, "size",
                                    _wall->size);
                            ParserUtils::try_parse_name(line, _wall->name);

                            if (endOfBlock(line)) {

                                walls.push_back(_wall);
                                _wall = nullptr;
                            }
                        }

                        if (robot_name.size() > 0 && 
                                ParserUtils::string_contains(line, robot_name)) {
                            in_robot = true;
                        }

                        if (in_robot) {

                            vec3_t trans = {};
                            if (ParserUtils::try_parse_vec3(line, "translation",
                                    trans)) {
                                robotPose.x = trans.x;
                                robotPose.y = trans.y;
                                robotPose.z = trans.z;
                            }

                            rotation_t rot = {};
                            if (ParserUtils::try_parse_rotation(line, "rotation",
                                    rot)) {
                                vec3_t euler = {};
                                rotation_to_euler(rot, euler);
                                robotPose.phi = euler.x;
                                robotPose.theta = euler.y;
                                robotPose.psi = euler.z;
                            }

                            if (endOfBlock(line)) {
                                in_robot = false;
                            }

                        }
                    }
                }

                else {
                    fprintf(stderr, "Unable to open file %s for input\n",
                            world_file_name.c_str());
                }
            }

            void report()
            {
                for (auto wall : walls) {
                    wall->dump();
                }
            }

        private:

            static bool endOfBlock(const string line) {

                return ParserUtils::string_contains(line, "}");
            }

    };
}
