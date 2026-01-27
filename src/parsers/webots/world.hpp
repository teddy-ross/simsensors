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
#include <simsensors/src/world.hpp>

namespace simsens {

    class WorldParser {

        public:

            static void parse(
                    const string world_file_name,
                    World & world,
                    const string robot_name="")
            {
                ifstream file(world_file_name);

                if (file.is_open()) {

                    string line;

                    Wall * _wall = nullptr;

                    bool in_robot = false;

                    world.y_inverted = true;

                    while (getline(file, line)) {

                        if (ParserUtils::string_contains(line, "Wall {")) {
                            _wall = new Wall();
                        }

                        if (_wall) {

                            parseWall(line, _wall);

                            if (endOfBlock(line)) {

                                world.walls.push_back(_wall);
                                _wall = nullptr;
                            }
                        }

                        if (robot_name.size() > 0 && 
                                ParserUtils::string_contains(line, robot_name)) {
                            in_robot = true;
                        }

                        if (in_robot) {

                            parseRobot(line, world);

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

        private:

            static bool endOfBlock(const string line) {

                return ParserUtils::string_contains(line, "}");
            }

            static void parseWall(const string line, Wall * wall)
            {
                ParserUtils::try_parse_vec3(line, "translation",
                        wall->translation);

                ParserUtils::try_parse_rotation(line, "rotation",
                        wall->rotation);

                ParserUtils::try_parse_vec3(line, "size",
                        wall->size);

                ParserUtils::try_parse_name(line, wall->name);
            }

            static void parseRobot(const string line, World & world)
            {
                vec3_t trans = {};
                if (ParserUtils::try_parse_vec3(line, "translation",
                            trans)) {
                    world.robotPose.x = trans.x;
                    world.robotPose.y = trans.y;
                    world.robotPose.z = trans.z;
                }

                rotation_t rot = {};
                if (ParserUtils::try_parse_rotation(line, "rotation",
                            rot)) {
                    vec3_t euler = {};
                    rotation_to_euler(rot, euler);
                    world.robotPose.phi = euler.x;
                    world.robotPose.theta = euler.y;
                    world.robotPose.psi = euler.z;
                }

            }

    };
}
