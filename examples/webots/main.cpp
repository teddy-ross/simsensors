/* 
   Webots world-parsing tester

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

#include <stdio.h>

#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/world.hpp>

int main(int argc, char ** argv) 
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s WORLDFILE ROBOTFILE\n", argv[0]);
    }

    else {

        simsens::World world = {};

        const std::string worldname =  argv[1];
        static simsens::WorldParser _worldParser;
        _worldParser.parse(worldname, world);
        _worldParser.report();

        const std::string robot =  argv[2];
        static simsens::RobotParser _robotParser;
        _robotParser.parse(robot);
        _robotParser.report();
    }

    return 0;
}
