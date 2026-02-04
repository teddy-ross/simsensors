#!/usr/bin/python3
'''
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
'''

from simsensors.parsers.webots.world import parse

DATADIR = '../../../data/webots/'

WORLD = DATADIR + 'twoexit.wbt'
ROBOT = DATADIR + 'DiyQuad.proto'

world = parse(WORLD)

print(world)

'''
int main(int argc, char ** argv) 
{
    if (argc < 3) {
    }

    else {

        simsens::World world = {};
        simsens::WorldParser::parse(argv[1], world);
        world.report();

        simsens::Robot robot = {};
        simsens::RobotParser::parse(argv[2], robot);
        robot.report();
    }

    return 0;
}
'''
