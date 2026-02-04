'''
Python classes parsing Webots .wbt world files

Copyright (C) 2026 Simon D. Levy

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

from simsensors.world import World
from simsensors.obstacles import Wall

def parse(worldfile, robot_path=None):

    world = World()

    with open(worldfile) as file:

        wall = None

        for line in file.read().split('\n'):

            if 'Wall {' in line:
                wall = Wall()

            if wall is not None:
                pass

            if '}' in line:
                wall = None

    return world

