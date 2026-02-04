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
from simsensors.parsers.webots.utils import try_parse_vec

def _parse_wall(line, wall):

    translation = try_parse_vec(line, 'translation')
    if translation is not None:
        wall.translation = translation

    rotation = try_parse_vec(line, 'rotation')
    if rotation is not None:
        wall.rotation = rotation

    size = try_parse_vec(line, 'size')
    if size is not None:
        wall.size = size

def parse(worldfile, robot_path=None):

    world = World()

    with open(worldfile) as file:

        wall = None

        for line in file.read().split('\n'):

            if 'Wall {' in line:
                wall = Wall()

            if wall is not None:
                _parse_wall(line, wall)

            if '}' in line:
                if wall is not None:
                    print('wall: ', wall.translation, wall.rotation, wall.size)
                    world.walls.append(wall)
                wall = None

    return world

