'''
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

from simsensors.parsers.webots.utils import parse_vec, parse_string, try_parse


def parse(worldfile, robot_path=None):

    world = {'walls': []}

    with open(worldfile) as file:

        wall = None

        for line in file.read().split('\n'):

            if 'Wall {' in line:
                wall = {}

            if wall is not None:
                try_parse(wall, line, 'translation', parse_vec)
                try_parse(wall, line, 'size', parse_vec)
                try_parse(wall, line, 'rotation', parse_vec)
                try_parse(wall, line, 'name', parse_string)

            if '}' in line:
                if wall is not None:
                    if 'rotation' not in wall:
                        wall['rotation'] = (0, 0, 1, 0)
                    world['walls'].append(wall)
                wall = None

    return world
