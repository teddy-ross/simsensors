#  Copyright (C) 2025 Simon D. Levy
 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http:--www.gnu.org/licenses/>.

CFLAGS = -O3 -std=c++11 -Wall -Wextra

HFDIR = $(HOME)/Desktop/hackflight/webots

all: main

run: main
	./main $(HFDIR)/worlds/maze.wbt $(HFDIR)/protos/DiyQuad.proto 

main: main.o
	g++ -o main main.o

main.o: main.cpp  *.h*
	g++ -c -I.. main.cpp

clean:
	rm -f main *.o

edit:
	vim main.cpp
