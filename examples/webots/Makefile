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

ROOTDIR = ../../..

SRCDIR = $(ROOTDIR)/simsensors/src

HFDIR = $(HOME)/Desktop/hackflight/webots

EXE = main

all: $(EXE)

run: $(EXE)
	./$(EXE) $(HFDIR)/worlds/maze.wbt $(HFDIR)/protos/DiyQuad.proto 

$(EXE): $(EXE).o
	g++ -o $(EXE) $(EXE).o

$(EXE).o: $(EXE).cpp  $(SRCDIR)/*/*.* $(SRCDIR)/parsers/webots/*.*
	g++ -c -I$(ROOTDIR) $(EXE).cpp

clean:
	rm -f $(EXE) *.o

edit:
	vim $(EXE).cpp
