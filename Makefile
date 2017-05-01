## List of all cpp files - deal with later
CPP_FILES = $(wildcard src/*.cpp)
# CPP_FILES += $(wildcard common_vid_exec/*.cpp)

# Makeflag variable
OPTIONS=$(-g)

## Include directories
INCL_DIRS = -I./include/ -I./common_vid_exec/

## Executable directories
EXEC_DIRS = ./bin/

## Optimisation flag
SPEC_FLAG=-O2

## From the list of cpp files, generate a list of .o files
OBJ_FILES=$(CPP_FILES:.cpp=.o)

## get the libs together
# Not included: -lwiringPi
LIBS = -ljpeg `pkg-config opencv --cflags --libs` -ldl -lstdc++

all: run

run: $(OBJ_FILES) main.cpp
	g++ $(OPTIONS) $(SPEC_FLAG) -o $(EXEC_DIRS)main main.cpp $(INCL_DIRS) $(OBJ_FILES) --std=c++11 -g $(LIBS) -pthread

min: main.cpp
	g++ $(OPTIONS) $(SPEC_FLAG) -o $(EXEC_DIRS)main main.cpp $(INCL_DIRS) --std=c++11 -g $(LIBS) -pthread

src/%.o: src/%.cpp
	g++ $(OPTIONS) $(SPEC_FLAG) -c -o $@ $< $(INCL_DIRS) --std=c++11 -pthread

common_vid_exec/%.o: common_vid_exec/%.cpp
	g++ $(OPTIONS) $(SPEC_FLAG) -c -o $@ $< $(INCL_DIRS) --std=c++11 -pthread


clean:
	rm -rf $(OBJ_FILES) $(EXEC_DIRS)run.exe
