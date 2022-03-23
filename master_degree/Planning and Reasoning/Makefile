PREFIX=/usr/local/
CC=g++
INCLUDE_DIRS=-I$(PREFIX)include/ompl-1.5 -I/usr/include/eigen3 -I/usr/include/opencv4

LIBS=-L$(PREFIX)lib -lompl -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lboost_program_options

BINS = HeuristicPlanning\

.phony: clean all

all:	$(BINS)

HeuristicPlanning:	HeuristicPlanning.cpp
	$(CC) $(INCLUDE_DIRS) -o $@ $^ $(LIBS)
	
clean:
	rm -rf $(BINS)
