all: piCar.hpp piCar.cpp main.cpp
	g++ -o picar "src/piCar.cpp" "src/main.cpp" -Wall -g -std=c++11 `pkg-config opencv --cflags` `pkg-config opencv --libs` -I/usr/local/include/ -lraspicam -lraspicam_cv -lwiringPi -lpthread

clean: picar
	rm picar
