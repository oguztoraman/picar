all: inc/piCar.hpp src/piCar.cpp src/main.cpp
	g++ -o picar "src/piCar.cpp" "src/main.cpp" -Wall -g -std=c++17 `pkg-config opencv --cflags` `pkg-config opencv --libs` -I/usr/local/include/ -lraspicam -lraspicam_cv -lwiringPi -lpthread

clean: picar
	rm picar
