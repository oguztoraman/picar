/*
 * main.cpp
 * 
 * PiCar control library C++ main file.
 * 
 * 
 */

#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include <thread>

#include "piCar.hpp"

/* Ultrasonic distance sensor trig and echo pins (wiringPi) */

#define TRIG 6
#define ECHO 1

/* Servos' pwm pins (wiringPi) */

#define SERVO_V 0
#define SERVO_H 3

/* Setup the car */

#define POWER 30

PiCar car(SERVO_V, SERVO_H, TRIG, ECHO, POWER);


int main(void)
{
  
	return 0;
}
