/*
 * main.cpp
 * 
 * PiCar control library C++ main file.
 * 
 * Copyright (C) 2021 Oğuz Toraman <oguz.toraman@protonmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
