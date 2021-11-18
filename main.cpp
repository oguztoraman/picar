/*
 * main.cpp
 *
 * picar control library C++ main file.
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

#include <mutex>
#include <chrono>
#include <thread>
#include <iostream>
//#include <softPwm.h>
//#include <wiringPi.h>
#include <opencv2/opencv.hpp>
//#include <raspicam/raspicam_cv.h>

#include <picar.hpp>

auto& car{raspberry_pi::picar::build()};

/* Your autonomous driving test code */
int main()
{
    return 0;
}
