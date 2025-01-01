/* SPDX-FileCopyrightText: Copyright (c) 2022-2025 Oğuz Toraman <oguz.toraman@tutanota.com> */
/* SPDX-License-Identifier: GPL-3.0-or-later */

#include <picar.hpp>

#include <mutex>
#include <chrono>
#include <thread>
#include <iostream>
#ifndef PC
#include <softPwm.h>
#include <wiringPi.h>
#endif
#include <opencv2/opencv.hpp>
#ifndef PC
#include <raspicam/raspicam_cv.h>
#endif

auto& car{raspberry_pi::picar::build()};

/* Your autonomous driving test code */
int main()
{
    return 0;
}
