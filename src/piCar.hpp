/*
 * piCar.hpp
 * 
 * PiCar control library C++ header file.
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

#ifndef PICAR_H
#define PICAR_H

/*  Camera directions  */

enum class Camera{ Right, Left, Up, Down };

/*  Driving directions  */

enum class Car{ Turn_Right_Forward, Turn_Right_Backward, Turn_Left_Forward, Turn_Left_Backward };

/* PiCar class */

class PiCar {
public:
	cv::Mat camera_input;
	raspicam::RaspiCam_Cv camera;
	
	PiCar(int power = 50, int servo_v = 0, int servo_h = 3, int trig = 6, int echo = 1);
	
	PiCar(const PiCar& other) = delete;
	
	PiCar& operator=(const PiCar& other) = delete;
	
	void set_camera_to_default_position(void);
	void turn_camera(enum Camera direction, float degree);
	
	int get_distance_from_obstacle(void) const;
	
	void stop(void);
	
	void go_forward(void);
	void go_backward(void);
	
	void turn_car(enum Car direction);
	
	void set_engine_power(int power);
	void increase_power_by(int increment);
	void decrease_power_by(int decrement);
	
private:
	int servo_v_pin;
	int servo_h_pin;
	int trig_pin;
	int echo_pin;
	int engine_power;
	
	void set_vertical_servo_to_position(int pos_v);
	void set_horizontal_servo_to_position(int pos_h);
};

#endif /* PICAR_H */
