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

#ifndef PICAR_HPP
#define PICAR_HPP

#include <iosfwd>
#include <string>
#include <exception>


namespace RaspberryPi {

/*  Camera directions  */
enum class Camera{ Right, Left, Up, Down };

/*  Driving directions  */
enum class Car{ Turn_Right_Forward, Turn_Right_Backward, Turn_Left_Forward, Turn_Left_Backward };

/* PiCar class */
class PiCar {
public:
	cv::Mat camera_input;
	raspicam::RaspiCam_Cv camera;
	
	static PiCar& build_PiCar(int power = 50, int servo_v = 0, int servo_h = 3, int trig = 6, int echo = 1);
	
	PiCar(const PiCar&) = delete;
	PiCar& operator=(const PiCar&) = delete;
	
	void set_camera_to_default_position();
	void turn_camera(const Camera& direction, float degree);
	
	int get_distance_from_obstacle() const;
	
	void stop();
	
	void go_forward();
	void go_backward();
	
	void turn_car(const Car& direction);
	
	void set_engine_power(int power);
	void increase_power_by(int increment);
	void decrease_power_by(int decrement);
	
	friend std::ostream& operator<<(std::ostream& os, const PiCar& picar);
	
private:
	int engine_power;
	int vertical_servo_pin;
	int horizontal_servo_pin;
	int trig_pin;
	int echo_pin;
	
	void set_vertical_servo_to_position(int pos_v);
	void set_horizontal_servo_to_position(int pos_h);
	
	PiCar(int power, int servo_v, int servo_h, int trig, int echo);
	
	/* Pwm pins (wiringPi) of right and left motors (from motor driver) */
	static constexpr int right_motors_pwm_pin = 26;
	static constexpr int left_motors_pwm_pin  = 25;
	
	/* Control pins (wiringPi) of right and left motors (from motor driver) */
	static constexpr int right_motors_forward_pin  = 22;
	static constexpr int right_motors_backward_pin = 23;
	static constexpr int left_motors_forward_pin   = 29;
	static constexpr int left_motors_backward_pin  = 28;
	
	/* Camera resolution */
	static constexpr int camera_resolution_width  = 720;
	static constexpr int camera_resolution_height = 720;
	
	/* Servos' default positions */
	static constexpr int vertical_servo_default_pos   = 15;
	static constexpr int horizontal_servo_default_pos = 14;
	
	/* Servos' rotation limits */
	static constexpr int vertical_servo_max_pos_right  = 4;
	static constexpr int vertical_servo_max_pos_left   = 23;
	static constexpr int horizontal_servo_max_pos_up   = 6;
	static constexpr int horizontal_servo_max_pos_down = 19;
};

class PiCar_exception : public std::exception {
public:
    PiCar_exception() = default;
    PiCar_exception(std::string error) : m_error{std::move(error)} {}

    const char* what() const noexcept override
    {
        return m_error.c_str();
    }
private:
    std::string m_error{"An error has occurred!"};
};
	
}  /* RaspberryPi namespace */

#endif /* PICAR_HPP */
