/*
 * piCar.cpp
 * 
 * PiCar control library C++ source file.
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

#include "piCar.hpp"

/* Servos' default positions */

#define DEFAULT_VERTICAL_SERVO_POS 15
#define DEFAULT_HORIZONTAL_SERVO_POS 14

/* Servos' rotation limits */

#define VERTICAL_SERVO_MAX_POS_R 4
#define VERTICAL_SERVO_MAX_POS_L 23

#define HORIZONTAL_SERVO_MAX_POS_U 6
#define HORIZONTAL_SERVO_MAX_POS_D 19

/* Pwm pins (wiringPi) of right and left motors (from motor driver) */

#define MOTORS_R_PWM_PIN 26
#define MOTORS_L_PWM_PIN 25

/* Control pins (wiringPi) of right and left motors (from motor driver) */

#define MOTOR_R_F_PIN 22
#define MOTOR_R_B_PIN 23
#define MOTOR_L_F_PIN 29
#define MOTOR_L_B_PIN 28

/* 
 * initial_setup  
 * 
 * This function sets the necessary initial settings for PiCar.
 * 
 */

void PiCar::initial_setup(void){
	
	/* WiringPi setup */
	
	if (wiringPiSetup () == -1){
		std::cerr << "Error! Couldn't setup WiringPi." << std::endl;
		exit(1);
	}
	
	/* Servos' pin settings */
	
	pinMode(servo_h_pin, SOFT_PWM_OUTPUT);
	softPwmCreate(servo_h_pin, 0, 100);
	
	pinMode(servo_v_pin, SOFT_PWM_OUTPUT);
	softPwmCreate(servo_v_pin, 0, 100);
	
	/* Set camera to default position */
	
	set_camera_to_default_position();
	
	/* Ultrasonic distance sensor settings */
	
	pinMode(trig_pin, OUTPUT);
	pinMode(echo_pin, INPUT);
	pullUpDnControl(echo_pin, PUD_DOWN);
	digitalWrite(trig_pin, LOW);
	delay(2000);
	
	/* Camera settings */
	
	camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION_W);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION_H);
	if (!camera.open()){
		std::cerr << "Error! Couldn't open the Camera." << std::endl;
		exit(2);
	}
	
	/* Motors' pin settings */
	
	pinMode(MOTOR_R_F_PIN, OUTPUT);
	pinMode(MOTOR_R_B_PIN, OUTPUT);
	pinMode(MOTOR_L_F_PIN, OUTPUT);
	pinMode(MOTOR_L_B_PIN, OUTPUT);
	
	/* Do not move */
	
	stop();
	
	/* Motors' pwm pin settings */
	
	pinMode(MOTORS_R_PWM_PIN, SOFT_PWM_OUTPUT);
	softPwmCreate(MOTORS_R_PWM_PIN,0,100);
	
	pinMode(MOTORS_L_PWM_PIN, SOFT_PWM_OUTPUT);
	softPwmCreate(MOTORS_L_PWM_PIN,0,100);
	
	/* Set the entered pwm value for motors */
	
	set_engine_power(engine_power);
}

/* 
 * turn_camera  
 * 
 * This function turns PiCar's camera in the desired direction.
 * 
 * Parameters;
 * ->direction
 * 		RIGHT
 * 		LEFT
 * 		UP
 * 		DOWN
 * ->degree
 * 		degree value from default camera position
 * 
 */

void PiCar::turn_camera(int direction, float degree)
{
	if (direction == RIGHT){
		if (degree < 4.5){
			set_horizontal_servo_to_position(14);	/*	~0 degree	*/
		} else if (degree >= 4.5 && degree < 13.5){
			set_horizontal_servo_to_position(13);	/*	~9 degree	*/
		} else if (degree >= 13.5 && degree < 22.5){
			set_horizontal_servo_to_position(12);	/*	~18 degree	*/
		} else if (degree >= 22.5 && degree < 31.5){
			set_horizontal_servo_to_position(11);	/*	~27 degree	*/
		} else if (degree >= 31.5 && degree < 40.5){
			set_horizontal_servo_to_position(10);	/*	~36 degree	*/
		} else if (degree >= 40.5 && degree < 49.5){
			set_horizontal_servo_to_position(9);	/*	~45 degree	*/
		} else if (degree >= 49.5 && degree < 58.5){
			set_horizontal_servo_to_position(8);	/*	~54 degree	*/
		} else if (degree >= 58.5 && degree < 67.5){
			set_horizontal_servo_to_position(7);	/*	~63 degree	*/
		} else if (degree >= 67.5 && degree < 76.5){
			set_horizontal_servo_to_position(6);	/*	~72 degree	*/
		} else if (degree >= 76.5 && degree < 85.5){
			set_horizontal_servo_to_position(5);	/*	~81 degree	*/
		} else if (degree >= 85.5){
			set_horizontal_servo_to_position(4);	/*	~90 degree	*/
		}
	} else if (direction == LEFT) {
		if (degree < 5){
			set_horizontal_servo_to_position(14);	/*	~0 degree	*/
		} else if (degree >= 5 && degree < 15){
			set_horizontal_servo_to_position(15);	/*	~10 degree	*/
		} else if (degree >= 15 && degree < 25){
			set_horizontal_servo_to_position(16);	/*	~20 degree	*/
		} else if (degree >= 25 && degree < 35){
			set_horizontal_servo_to_position(17);	/*	~30 degree	*/
		} else if (degree >= 35 && degree < 45){
			set_horizontal_servo_to_position(18);	/*	~40 degree	*/
		} else if (degree >= 45 && degree < 55){
			set_horizontal_servo_to_position(19);	/*	~50 degree	*/
		} else if (degree >= 55 && degree < 65){
			set_horizontal_servo_to_position(20);	/*	~60 degree	*/
		} else if (degree >= 65 && degree < 75){
			set_horizontal_servo_to_position(21);	/*	~70 degree	*/
		} else if (degree >= 75 && degree < 85){
			set_horizontal_servo_to_position(22);	/*	~80 degree	*/
		} else if (degree >= 85){
			set_horizontal_servo_to_position(23);	/*	~90 degree	*/
		}
	} else if (direction == UP){
		if (degree < 5){
			set_vertical_servo_to_position(15);		/*	~0 degree	*/
		} else if (degree >= 5 && degree < 15){
			set_vertical_servo_to_position(14);		/*	~10 degree	*/
		} else if (degree >= 15 && degree < 25){
			set_vertical_servo_to_position(13);		/*	~20 degree	*/
		} else if (degree >= 25 && degree < 35){
			set_vertical_servo_to_position(12);		/*	~30 degree	*/
		} else if (degree >= 35 && degree < 45){
			set_vertical_servo_to_position(11);		/*	~40 degree	*/
		} else if (degree >= 45 && degree < 55){
			set_vertical_servo_to_position(10);		/*	~50 degree	*/
		} else if (degree >= 55 && degree < 65){
			set_vertical_servo_to_position(9);		/*	~60 degree	*/
		} else if (degree >= 65 && degree < 75){
			set_vertical_servo_to_position(8);		/*	~70 degree	*/
		} else if (degree >= 75 && degree < 85){
			set_vertical_servo_to_position(7);		/*	~80 degree	*/
		}  else if (degree >= 85){
			set_vertical_servo_to_position(6);		/*	~90 degree	*/
		}
	} else if (direction == DOWN){
		if (degree < 3.75){
			set_vertical_servo_to_position(15);		/*	~0 degree	*/
		} else if (degree >= 3.57 && degree < 11.25){
			set_vertical_servo_to_position(16);		/*	~7.5 degree	*/
		} else if (degree >= 11.25 && degree < 18.75){
			set_vertical_servo_to_position(17);		/*	~15 degree	*/
		} else if (degree >= 18.75 && degree < 22.25){
			set_vertical_servo_to_position(18);		/*	~22.5 degree*/
		}  else if (degree >= 22.25){
			set_vertical_servo_to_position(19);		/*	~30 degree	*/
		}
	} else {
		std::cout << "Invalid direction!" << std::endl;
	}
}

/* 
 * set_horizontal_servo_to_position  
 * 
 * This function turns horizontal servo to the desired position.
 * 
 * Parameters;
 * ->pos_h
 * 		pwm value of the position
 */
 
void PiCar::set_horizontal_servo_to_position(int pos_h)
{
	if (pos_h <= VERTICAL_SERVO_MAX_POS_R){
		pos_h = VERTICAL_SERVO_MAX_POS_R;
	}
	if (pos_h >= VERTICAL_SERVO_MAX_POS_L){
		pos_h = VERTICAL_SERVO_MAX_POS_L;
	}
	softPwmWrite(servo_h_pin, pos_h);
	delay(250);
	softPwmWrite(servo_h_pin, 0);
	delay(50);
}

/* 
 * set_vertical_servo_to_position  
 * 
 * This function turns vertical servo to the desired position.
 * 
 * Parameters;
 * ->pos_v
 * 		pwm value of the position
 */

void PiCar::set_vertical_servo_to_position(int pos_v)
{
	if (pos_v <= HORIZONTAL_SERVO_MAX_POS_U){
		pos_v = HORIZONTAL_SERVO_MAX_POS_U;
	}
	if (pos_v >= HORIZONTAL_SERVO_MAX_POS_D){
		pos_v = HORIZONTAL_SERVO_MAX_POS_D;
	}
	softPwmWrite(servo_v_pin, pos_v);
	delay(250);
	softPwmWrite(servo_v_pin, 0);
	delay(50);
}

/* 
 * set_camera_to_default_position  
 * 
 * This function turns the camera to the default position.
 * 
 */

void PiCar::set_camera_to_default_position(void)
{
	set_horizontal_servo_to_position(DEFAULT_HORIZONTAL_SERVO_POS);
	set_vertical_servo_to_position(DEFAULT_VERTICAL_SERVO_POS);
}

/* 
 * get_distance_from_obstacle  
 * 
 * This function returns the distance between obstacle 
 * which is in front of PiCar and PiCar in cantimeters 
 * using ultrasonic distance sensor.
 * 
 * To use HC-SR04 ultrasonic distance sensor;
 * -set trig pin to high along 10 microseconds
 * -measure the time interval between sending trigger signal 
 *  and receiving echo signal in microseconds.
 * -divide the measured time by 58 to find
 * 	distance in cantimeters
 * -and wait for 150 microseconds
 * 
 * Return value;
 * -> distance in cantimeters
 * 
 */

int PiCar::get_distance_from_obstacle(void)
{
	digitalWrite(trig_pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig_pin, LOW);
	while (digitalRead(echo_pin) != HIGH);
	auto start = std::chrono::steady_clock::now();
	while (digitalRead(echo_pin) != LOW);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> timeInterval = end - start;
	int distance = int((timeInterval.count() / 58) * 1000000);
	if (distance < 0){
		distance = -1;
	}
	delay(150);
	return distance;
}

/* 
 * stop  
 * 
 * This function stops PiCar.
 * 
 */

void PiCar::stop(void)
{
	digitalWrite(MOTOR_R_F_PIN, LOW);
	digitalWrite(MOTOR_R_B_PIN, LOW);
	digitalWrite(MOTOR_L_F_PIN, LOW);
	digitalWrite(MOTOR_L_B_PIN, LOW);
}

/* 
 * set_engine_power  
 * 
 * This function changes motors' pwm values to set the engine power.
 * 
 * Parameters;
 * ->power
 * 		pwm range (0 <= power <= 100)
 * 
 */

void PiCar::set_engine_power(int power)
{
	if (power >= 100){
		power = 100;
	}
	if (power <= 0){
		power = 0;
	}
	softPwmWrite(MOTORS_R_PWM_PIN, power);
	softPwmWrite(MOTORS_L_PWM_PIN, power);
}

/* 
 * increase_power_by  
 * 
 * This function changes motors' pwm values to increase the engine power.
 * 
 * Parameters;
 * ->increment
 * 		pwm increment value
 * 
 */

void PiCar::increase_power_by(int increment)
{
	engine_power += increment;
	set_engine_power(engine_power);
}

/* 
 * decrease_power_by  
 * 
 * This function changes motors' pwm values to decrease the engine power.
 * 
 * Parameters;
 * ->decrement
 * 		pwm decrement value
 * 
 */

void PiCar::decrease_power_by(int decrement)
{
	engine_power -= decrement;
	set_engine_power(engine_power);
}

/* 
 * go_forward  
 * 
 * This function drives PiCar forward.
 * 
 */

void PiCar::go_forward(void)
{
	digitalWrite(MOTOR_R_F_PIN, HIGH);
	digitalWrite(MOTOR_R_B_PIN, LOW);
	digitalWrite(MOTOR_L_F_PIN, HIGH);
	digitalWrite(MOTOR_L_B_PIN, LOW);
}

/* 
 * go_backward  
 * 
 * This function drives PiCar backward.
 * 
 */

void PiCar::go_backward(void)
{
	digitalWrite(MOTOR_R_F_PIN, LOW);
	digitalWrite(MOTOR_R_B_PIN, HIGH);
	digitalWrite(MOTOR_L_F_PIN, LOW);
	digitalWrite(MOTOR_L_B_PIN, HIGH);
}

/* 
 * turn_car  
 * 
 * This function turns PiCar.
 * 
 * Parameters;
 * ->direction
 * 		TURN_RIGHT_FORWARD
 * 		TURN_RIGHT_BACKWARD
 * 		TURN_LEFT_FORWARD
 * 		TURN_LEFT_BACKWARD
 * 
 * Pay attention!
 * PiCar continues its motion after calling turn_car 
 * function because the motors' pin value do not change
 * after calling turn_car function.
 * The turning movements are not made with the
 * rotating front wheels so, turning with degree
 * is not supported by PiCar control library for now.
 * 
 */

void PiCar::turn_car(int direction)
{
	if (direction == TURN_RIGHT_FORWARD){
		digitalWrite(MOTOR_R_F_PIN, LOW);
		digitalWrite(MOTOR_R_B_PIN, LOW);
		digitalWrite(MOTOR_L_F_PIN, HIGH);
		digitalWrite(MOTOR_L_B_PIN, LOW);
	} else if (direction == TURN_LEFT_FORWARD){
		digitalWrite(MOTOR_R_F_PIN, HIGH);
		digitalWrite(MOTOR_R_B_PIN, LOW);
		digitalWrite(MOTOR_L_F_PIN, LOW);
		digitalWrite(MOTOR_L_B_PIN, LOW);
	} else if (direction == TURN_RIGHT_BACKWARD){
		digitalWrite(MOTOR_R_F_PIN, LOW);
		digitalWrite(MOTOR_R_B_PIN, LOW);
		digitalWrite(MOTOR_L_F_PIN, LOW);
		digitalWrite(MOTOR_L_B_PIN, HIGH);
	} else if (direction == TURN_LEFT_BACKWARD){
		digitalWrite(MOTOR_R_F_PIN, LOW);
		digitalWrite(MOTOR_R_B_PIN, HIGH);
		digitalWrite(MOTOR_L_F_PIN, LOW);
		digitalWrite(MOTOR_L_B_PIN, LOW);
	} else {
		std::cout << "Invalid direction!" << std::endl;
	}
}