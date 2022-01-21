/*
 * picar.cpp
 *
 * picar control library C++ source file.
 *
 * Copyright (C) 2021-2022 Oğuz Toraman <oguz.toraman@protonmail.com>
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

#include  <picar.hpp>

#ifndef PC /* raspberry pi */
#include <softPwm.h>
#include <wiringPi.h>
#include <raspicam/raspicam_cv.h>
#endif /* board */

#include <chrono>
#include <sstream>
#include <opencv2/opencv.hpp>

namespace raspberry_pi {

/*
 * constructor
 *
 * This function sets the necessary initial settings for
 * picar with desired control pins and engine power.
 *
 */
picar::picar(int power, int servo_v, int servo_h, int trig, int echo)
    : engine_power{power}, vertical_servo_pin{servo_v},
      horizontal_servo_pin{servo_h}, trig_pin{trig}, echo_pin{echo}
{
#ifndef PC /* raspberry pi */
    if (wiringPiSetup () == -1){
        throw rpi_error{"PiCar failed to setup WiringPi!"};
    }

    pinMode(horizontal_servo_pin, SOFT_PWM_OUTPUT);
    softPwmCreate(horizontal_servo_pin, 0, 100);

    pinMode(vertical_servo_pin, SOFT_PWM_OUTPUT);
    softPwmCreate(vertical_servo_pin, 0, 100);

    set_camera_to_default_position();

    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
    pullUpDnControl(echo_pin, PUD_DOWN);
    digitalWrite(trig_pin, LOW);
    delay(2000);

    camera.set(CV_CAP_PROP_FRAME_WIDTH, camera_resolution_width);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, camera_resolution_height);
    if (!camera.open()){
        throw rpi_error{"PiCar failed to open the camera!"};
    }

    pinMode(right_motors_forward_pin, OUTPUT);
    pinMode(right_motors_backward_pin, OUTPUT);
    pinMode(left_motors_forward_pin, OUTPUT);
    pinMode(left_motors_backward_pin, OUTPUT);

    stop();

    pinMode(right_motors_pwm_pin, SOFT_PWM_OUTPUT);
    softPwmCreate(right_motors_pwm_pin, 0, 100);

    pinMode(left_motors_pwm_pin, SOFT_PWM_OUTPUT);
    softPwmCreate(left_motors_pwm_pin, 0, 100);
#endif /* board */
    set_engine_power(engine_power);
}

/*
 * build_PiCar
 *
 * This function implements Meyer’s Singleton.
 *
 * Default engine power;
 * -> POWER   = 50
 *
 * Default control pins(wiringPi);
 * -> SERVO_V = 0
 * -> SERVO_H = 3
 * -> TRIG    = 6
 * -> ECHO    = 1
 *
 */
picar& picar::build(int power, int servo_v, int servo_h, int trig, int echo)
{
    static picar car{power, servo_v, servo_h, trig, echo};
    return car;
}

/*
 * turn_camera
 *
 * This function turns PiCar's camera in the desired direction.
 *
 * Parameters;
 * ->direction
 * 		Camera::Right
 * 		Camera::Left
 * 		Camera::Up
 * 		Camera::Down
 * ->degree
 * 		degree value from default camera position
 *
 */
void picar::turn_camera(const Camera& direction, float degree)
{
    switch (direction) {
    case Camera::Right :
        set_horizontal_servo_to_position(find_pwm(degree, camera_angles_right));
        break;
    case Camera::Left :
        set_horizontal_servo_to_position(find_pwm(degree, camera_angles_left));
        break;
    case Camera::Up :
        set_vertical_servo_to_position(find_pwm(degree, camera_angles_up));
        break;
    case Camera::Down :
        set_vertical_servo_to_position(find_pwm(degree, camera_angles_down));
        break;
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
void picar::set_horizontal_servo_to_position(int pos_h)
{
    if (pos_h <= vertical_servo_max_pos_right){
        pos_h = vertical_servo_max_pos_right;
    }
    if (pos_h >= vertical_servo_max_pos_left){
        pos_h = vertical_servo_max_pos_left;
    }
#ifndef PC /* raspberry pi */
    softPwmWrite(horizontal_servo_pin, pos_h);
    delay(50);
    softPwmWrite(horizontal_servo_pin, 0);
    delay(50);
#endif /* board */
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
void picar::set_vertical_servo_to_position(int pos_v)
{
    if (pos_v <= horizontal_servo_max_pos_up){
        pos_v = horizontal_servo_max_pos_up;
    }
    if (pos_v >= horizontal_servo_max_pos_down){
        pos_v = horizontal_servo_max_pos_down;
    }
#ifndef PC /* raspberry pi */
    softPwmWrite(vertical_servo_pin, pos_v);
    delay(50);
    softPwmWrite(vertical_servo_pin, 0);
    delay(50);
#endif /* board */
}

/*
 * set_camera_to_default_position
 *
 * This function turns the camera to the default position.
 *
 */
void picar::set_camera_to_default_position()
{
    set_horizontal_servo_to_position(horizontal_servo_default_pos);
    set_vertical_servo_to_position(vertical_servo_default_pos);
}

/*
 * get_distance_from_obstacle
 *
 * This function returns the distance between obstacle
 * which is in front of PiCar and PiCar in cantimeters
 * using ultrasonic distance sensor.
 *
 * To use HC-SR04 ultrasonic distance sensor;
 * -set trig pin to high voltage along 10 microseconds
 * -measure the time interval of reading high voltage
 * at echo pin in microseconds.
 * -divide the measured time by 58 to find
 * 	distance in cantimeters
 * -and wait for 150 microseconds
 *
 * Return value;
 * -> distance in cantimeters
 *
 */
int picar::get_distance_from_obstacle() const
{
#ifndef PC /* raspberry pi */
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    while (digitalRead(echo_pin) != HIGH);
    auto start = std::chrono::steady_clock::now();
    while (digitalRead(echo_pin) != LOW);
    auto end = std::chrono::steady_clock::now();
    std::chrono::microseconds us{
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
    };
    int distance = static_cast<int>(us.count() / 58);
    delayMicroseconds(150);
    if (distance < 0){
        return -1;
    }
    return distance;
#else
    return 0;
#endif /* board */
}

/*
 * stop
 *
 * This function stops PiCar.
 *
 */
void picar::stop()
{
#ifndef PC /* raspberry pi */
    digitalWrite(right_motors_forward_pin, LOW);
    digitalWrite(right_motors_backward_pin, LOW);
    digitalWrite(left_motors_forward_pin, LOW);
    digitalWrite(left_motors_backward_pin, LOW);
#endif /* board */
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
void picar::set_engine_power(int power)
{
    if (power >= 100){
        power = 100;
    }
    if (power <= 0){
        power = 0;
    }
#ifndef PC /* raspberry pi */
    softPwmWrite(right_motors_pwm_pin, power);
    softPwmWrite(left_motors_pwm_pin, power);
#endif /* board */
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
void picar::increase_power_by(int increment)
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
void picar::decrease_power_by(int decrement)
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
void picar::go_forward()
{
#ifndef PC /* raspberry pi */
    digitalWrite(right_motors_forward_pin, HIGH);
    digitalWrite(right_motors_backward_pin, LOW);
    digitalWrite(left_motors_forward_pin, HIGH);
    digitalWrite(left_motors_backward_pin, LOW);
#endif /* board */
}

/*
 * go_backward
 *
 * This function drives PiCar backward.
 *
 */
void picar::go_backward()
{
#ifndef PC /* raspberry pi */
    digitalWrite(right_motors_forward_pin, LOW);
    digitalWrite(right_motors_backward_pin, HIGH);
    digitalWrite(left_motors_forward_pin, LOW);
    digitalWrite(left_motors_backward_pin, HIGH);
#endif /* board */
}

/*
 * turn_car
 *
 * This function turns PiCar.
 *
 * Parameters;
 * ->direction
 * 		Car::Turn_Right_Forward
 * 		Car::Turn_Right_Backward
 * 		Car::Turn_Left_Forward
 * 		Car::Turn_Left_Backward
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
void picar::turn_car(const Car& direction)
{
    switch (direction) {
    case Car::Turn_Right_Forward :
#ifndef PC /* raspberry pi */
        digitalWrite(right_motors_forward_pin, LOW);
        digitalWrite(right_motors_backward_pin, LOW);
        digitalWrite(left_motors_forward_pin, HIGH);
        digitalWrite(left_motors_backward_pin, LOW);
#endif /* board */
        break;
    case Car::Turn_Left_Forward :
#ifndef PC /* raspberry pi */
        digitalWrite(right_motors_forward_pin, HIGH);
        digitalWrite(right_motors_backward_pin, LOW);
        digitalWrite(left_motors_forward_pin, LOW);
        digitalWrite(left_motors_backward_pin, LOW);
#endif /* board */
        break;
    case Car::Turn_Right_Backward :
#ifndef PC /* raspberry pi */
        digitalWrite(right_motors_forward_pin, LOW);
        digitalWrite(right_motors_backward_pin, LOW);
        digitalWrite(left_motors_forward_pin, LOW);
        digitalWrite(left_motors_backward_pin, HIGH);
#endif /* board */
        break;
    case Car::Turn_Left_Backward :
#ifndef PC /* raspberry pi */
        digitalWrite(right_motors_forward_pin, LOW);
        digitalWrite(right_motors_backward_pin, HIGH);
        digitalWrite(left_motors_forward_pin, LOW);
        digitalWrite(left_motors_backward_pin, LOW);
#endif /* board */
        break;
    }
}

/*
 * inserter
 *
 * This function prints the specs of PiCar.
 *
 */
std::ostream& operator<<(std::ostream& os, const picar& car)
{
    std::ostringstream oss;
    oss << "-*-*-*-*-*-*-PiCar-*-*-*-*-*-*-\n";
    oss << "Engine power: " << car.engine_power << "\n";
    oss << "Camera position: ???" << "\n"; // will be implemented later
    oss << "Distance from obstacle: " << car.get_distance_from_obstacle() << "\n";
    return os << oss.str();
}

}  /* namespace raspberry_pi */
