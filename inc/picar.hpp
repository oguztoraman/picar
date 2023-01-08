/*
 * picar.hpp
 *
 * picar control library C++ header file.
 *
 * Copyright (C) 2021-2023 Oğuz Toraman <oguz.toraman@protonmail.com>
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

#define PC

#include <cmath>
#include <iosfwd>
#include <string>
#include <exception>

namespace raspberry_pi {

class picar {
public:
    static_assert (__cplusplus >= 202002L, "c++20 required!");

#ifndef PC /* raspberry pi */
    cv::Mat camera_input;
    raspicam::RaspiCam_Cv camera;
#endif /* board */

    enum class Camera{
        Right, Left, Up, Down
    };

    enum class Car{
        Turn_Right_Forward,
        Turn_Right_Backward,
        Turn_Left_Forward,
        Turn_Left_Backward
    };

    [[nodiscard]]
    static picar& build(
            int power   = 50,
            int servo_v = 0,
            int servo_h = 3,
            int trig    = 6,
            int echo    = 1
    );

    picar(const picar&) = delete;
    picar& operator=(const picar&) = delete;

    void set_camera_to_default_position();
    void turn_camera(const Camera& direction, float degree);

    [[nodiscard]]
    int get_distance_from_obstacle() const;

    void stop();

    void go_forward();
    void go_backward();

    void turn_car(const Car& direction);

    void set_engine_power(int power);
    void increase_power_by(int increment);
    void decrease_power_by(int decrement);

    friend std::ostream& operator<<(std::ostream& os, const picar& car);

private:
    int engine_power;
    int vertical_servo_pin;
    int horizontal_servo_pin;
    int trig_pin;
    int echo_pin;

    void set_vertical_servo_to_position(int pos_v);
    void set_horizontal_servo_to_position(int pos_h);

    picar(int power, int servo_v, int servo_h, int trig, int echo);

    static constexpr int right_motors_pwm_pin = 26;
    static constexpr int left_motors_pwm_pin  = 25;

    static constexpr int right_motors_forward_pin  = 22;
    static constexpr int right_motors_backward_pin = 23;
    static constexpr int left_motors_forward_pin   = 29;
    static constexpr int left_motors_backward_pin  = 28;

    static constexpr int camera_resolution_width  = 720;
    static constexpr int camera_resolution_height = 720;

    static constexpr int vertical_servo_default_pos   = 15;
    static constexpr int horizontal_servo_default_pos = 14;

    static constexpr int vertical_servo_max_pos_right  = 4;
    static constexpr int vertical_servo_max_pos_left   = 23;
    static constexpr int horizontal_servo_max_pos_up   = 6;
    static constexpr int horizontal_servo_max_pos_down = 19;

    static constexpr int camera_angles_right[][11]{
        { 5, 14, 23, 32, 41, 50, 59, 68, 77, 86, 90},
        {14, 13, 12, 11, 10,  9,  8,  7,  6,  5, 4}
    };

    static constexpr int camera_angles_left[][10]{
        { 5, 15, 25, 35, 45, 55, 65, 75, 85, 90},
        {14, 15, 16, 17, 18, 19, 20, 21, 22, 23}
    };

    static constexpr int camera_angles_up[][10]{
        { 5, 15, 25, 35, 45, 55, 65, 75, 85, 90},
        {15, 14, 13, 12, 11,  9,  8,  7,  6, 5}
    };

    static constexpr int camera_angles_down[][5]{
        { 4, 11, 19, 23, 90},
        {15, 16, 17, 18, 19}
    };

    template <int N>
    [[nodiscard]]
    int find_pwm(float angle, const int(&angle_pwm_table)[2][N]) const
    {
        if (angle <= angle_pwm_table[0][0]){
            return angle_pwm_table[1][0];
        }
        if (angle >= angle_pwm_table[0][N - 1]){
            return angle_pwm_table[1][N - 1];
        }
        int i{N/2};
        for (int exp{2}, step{}; i >= 1 && i < N; ++exp){
            step = N/std::pow(2, exp) ? N/std::pow(2, exp) : 1;
            if (angle > angle_pwm_table[0][i - 1] &&
                angle <= angle_pwm_table[0][i]){
                    break;
            }
            angle > angle_pwm_table[0][i] ? i += step : i -= step;
        }
        return angle_pwm_table[1][i];
    }
};

class rpi_error : public std::exception {
public:
    rpi_error() = default;
    rpi_error(std::string error) : m_error{std::move(error)} { }

    [[nodiscard]]
    const char* what() const noexcept override
    {
        return m_error.c_str();
    }
private:
    std::string m_error{"An error has occurred!"};
};

}  /* namespace raspberry_pi */

#endif /* PICAR_HPP */
