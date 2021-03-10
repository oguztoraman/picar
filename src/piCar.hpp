/*
 * piCar.hpp
 * 
 * PiCar control library C++ header file.
 * 
 * 
 */

#ifndef PICAR
#define PICAR

/*  Camera resolution  */

#define CAMERA_RESOLUTION_W 720
#define CAMERA_RESOLUTION_H 720

/*  Camera directions  */

#define RIGHT 	0
#define LEFT 	1
#define UP		2
#define DOWN	3

/*  Driving directions  */

#define TURN_RIGHT_FORWARD	0
#define TURN_RIGHT_BACKWARD	1
#define TURN_LEFT_FORWARD	2
#define TURN_LEFT_BACKWARD	3

class PiCar {
public:
	
	cv::Mat camera_input;
	raspicam::RaspiCam_Cv camera;
	
	PiCar(int servo_v, int servo_h, int trig, int echo, int power){
		servo_v_pin = servo_v;
		servo_h_pin = servo_h;
		trig_pin = trig;
		echo_pin = echo;
		engine_power = power;
		initial_setup();
	}
	
	void set_camera_to_default_position(void);
	void turn_camera(int direction, float degree);
	
	int get_distance_from_obstacle(void);
	
	void stop(void);
	
	void go_forward(void);
	void go_backward(void);
	
	void turn_car(int direction);
	
	void set_engine_power(int power);
	void increase_power_by(int increment);
	void decrease_power_by(int decrement);
	
private:
	int servo_v_pin;
	int servo_h_pin;
	int trig_pin;
	int echo_pin;
	int engine_power;
	
	void initial_setup(void);
	void set_vertical_servo_to_position(int pos_v);
	void set_horizontal_servo_to_position(int pos_h);
};

#endif /* PICAR */
