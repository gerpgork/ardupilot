#include "MotorDelay.hpp"
#include <math.h>
#include <stdio.h>
motor_delay::motor_delay(float update_hz, float latency_sec, float pwm_min){

	if(update_hz > 0){
		float update_dt 	= 1.0f/update_hz;
		iterations_to_hold 	= round(latency_sec / update_dt);
		//printf("holding for %d iterations\n",iterations_to_hold);
	}else{
		iterations_to_hold 	= 0;
	}

	min_val = pwm_min;

}

void motor_delay::reset_latency(float latency_sec){
	float update_dt 	= 1.0f/1200.0;
	iterations_to_hold 	= round(latency_sec / update_dt);
}

float motor_delay::update(float current_cmd){

	if(cmd_vec.size() < iterations_to_hold){
		//we haven't delayed enough - just hold - push zero out
		cmd_vec.push(current_cmd);
		return min_val;
	}else{
		//we have delayed enough
		cmd_vec.push(current_cmd); 	 //add to list
		float out = cmd_vec.front(); //grab oldest
		cmd_vec.pop(); 			 	 //delete oldest
		return out;
	}
}