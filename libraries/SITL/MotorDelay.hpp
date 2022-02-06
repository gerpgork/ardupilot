#include <queue>

class motor_delay {

	public:
		motor_delay(float update_hz, float latency_sec, float pwm_min);

		float update(float current_cmd); //take in the current command - output the latent command
		void reset_latency(float latency_sec);
	private:
		std::queue<float> cmd_vec;
		unsigned int iterations_to_hold = 0;
		float min_val = 0; 

};