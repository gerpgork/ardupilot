
#pragma once

#include "optimAero.h"
#include "AP_OA_Backend.h"
#include "OA_Parser.h"
#include "OA_MessageTypes.h"
#include <GCS_MAVLink/GCS_MAVLink.h> 	// to be able to send data to GCS
#include <AP_Logger/AP_Logger.h>

class AP_OA_Serial : public AP_OA_Backend
{

public:
	//const
	AP_OA_Serial(optimAero::optimAero_State &_state,
			AP_OA_Params &_params,
			uint8_t serial_instance);

	//static detection
	static bool detect(uint8_t serial_instance);

	//update state
	void update400hz(void) override;
	void update100hz(void) override;
	void update50hz(void) override;
	void update10hz(void) override;
    void update3hz(void) override;	
	void update1hz(void) override;
	void populateBuffer(OA_Parser &parser_in) override; //sim only use

	void checkForData(void);	/*function to read serial data and update structures*/

	void updateBatteryHealth(void);
	void LogESC();

 protected:

private:
	AP_HAL::UARTDriver *uart 		= nullptr;

	OA_Parser						parse_params_;
	struct MsgBattCell_ref 			battery_in_;

	struct HEX_OUT 					v2_in;
	struct tach_rpm 				rpm_in;
	bool 							battery_updated_;

	unsigned char 					battery_health_cntr_;
	unsigned char 					msgCnt=0; //for tach
	unsigned char 					log_cnt 	 = 0;
	unsigned char 					quad_msg_cnt = 0 ;
	bool 							debug_mode = true;
	unsigned short input_thr[6] = {0};
	unsigned short output_thr[6] = {0};
};
