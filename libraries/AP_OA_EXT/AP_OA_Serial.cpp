#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_OA_Serial.h"
#include <GCS_MAVLink/GCS.h> 					/*to send data to GCS */
#include <AP_Logger/AP_Logger.h>

#if 1
#include <stdio.h> //for sim TBD
#endif

extern const AP_HAL::HAL& hal;

AP_OA_Serial::AP_OA_Serial(optimAero::optimAero_State &_state,
				AP_OA_Params &_params,
				uint8_t serial_instance) :
	AP_OA_Backend(_state,_params)
{

	const AP_SerialManager &serial_manager = AP::serialmanager();
	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance);

	if(uart != nullptr){
		uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_optimAero, serial_instance));
	} //tbd might want this a param

	parse_params_.bytesInBuffer = 0; //make sure it knows its zero
	
	/*updates these if we have recvd them from arduino*/
	battery_updated_ 		= false;

	/*counters to monitor healthy signals*/
	battery_health_cntr_ 	= 0;
	state.batt_health 		= false;
	state.status  			= optimAero::optimAero_SerialBad;
	quad_msg_cnt 			= 0;
	log_cnt 				= 0;
	printf("SERIAL OA\n");
}

/*detect if connected*/

bool AP_OA_Serial::detect(uint8_t serial_instance)
{
	return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_optimAero, serial_instance) != nullptr;
}

void AP_OA_Serial::update400hz(void){
	//do nothing
}

void AP_OA_Serial::update100hz(void)
{
	//uart->write("Hello\n");
	checkForData();
}

void AP_OA_Serial::update50hz(void){

}

void AP_OA_Serial::update10hz(void){
	
}

void AP_OA_Serial::update3hz(void){

}

void AP_OA_Serial::update1hz(void){
	if(params.function | MASK_ARDUINO_BATT){
		updateBatteryHealth(); //determines health -> also will bitch about no data hopefully
	}
}

void AP_OA_Serial::populateBuffer(OA_Parser &parser_in){
	/*sim function only*/

	if(parser_in.bytesInBuffer > 0 && parse_params_.bytesInBuffer < OA_PARSER_MAX_BUFFER_SIZE)
	{
		// populate a buffer if data avail and buffer container size is not full

		for(int i=0; i<parser_in.bytesInBuffer; i++){
			parse_params_.datalinkBuffer[parse_params_.bytesInBuffer] = parser_in.datalinkBuffer[i];
			#if 0
				printf("byte[%d] = %02X \n",parse_params_.bytesInBuffer,parse_params_.datalinkBuffer[parse_params_.bytesInBuffer]);
			#endif
			parse_params_.bytesInBuffer++;
		}
		
		memset (parser_in.datalinkBuffer,0,parser_in.bytesInBuffer);
		parser_in.bytesInBuffer = 0;
	}
}

void AP_OA_Serial::LogESC(){

	AP_Logger *logger = AP_Logger::get_singleton();

    if (logger == nullptr) {
        return;
    } 
	
	for(int i=0; i<6; i++){
		logger->Write_ESC(  i,
									AP_HAL::micros64(),
									(int32_t)(state.rpm[i]),      //   rpm is eRPM (rpm * 100)
									(uint16_t)(state.volt[i]),     //   voltage is in centi-volts
									(uint16_t)(state.curr[i]),     //   current is in centi-amps
									(int16_t)(state.mos_tmp[i]),  	  //   temperature is in centi-degrees Celsius
									(int16_t)(state.cap_tmp[i]),
									(uint16_t)(input_thr[i]),
									(uint16_t)(output_thr[i]) );
	}
}

void AP_OA_Serial::checkForData(void){

	if(nullptr != uart ){
		// uart setup good to go
		while(uart->available() > 0 && parse_params_.bytesInBuffer < OA_PARSER_MAX_BUFFER_SIZE)
		{
			// populate a buffer if data avail and buffer container size is not full
			parse_params_.datalinkBuffer[parse_params_.bytesInBuffer] = uart->read();
			// hal.uartE->printf("byte[%d] = %02X ,",parse_params_.bytesInBuffer,parse_params_.datalinkBuffer[parse_params_.bytesInBuffer]);
			parse_params_.bytesInBuffer++;
		}

		int iData       = 0;
    	int dataDone    = 0;

		while( (iData <= parse_params_.bytesInBuffer - OPTIM_MESSAGE_HDR_SIZE) && !dataDone )
		{
            if(parse_params_.datalinkBuffer[iData] == 0x9b &&
					parse_params_.datalinkBuffer[iData+1] == 0x8b &&
					parse_params_.datalinkBuffer[iData+2] == 0x7b &&
					parse_params_.datalinkBuffer[iData+3] == 0x6b)
			{ // have we read a motor?
				parse_params_.bf  = &(parse_params_.datalinkBuffer[iData]);

				//we could have esc pkt
				if( (iData+(int)sizeof(HEX_OUT)) <= parse_params_.bytesInBuffer) 
				{   //have we read enough - if we got sizeof(t_motorESC)  we have full set
					memcpy(&v2_in, parse_params_.bf, sizeof(HEX_OUT) );
					iData += sizeof(HEX_OUT);
					msgCnt++; //for debug
					quad_msg_cnt++; //for logging
					//hal.uartE->printf("%d\n",msgCnt);
					//gcs().send_text(MAV_SEVERITY_NOTICE,"%d\n",msgCnt);

					float batt_volt = 0.0f;
					float batt_curr = 0.0f;
					unsigned char batt_cnt = 0;
					for(int i=0; i<6; i++)
					{
						state.curr[i] 			= v2_in.current[i] * params.curr_sf;
						state.volt[i] 			= v2_in.voltage[i];
						state.mos_tmp[i] 		= v2_in.mos_tmp[i];
						state.cap_tmp[i] 		= v2_in.cap_tmp[i];
						state.mot_msg_num[i] 	= v2_in.msg_num[i];
						state.rpm[i] 			= v2_in.rpm[i]*params.rpm_sf;


						input_thr[i] 			= v2_in.input_thr[i];
						output_thr[i] 			= v2_in.output_thr[i];
						if(state.volt[i] > 0 && state.volt[i] < 6500)
						{
							batt_volt += (state.volt[i])/10.0f; //convert mV to V
							batt_curr += (state.curr[i]);
							batt_cnt++;
						}
					}
					if(batt_cnt > 0){
						state.batt_voltage 		= batt_volt/batt_cnt;
						state.batt_current		= batt_curr;
					}else{
						state.batt_voltage 		= 0;
						state.batt_current 		= 0;
					}

					state.batt_health		= true;
					state.status  			= optimAero::optimAero_SerialGood;
            		state.last_reading_ms   = AP_HAL::millis();
					LogESC();

					if(state.batt_voltage > 65 || state.batt_voltage < 1){ 
						//encase we get garbage 
						state.batt_health = false;
						debug_mode = true;
						//hal.uartE->printf("bad V = %f\n",state.batt_voltage);
						//gcs().send_text(MAV_SEVERITY_NOTICE,"bad V = %f\n",state.batt_voltage);

					}

					battery_updated_ 		= true;

					#if 0
						if( (msgCnt%25) == 0)
						{
							for(int i=0; i<4; i++)
							{
								//hal.uartE->printf("V[%d] = %d\n",i,state.hex_.voltage[i]);
								gcs().send_text(MAV_SEVERITY_NOTICE,"V[%d] = %d\n",i,state.hex_.voltage[i]);

							}
						}
					#endif

					if(debug_mode && state.batt_health){
						gcs().send_text(MAV_SEVERITY_NOTICE, "GOOD T-Motor, V=%f",state.batt_voltage);
						debug_mode = false;
					}

				}else{
					dataDone = 1;
				} 				
			} 
			else{
				// buffer(idata) != sync
				iData++;
			}

		}//end of while parse loop

	if(iData > 0){ 
		// only need to shift buffer if we read anything
		parse_params_.bytesInBuffer = parse_params_.bytesInBuffer - iData;
		memmove( parse_params_.datalinkBuffer, &(parse_params_.datalinkBuffer[iData]), parse_params_.bytesInBuffer );
		//printf("memmove%d\n",iData);

	}
	

	}//end of nullptr = uart

}


void AP_OA_Serial::updateBatteryHealth(void){

	
	/////////////////////////////////////////////////////////////
	///////////// CHECK FOR UPDATE
	if(battery_updated_){
		battery_health_cntr_ 	= 0;
		battery_updated_ 		= false;
	}else{
		battery_health_cntr_++;
	}

	if(battery_health_cntr_ > 5){
		state.batt_health = false;
	}
	///////////////////////////////////////////////////////////////

}