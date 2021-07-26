/*external library for comms with ardupilot dev by optimaero*/

#include "optimAero.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h> 					/*to send data to GCS */


//supported features
#include "AP_OA_Serial.h"


extern const AP_HAL::HAL &hal;

//table of user set params
const AP_Param::GroupInfo optimAero::var_info[] = {

	// @Group: 1_
	// @Path: AP_OA_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, optimAero, AP_OA_Params),
	/*TBD figure out what SUBGROUPINFO takes in that 25 is BS*/
	AP_SUBGROUPVARPTR(drivers[0], "1_", 57, optimAero, backend_var_info[0]),

#if OA_MAX_INSTANCES > 1
	// @Group: 1_
	// @Path: AP_OA_Params.cpp
	AP_SUBGROUPINFO(params[1], "2_", 27, optimAero, AP_OA_Params),
	/*TBD figure out what SUBGROUPINFO takes in that 27 is BS*/
	AP_SUBGROUPVARPTR(drivers[1], "2_", 58, optimAero, backend_var_info[1]),
#endif

#if OA_MAX_INSTANCES > 2
	// @Group: 1_
	// @Path: AP_OA_Params.cpp
	AP_SUBGROUPINFO(params[2], "3_", 29, optimAero, AP_OA_Params),
	/*TBD figure out what SUBGROUPINFO takes in that 25 is BS*/
	AP_SUBGROUPVARPTR(drivers[2], "3_", 59, optimAero, backend_var_info[2]),
#endif

	AP_GROUPEND
};

const AP_Param::GroupInfo *optimAero::backend_var_info[OA_MAX_INSTANCES];

optimAero::optimAero()
{

	if (_singleton) {

		#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
		AP_HAL::panic("Too many oa modules");
		#endif

		return;
	}
	_singleton = this;

	AP_Param::setup_object_defaults(this, var_info);
	
}

/*
 * Get the  singleton
 */
optimAero *optimAero::_singleton;
optimAero *optimAero::get_singleton()
{
    return _singleton;
}

void optimAero::init(void)
{

	if(num_instances != 0 ){
		//init called a 2nd time
		return;
	}

	for(uint8_t i=0, serial_instance =0; i<OA_MAX_INSTANCES; i++) {
		//serial_instance will be increased in detect_instance
		detect_instance(i, serial_instance);

		if(drivers[i] != nullptr){
			//we loaded a driver for this instance
			num_instances = i+1;
		}

		state[i].status = optimAero_NotConnected;

	}//end of for loop

}

void optimAero::update400hz(void){
	
	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			drivers[i]->update400hz();
		}

	}//end of for	
}

void optimAero::update100hz(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}	
			drivers[i]->update100hz();
		}
	}//end of for
}

void optimAero::update50hz(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			drivers[i]->update50hz();
		}

	}//end of for
}

void optimAero::update10hz(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			drivers[i]->update10hz();
		}
	}//end of for
}

void optimAero::update3hz(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			drivers[i]->update3hz();
		}

	}//end of for
}

void optimAero::update1hz(void)
{

	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {
			if(params[i].type == optimAero_TYPE_NONE){
				//allow user to disable at run time
				state[i].status = optimAero_NotConnected;
				continue;
			}
			drivers[i]->update1hz();
		}

	}//end of for
}

bool optimAero::_add_backend(AP_OA_Backend *backend)
{

	if(!backend){
		return false;
	}

	if(num_instances == OA_MAX_INSTANCES){
		AP_HAL::panic("Too many oa unit backends");
	}

	drivers[num_instances++] = backend;
	return true;
}

/*detect if an instance of oa is connected*/
void optimAero::detect_instance(uint8_t instance, uint8_t& serial_instance)
{ 
	enum optimAero_Type _type = (enum optimAero_Type)params[instance].type.get();

	switch(_type){
	
		/* _add_backend insert, detect(serial_instance)...*/
		case optimAero_TYPE_SERIAL:
			if(AP_OA_Serial::detect(serial_instance)) {
				drivers[instance] = new AP_OA_Serial(state[instance], params[instance], serial_instance++);
			}
			break;
		case optimAero_TYPE_EXTCOMP:
			break;
		case optimAero_TYPE_EXTNAV:
			break;
		default:
			break;

	}//end of switch

	if(drivers[instance] && state[instance].var_info){
		backend_var_info[instance] = state[instance].var_info;
		AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
	}
}


AP_OA_Backend *optimAero::get_backend(uint8_t id) const{

	if(id >= num_instances){
		return nullptr;
	}

	if(drivers[id] != nullptr){
		if(drivers[id]->type() == optimAero_TYPE_NONE){
			//it isn't her disabed at runtime
			return nullptr;
		}
	}
	return drivers[id];
}

AP_OA_Backend *optimAero::find_instance(void) const
{
	//adapted code not sure if requried
	for(uint8_t i=0; i<num_instances; i++){
		AP_OA_Backend *backend = get_backend(i);
		if(backend != nullptr){
			return backend;
		}
	}//end of for

	return nullptr;

}


void optimAero::Log_optimAero()
{

	/*if(_log_oa_bit == uint32_t(-1)){
		return;
	}*/

	#if 0
	AP_Logger &logger = AP::logger();
	if(!logger.should_log(_log_oa_bit)){
		return;
	}

	for(uint8_t i=0; i<OA_MAX_INSTANCES; i++){
		const AP_OA_Backend *s = get_backend(i);
		if( s == nullptr){
			continue;
		}

		//define const struct log packet

		// AP::logger().WriteBlock(&pkt, sizeof(pkt));

	}//endfor
	#endif
    //AP_Logger *logger = AP_Logger::get_singleton();
    //logger->Write_OA_Arduino(0); //hrt
	//logger->Write_OA_Arduino(1); //thermister and arm data
	//logger->Write_OA_Arduino(2); //analog
	//logger->Write_OA_Arduino(9); //tmotor


}

void optimAero::send_esc_telemetry_mavlink(uint8_t mav_chan)
{

	if(num_instances < 1){
		return;
	}

	uint8_t temperature[4] {};
    uint16_t voltage[4] {};
    uint16_t current[4] {};
    uint16_t totalcurrent[4] {};
    uint16_t rpm[4] {};
    uint16_t count[4] {};
    //uint32_t now = AP_HAL::millis();


	for(uint8_t j=0; j < num_instances; j++)
	{
		if(drivers[j] != nullptr) 
		{
			if(params[j].type == optimAero_TYPE_SERIAL)
			{
				//if( (now - state[j].last_reading_ms < 40000) && state[j].status == optimAero_SerialGood ){

				for(int i=0; i<6; i++)
				{
			        uint8_t idx 		= i % 4;
					temperature[idx] 	= state[j].mos_tmp[i]; 			 //degC
					voltage[idx] 		= (uint16_t)state[j].volt[i]*10;    //centi-V
					current[idx] 		= (uint16_t)state[j].curr[i]*10;    //centi-A
					rpm[idx] 			= (uint16_t)state[j].rpm[i]; 
					count[idx] 		    = state[j].mot_msg_num[i];
					totalcurrent[idx]   = state[j].cap_tmp[i];

					if(i==3){
						if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
                			return;
            			}
		                mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
					}
				}

				if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
					return;
				}   
				mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
	

			}//end of param esc
		}//driver is real
	}

	if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
		return;
	}
	mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
}

void optimAero::sendBuffer(OA_Parser &parser_in){


	for(uint8_t i=0; i < num_instances; i++){
		if(drivers[i] != nullptr) {			
			drivers[i]->populateBuffer(parser_in);
		}

	}//end of for
}

namespace AP {

	optimAero *oa()
	{
		return optimAero::get_singleton();
	}
};
