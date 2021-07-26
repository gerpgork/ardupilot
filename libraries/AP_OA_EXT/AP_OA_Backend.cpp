#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "optimAero.h"
#include "AP_OA_Backend.h"

extern const AP_HAL::HAL& hal;

/*base class const*/

AP_OA_Backend::AP_OA_Backend(optimAero::optimAero_State &_state, AP_OA_Params &_params) :
		state(_state),
		params(_params)
{
	_backend_type = (optimAero::optimAero_Type)params.type.get();
}

optimAero::optimAero_Status AP_OA_Backend::status() const {

	if(params.type == optimAero::optimAero_TYPE_NONE) {
		//turned off
		return optimAero::optimAero_NotConnected; //interesting....TBD
	}

	return state.status;

}


bool AP_OA_Backend::has_data() const {
	return(( state.status != optimAero::optimAero_NotConnected) &&
		( state.status != optimAero::optimAero_NoData));
}

void AP_OA_Backend::update_status()
{
	//call of set_status	

}

void AP_OA_Backend::set_status(optimAero::optimAero_Status _status)
{

	state.status = _status;

	//update
	if( _status == optimAero::optimAero_SerialGood){
		//state.valid_cound++	
	}

}
