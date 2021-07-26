/*backend for OA units*/

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "optimAero.h"

#include "OA_Parser.h" //for sim only

class AP_OA_Backend
{

public:
	//constructor
	AP_OA_Backend(optimAero::optimAero_State &_state, AP_OA_Params &_params);

	//we declare virt destroy so drivers can override with custom destroy if needed
	virtual ~AP_OA_Backend(void) {}

	//update the state
	virtual void update400hz() = 0;
	virtual void update100hz() = 0;
	virtual void update50hz() = 0;
	virtual void update10hz() = 0;
    virtual void update3hz() = 0;	
	virtual void update1hz() = 0;
	virtual void populateBuffer(OA_Parser &parser_in) = 0;

	//virtual void handle_msg(const mavlink_message_t &msg) {return;}

	optimAero::optimAero_Status status() const;
	optimAero::optimAero_Type type() const { return (optimAero::optimAero_Type)params.type.get(); }

	
	//true if sensor is retn data
	bool has_data() const;	

	//return last system time of success read
	uint32_t last_reading_ms() const { return state.last_reading_ms; }

protected:
	//update status
	void update_status();

	void set_status(optimAero::optimAero_Status status);

	optimAero::optimAero_State &state; //access by front end
	AP_OA_Params &params; //access by front end

	//semaphore for acces to shared frontend data
	HAL_Semaphore _sem;

	//type
	optimAero::optimAero_Type _backend_type;

};
