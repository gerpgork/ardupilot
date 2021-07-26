#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_OA_Params {

public:
	static const struct AP_Param::GroupInfo var_info[];
	AP_OA_Params(void);

	/*don't allow copies*/
	AP_OA_Params(const AP_OA_Params &other) = delete;
	AP_OA_Params &operator = (const AP_OA_Params&) = delete;

	AP_Int8 type; 		//type of OA unit
	AP_Int8 function; 	//what to allow OA unit to perform
	AP_Float rpm_sf; 	//scale factor 
	AP_Float curr_sf;   //scale factor

};
