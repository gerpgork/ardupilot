#include "AP_OA_Params.h"
#include "optimAero.h"

const AP_Param::GroupInfo AP_OA_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: OA type
    // @Description: What type of OA device that is connected
    // @Values: 0:None,1:Serial,2:ExtComp,3:ExtNav
    // @User: Standard
	AP_GROUPINFO("TYPE", 1 , AP_OA_Params, type, 0),

    // @Param: FUNCTION
    // @DisplayName: OA type
    // @Description: What to allow OA device to do
    // @Values: 0:Nothing, 1:log, 2:ArduinoARM...TBD
    // @User: Standard
	AP_GROUPINFO("FUNCTION", 2 , AP_OA_Params, function, 0),
	
    // @Param: RPMSF
    // @DisplayName: OA type
    // @Description: RPM scale factor
    // @User: Standard
	AP_GROUPINFO("RPMSF", 3 , AP_OA_Params, rpm_sf, 1.0),
    // @Param: CURSF
    // @DisplayName: OA type
    // @Description: Current scale factor
    // @User: Standard
	AP_GROUPINFO("CURSF", 4 , AP_OA_Params, curr_sf, 1.0),

	AP_GROUPEND
};

AP_OA_Params::AP_OA_Params(void) {
	AP_Param::setup_object_defaults(this, var_info);
}
