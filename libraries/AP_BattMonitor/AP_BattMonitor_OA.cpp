#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_OA.h"

#include <stdio.h>
/*
reads data from external source
 */
extern const AP_HAL::HAL& hal;


/// Constructor
AP_BattMonitor_OA::AP_BattMonitor_OA(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params,
                                       uint8_t instance) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _instance(instance)
{
    printf("BATT OA\n");
}

// read the voltage 
void
AP_BattMonitor_OA::read()
{

    const optimAero *oa_ = AP::oa();
    AP_Int8 typeOA;

    for(uint8_t i = 0; i< oa_->num_oa_connections(); i++)
    {

        typeOA = oa_->get_type(i);
        if(typeOA == optimAero::optimAero_TYPE_SERIAL)
        {
            //we could potentially have battery data
            if(oa_->get_batt_health(i))
            {

                if(oa_->get_batt_voltage(i) > 55){

                }else{
                    _state.voltage                      = oa_->get_batt_voltage(i);
                    _state.current_amps                 = oa_->get_batt_current(i);
                    _state.last_time_micros             = AP_HAL::micros();
                    _state.healthy                      = 1;      
                    if(!debug_oa){
                        debug_oa = true;
                        gcs().send_text(MAV_SEVERITY_INFO,"OA BATT VOLTAGE = %f",_state.voltage);
                    }
                }
  
            }else{
                _state.healthy                      = 0;
            }
    
        }
    }

}

