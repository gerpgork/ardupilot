#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    init_optimaero(); 
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    update_optimaero_100Hz();
 }
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    update_optimaero_50Hz();
 }
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    update_optimaero_10Hz();
 }
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
    update_optimaero_3Hz();
 }
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    update_optimaero_1Hz();
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif



#ifdef USERHOOK_400HZ
void Copter::userhook_400hz(){
    update_optimaero_400Hz();
 }
#endif