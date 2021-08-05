/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  multicopter simulator class
*/

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

MultiCopter::MultiCopter(const char *frame_str) :
    Aircraft(frame_str)
{
    frame = Frame::find_frame(frame_str);
    if (frame == nullptr) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }

    frame->init(frame_str, &battery);

    mass = frame->get_mass();
    frame_height = 0.1;
    num_motors = frame->num_motors;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    lock_step_scheduled = true;

    //optim
    hex_motor_data.start_bit[0] = 0x9b;
    hex_motor_data.start_bit[1] = 0x8b;
    hex_motor_data.start_bit[2] = 0x7b;
    hex_motor_data.start_bit[3] = 0x6b;
    for(int i=0; i<4; i++){
        hex_motor_data.msg_num[i] = 0;
    }

    last_optim_update = AP_HAL::millis();    
}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel, rpm);

    add_shove_forces(rot_accel, body_accel);
    add_twist_forces(rot_accel);
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    // estimate voltage and current
    frame->current_and_voltage(battery_voltage, battery_current);

    battery.set_current(battery_current);

    update_dynamics(rot_accel);
    update_external_payload(input);
    optim_update();

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

void MultiCopter::optim_update(){

    const uint32_t now = AP_HAL::millis();
    wait_cnt++;
    if(now - last_optim_update >= (1000/10)  && wait_cnt > 8000){

        last_optim_update = now;
        
        for(int i=0; i<6; i++){
            hex_motor_data.input_thr[i]    = 1;
            hex_motor_data.cap_tmp[i]      = 10;
            hex_motor_data.mos_tmp[i]      = 11;
            hex_motor_data.output_thr[i]   = 2;
            hex_motor_data.rpm[i]          = 101;
            hex_motor_data.msg_num[i]      = (unsigned char)now;
            hex_motor_data.current[i]      = (unsigned short)battery_current/4;
            hex_motor_data.voltage[i]      = (unsigned short)battery_voltage;
        }


        optimAero *oa_ = AP::oa();
        for(uint8_t i = 0; i< oa_->num_oa_connections(); i++)
        {
            AP_Int8 typeOA = oa_->get_type(i);
            if(typeOA == optimAero::optimAero_TYPE_SERIAL)
            {
                write2Buffer((uint8_t *)&hex_motor_data, sizeof(hex_motor_data));
                oa_->sendBuffer(data2send_);
            }
        }
    }

}

void MultiCopter::write2Buffer(uint8_t* ptr, uint32_t len){

    uint32_t i;
    for(i=0; i<len;i++){
        if(data2send_.bytesInBuffer < DATALINK_BUFFER_SIZE){
            data2send_.datalinkBuffer[data2send_.bytesInBuffer++] = ptr[i];
        }
    }
}