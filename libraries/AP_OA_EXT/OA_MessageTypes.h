#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define MASK_ARDUINO_BATT           (1<<0)
#define MASK_ARDUINO_ARMS           (1<<1)
#define MASK_ARDUINO_TEMPERATURE    (1<<2)
#define MASK_ARDUINO_ANALOG         (1<<3)

#define MASK_EXT_RC                 (1<<4)
#define MASK_ARDUINO_LINKHEALTH     (1<<5)
#define MASK_EXT_RC_LINKHEALTH      (1<<6)
#define MASK_TMOTOR                 (1<<7)
#define MASK_EM_ALL                  0xFF

#define MASK_ARDUINO_ARM1           (1<<0)
#define MASK_ARDUINO_ARM2           (1<<1)
#define MASK_ARDUINO_ARM3           (1<<2)
#define MASK_ARDUINO_ARM4           (1<<3)
#define MASK_ARDUINO_ARM5           (1<<4)
#define MASK_ARDUINO_ARM6           (1<<5)


#define OPTIM_MESSAGE_SYNC              0xa1
#define OPTIM_MESSAGE_CSUM_SIZE         2
#define OPTIM_MESSAGE_HDR_SIZE          4 /*changed to four for motors*/

#define DATALINK_BUFFER_SIZE            256
#define OPTIM_MESSAGE_MAX_SIZE          120

// Message sent over uart (4 chars, 6 shorts = 4+12=16)
#pragma pack(push, 1) // exact fit - no padding
struct thrust_stand_command_ref {
    unsigned char   sync1;          // OxAC
    unsigned char   sync2;          // 0XFC
    unsigned char   msgId;          // 0x01 
    unsigned char   msgLen;         // length
    unsigned short  rc_channels[6]; // range 
};
#pragma pack(pop) //back to whatever the previous packing mode was 

//////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ID_REPEATER_HEARTBEAT  101
#define ID_BAD_AP_LINK         104
#pragma pack(push, 1) // exact fit - no padding
/*send cksum, badid, missedAP, arm/disarm */
struct rc_repeater_heartbeat_ref {
    unsigned char   sync;               // OPTIM_MESSAGE_SYNC
    unsigned char   messageID;          // AP_HEARTBEAT
    unsigned char   messageSize;        // (4chars + 2short + 3*ints == 4+4+12 bytes = 20 bytes)
    unsigned char   count;              // counter

    uint32_t        badChecksum;
    uint32_t        unknownIDs;
    uint32_t        missedAP;

    unsigned short  armStatus;
    unsigned short  csum;
};
#pragma pack(pop) //back to whatever the previous packing mode was 
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ID_EXTERNAL_AP_RC_OUT 102
#define EXTERNAL_AP_NUM_MOT 8
// external_ap_rc_out {contains id,len,count, rc values(1-8) , csum }
#pragma pack(push, 1) // exact fit - no padding
struct external_ap_rc_out_ref {
    unsigned char   sync;               // OPTIM_MESSAGE_SYNC
    unsigned char   messageID;          // EXTERNAL_AP_RC_OUT
    unsigned char   messageSize;        // (4chars + 2short + RC == 8 + 32 = 40 bytes)
    unsigned char   count;              // counter
    uint16_t        rc[EXTERNAL_AP_NUM_MOT];
    unsigned short  motor_enable;       // the arm button
    unsigned short  csum;
};
#pragma pack(pop) //back to whatever the previous packing mode was 
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ID_EXTERNAL_AP_MIXER_OUT 103
// external_ap_mixer_out {contains id,len,count, mixer values (1-6), csum }
#pragma pack(push, 1) // exact fit - no padding
struct external_ap_mixer_out_ref {
    unsigned char   sync;               // OPTIM_MESSAGE_SYNC
    unsigned char   messageID;          // EXTERNAL_AP_MIXER_OUT
    unsigned char   messageSize;        // (4chars + short == 6 bytes)
    unsigned char   count;              // counter

    float           mixer[6];            // mixer values (rpyt,dfcx, dfcy)

    unsigned short  align;
    unsigned short  csum;
};
#pragma pack(pop) //back to whatever the previous packing mode was 

// msg 104 is bad_AP

/* MESSAGES 105,106,107 need to match arduino_interface.h*/
#define ID_MESSAGE_BATT_CELL 105
#define NUMCELL 6
#define NUMBATT 8
#pragma pack(push, 1) // exact fit - no padding
struct MsgBattCell_ref {
  unsigned char sync;
  unsigned char messageID;
  unsigned char messageSize; //MAX= 4+ 48*2 + 4 = 104
  unsigned char count;

  unsigned short cell[48]; // 8 batteries - each with 6 cell is max

  unsigned short align;
  unsigned short csum;
};
#pragma pack(pop) //back to whatever the previous packing mode was 

#define MAX_ANALOG_CAPACITY 10
#define ID_MESSAGE_ANALOG_DATA 106

#define ARDUINO_ARM_CONNECTED     1
#define ARDUINO_ARM_DISCONNECTED  0
#define ARDUINO_ARM_NOT_ENABLED  -1

struct MsgAnalogData_ref {
  unsigned char sync;
  unsigned char messageID;
  unsigned char messageSize;
  unsigned char count;
  //TBD can we make data values chars (-1,0,1)??
  int16_t data[MAX_ANALOG_CAPACITY]; // message to store analog pin readings - could be used for anything
  unsigned short csum;	
};

#define NUM_THERMISTERS 4
#define MAX_TEMPERATURE_THERMISTER 120.0f
#define ID_MESSAGE_THERMISTERS_DATA 107
struct MsgThermistersData_ref {
  unsigned char sync;
  unsigned char messageID;
  unsigned char messageSize;
  unsigned char count;

  float tempF[NUM_THERMISTERS]; 

  unsigned short numThermisters; // num of thermisters
  unsigned short csum;		
};

struct __attribute__((__packed__)) t_motorESC {
	unsigned char 	start_bit[4];  /* 0x9b = 155*/
	unsigned int 	  msg_num;    /* 0x01 0x0D 0x02 */
	unsigned short 	input_thr; 	/* 0x03 0x20 = 800*/
	unsigned short 	output_thr; /* 0x03 0x52 = 850*/
	unsigned int 	  rpm;        
	unsigned short 	voltage;
	unsigned short 	current; 
	unsigned short 	mos_tmp;
	unsigned short 	cap_tmp;
}; /*24 bytes*/ 

// char =1 byte, short=2byte, long=4byte, float = 4byte
struct __attribute__((__packed__)) QUAD_OUT {
  unsigned char   start_bit[4];  /* 0x9b = 155*/
  unsigned char   msg_num[4];        
  unsigned short  input_thr[4];      
  unsigned short  output_thr[4];       
  unsigned short  voltage[4];
  unsigned short  current[4];
  unsigned short  mos_tmp[4];
  unsigned short  cap_tmp[4];
  unsigned long   rpm[4];        
}; /*72 bytes*/

// char =1 byte, short=2byte, long=4byte, float = 4byte
struct __attribute__((__packed__)) HEX_OUT {
  unsigned char   start_bit[4];  /* 0x9b = 155*/
  unsigned char   msg_num[6];        
  unsigned short  input_thr[6];      
  unsigned short  output_thr[6];       
  unsigned short  voltage[6];
  unsigned short  current[6];
  unsigned short  mos_tmp[6];
  unsigned short  cap_tmp[6];
  unsigned long   rpm[6];        
}; /*106 bytes*/


struct __attribute__((__packed__)) tach_rpm {
	unsigned char 	start_bit[3];  /* 0x1b*/
  unsigned char   counter;
	unsigned int 	  tach_rpm;
}; 

#ifdef __cplusplus
}
#endif

 