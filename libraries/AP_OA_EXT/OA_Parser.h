#pragma once

#include <AP_HAL/AP_HAL.h>

#define OA_PARSER_MAX_BUFFER_SIZE 256
class OA_Parser {

    public:
    /* UART VALUES */
    unsigned int    nothingOnPort;
    unsigned char   datalinkBuffer[OA_PARSER_MAX_BUFFER_SIZE]   = {0};
    int             bytesInBuffer  = 0;
    unsigned char   *bf;
    int             iData;
    int             dataDone;
    unsigned int    csumComp;
    unsigned int    csum;
    unsigned char   readID;
    unsigned char   readSize;
    unsigned int    unknownReadID = 0;
    unsigned int    badChecksums  = 0;
    //unsigned int    nonFullWrites = 0; /*tbd code to add - track how many non-full writes we are doing*/

    unsigned int fletcher16( unsigned char *data, int len )
    {        
        if(len > 0)
        {             
            unsigned int check;
            unsigned int sum1 = 0xff, sum2 = 0xff;

            while (len) {
                int tlen = len > 21 ? 21 : len;
                len -= tlen;

                do {
                    sum1 += *data++;
                    sum2 += sum1;
                } while (--tlen);

                sum1 = (sum1 & 0xff) + (sum1 >> 8);
                sum2 = (sum2 & 0xff) + (sum2 >> 8);
            }

            // Second reduction step to reduce sums to 8 bits
            sum1 = (sum1 & 0xff) + (sum1 >> 8);
            sum2 = (sum2 & 0xff) + (sum2 >> 8);

            check = (unsigned int)(sum1 << 8) | (unsigned int)(sum2);

            return check;
        }else{
            return 0; //to keep code from bricking
        }
       
    }

    uint32_t optim_serialWrite(AP_HAL::UARTDriver *uart,  uint8_t* ptr, uint32_t len ){

        if(nullptr != uart )
        { //make sure not nullptr    
            uint32_t i;
            for(i=0;i<len;i++){
                uart->write(ptr[i]);
            }
            return len;
            // TBD add code to actually track how many bytes we are sending...or a counter to increment everytime we aren't sending full requested bytes
        }else{
            return 0;
        }
    }


};