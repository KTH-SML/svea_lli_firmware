#ifndef CAN_SETUP
#define CAN_SETUP

#include <FlexCAN_T4.h>

namespace can_setup{
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan3;
static const uint32_t CAN_BITRATE = 500 * 1000 ;
CAN_message_t msg ;

volatile float CAN_MTR_RPM, CAN_INPUT_VOLTS, CAN_MTR_AMPS, CAN_MTR_DUTY_CYCLE;
float MTR_USED_Ah, MTR_REGEN_Ah, MTR_USED_Wh, MTR_REGEN_Wh;
float CAN_MSFETS_TEMP, CAN_MTR_TEMP;

uint16_t VESC_ID;

void decode_can_msg_1(uint8_t data[])
{
    CAN_MTR_RPM = (float)(data[0]<<24 |
                          data[1]<<16 | 
                          data[2]<<8 | 
                          data[3]<<0);
    CAN_MTR_AMPS = (float)(data[4]<<8 |
                           data[5]<<0)/10;
    CAN_MTR_DUTY_CYCLE = (float)(data[6]<<8 |
                                 data[7]<<0)/10;
}
void decode_can_msg_2(uint8_t data[])
{
    MTR_USED_Ah = (float)(data[0]<<24 |
                          data[1]<<16 | 
                          data[2]<<8 | 
                          data[3]<<0)/10000;
    MTR_REGEN_Ah = (float)(data[4]<<24 |
                          data[5]<<16 | 
                          data[6]<<8 | 
                          data[7]<<0)/10000;
}
void decode_can_msg_3(uint8_t data[])
{
    MTR_USED_Wh = (float)(data[0]<<24 |
                          data[1]<<16 | 
                          data[2]<<8 | 
                          data[3]<<0)/10000;
    MTR_REGEN_Wh = (float)(data[4]<<24 |
                          data[5]<<16 | 
                          data[6]<<8 | 
                          data[7]<<0)/10000;
}
void decode_can_msg_4(uint8_t data[])
{
    CAN_MSFETS_TEMP = (float)(data[0]<<8 |
                              data[1]<<0)/10;
    CAN_MTR_TEMP = (float)(data[2]<<8 |
                           data[3]<<0)/10;
}
void decode_can_msg_5(uint8_t data[])
{
    CAN_INPUT_VOLTS = (float)(data[4]<<8 |
                              data[5]<<0)/10;
}

void read_can_data()
{
    if (can_setup::myCan3.read(msg))
    {           
        switch (msg.id) {
        case 0x902:
            decode_can_msg_1(msg.buf);
            break;
        case 0xE02:
            decode_can_msg_2(msg.buf);
            break;
        case 0xF02:
            decode_can_msg_3(msg.buf);
            break;
        case 0x1002:
            decode_can_msg_4(msg.buf);
            break;
        case 0x1B02:
            decode_can_msg_5(msg.buf);
            break;
        default:
            break;
        }
    } 
}

void setup() {
    myCan3.begin();
    myCan3.setBaudRate(CAN_BITRATE);
}


} /*End of namespace*/
#endif /*CAN_SETUP */