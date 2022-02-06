#ifndef CAN_SETUP
#define CAN_SETUP

#include <ACAN_T4.h>
#include <FlexCAN_T4.h>
namespace can_setup{

static const uint32_t CAN_BITRATE = 500 * 1000 ;
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;


void setup() {
    
    myCan.begin();
    //myCan.setBaudRate(CAN_BITRATE);
    CANFD_timings_t config;
config.clock = CLK_8MHz;
config.baudrate = CAN_BITRATE;
config.baudrateFD = CAN_BITRATE;
config.propdelay = 190;
config.bus_length = 1;
config.sample = 70;
myCan.setBaudRate(config);

}


} /*End of namespace*/
#endif /*CAN_SETUP */