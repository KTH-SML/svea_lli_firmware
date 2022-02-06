#include <Arduino.h>
#ifndef SVEA_ACTUATION_CONSTANTS
#define SVEA_ACTUATION_CONSTANTS

/*
 * @defgroup PwmOutputVariables Output pwm variables 
*/
/*@{*/
const int PWM_OUT_BITS = 16; //!< Output pwm resolution in bits
const int PWM_OUT_RES = 1 << (PWM_OUT_BITS); //!< Output pwm resolution in number of states
const float DEFAULT_PWM_OUT_MIN_PW[] = {1.0, 1.0, 1.0, 1.0, 1.0};
const float DEFAULT_PWM_OUT_MAX_PW[] = {2.0, 2.0, 2.0, 2.0, 2.0};
const float TUNABLE_OFFSET_PWM_OUT_PW[] = {0, 0, 0, 0, 0};
static int ACTUATION_DIRECTION[] = {-1, 1, 1, 1, 1};
const int8_t ACTUATION_MIN      = -127;    //!< Minimum actuation value 
const int8_t ACTUATION_NEUTRAL  = 0;       //!< Neutral actuation value 
const int8_t ACTUATION_MAX      = 127;     //!< Maximum actuation value 
const float PWM_OUT_FREQUENCY = 50.0;     //!< Pwm frequency (Hz)
/*@}*/

//! Value that should be actuated if the corresponding bit in msg.gear_diff is not set
const int8_t MSG_TO_ACT_OFF[3] = {ACTUATION_MAX, 
                                  ACTUATION_MAX, 
                                  ACTUATION_MIN + 10};
//! Value that should be actuated if the corresponding bit in msg.gear_diff is not set
const int8_t MSG_TO_ACT_ON[3] = {ACTUATION_MIN, ACTUATION_MIN, ACTUATION_MAX};
/*@}*/ 
const int8_t IDLE_ACTUATION[5] = {0,
                                  0, 
                                  MSG_TO_ACT_OFF[0], 
                                  MSG_TO_ACT_OFF[1], 
                                  MSG_TO_ACT_OFF[2]}; 

const int8_t STEERING_CLOCKWISE = 1;
const int8_t STEERING_COUNTERCLOCKWISE = -1;
//! Sets the steering direction that is sent and recieved from ROS
const int8_t STEERING_DIRECTION = STEERING_COUNTERCLOCKWISE;

const int8_t DEAD_ZONE = 2; //!< Deadzone for actuation signals


/*!  
 * @defgroup MsgBitPositions Bit positions used for the trans_diff_ctrl field in messages
 */
/*@{*/
const uint8_t GEAR_BIT = 0;   //!< Bit used for gear value (0 unlocked, 1 locked)
const uint8_t FDIFF_BIT = 1;  //!< Bit used for front differential value (0 unlocked, 1 locked)
const uint8_t RDIFF_BIT = 2; //!< Bit used for rear differential value (0 unlocked, 1 locked)
//! Vector with the bit postitions in msg.gear_diff in order: gear, front diff, rear diff
const uint8_t ACT_BITS[3] = {GEAR_BIT, FDIFF_BIT, RDIFF_BIT}; 
//! Bit indicating if the GEAR_BIT value should be read from incoming messages
const uint8_t ENABLE_GEARCHANGE_BIT = 3;
//! Bit indicating if the front differential values should be read from incoming messages
const uint8_t ENABLE_FDIFCHANGE_BIT = 4;
//! Bit indicating if the rear differential values should be read from incoming messages
const uint8_t ENABLE_RDIFCHANGE_BIT = 5;
//! Vector with the enable change bits in order: gear, front diff, rear diff
const uint8_t ENABLE_ACT_CHANGE_BITS[3] = { ENABLE_GEARCHANGE_BIT, 
                                            ENABLE_FDIFCHANGE_BIT, 
                                            ENABLE_RDIFCHANGE_BIT };
#endif //SVEA_ACTUATION_CONSTANTS