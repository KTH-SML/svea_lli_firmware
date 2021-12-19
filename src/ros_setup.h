#ifndef ROS_SETUP
#define ROS_SETUP

#include <Arduino.h>
#include <ros.h>
#include "svea_teensy.h"
#include "pwm_reader.h"

/*
 * SETUP ROS
 */

/*!
 * @brief Callback function for control requests from ROS
 * Interprets the message and sends the values to the pwm board
 * through actuate().
 * 
 * @param data Message to be evaluated
 */
void callbackCtrlRequest(const lli_ctrl_in_t& data){  
  SW_ACTUATION[0] = STEERING_DIRECTION * data.steering;
  SW_ACTUATION[1] = data.velocity;
  
  // Set the on/off values
  for (int i=0; i<3; i++){
    // Only change gear/diff settings if the corresponding enable change bit is set
    if(data.trans_diff & bit(ENABLE_ACT_CHANGE_BITS[i])){ 
      int8_t is_on = data.trans_diff & bit(ACT_BITS[i]);
      SW_ACTUATION[i+2] = is_on ? MSG_TO_ACT_ON[i]:MSG_TO_ACT_OFF[i]; 
    }
    else { // Otherwise use the previous value
      SW_ACTUATION[i+2] = -128;
    }
  }
  SW_IDLE = false; 
  SW_T_RECIEVED = millis();
  if (!pwm_reader::REM_OVERRIDE && !SW_EMERGENCY){
    actuate(SW_ACTUATION);
  }
}

/*!
 * @brief Callback function for emergency requests from ROS
 * Set/clear the emergency flag depending on message content.
 * The ID field functionality is not yet implemented.
 * 
 * @param data Message to be evaluated
 */
void callbackEmergency(const svea_msgs::lli_emergency& data){
    SW_EMERGENCY = data.emergency;
    SW_IDLE = false;
    SW_T_RECIEVED = millis();
}

//! Setup ROS
void rosSetup() {
  nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
  nh.initNode();
  // NOTE: Putting advertise before subscribe destroys 
  //       EVERYTHING :DDDD~~~~~
  nh.subscribe(ctrl_request);
  nh.subscribe(emergency_request);
  nh.advertise(remote_pub);
  nh.advertise(ctrl_actuated_pub);
  nh.advertise(encoder_pub);
  nh.advertise(debug_pub);
}
// END OF ROS SETUP
#endif