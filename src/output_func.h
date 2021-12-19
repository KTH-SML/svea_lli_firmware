#ifndef OUTPUT_FUNC
#define OUTPUT_FUNC

#include <Arduino.h>
#include "pwm_reader.h"
#include "svea_teensy.h"

/*
 * ACTUATION AND OUTPUT FUNCTIONS
 */

/*! 
 * @brief Set actuation PWM
 * convert a 8 bit actuation value to a pwm signal and send it to the pwm board. 
 * The value gets scaled to a duration that suits the servos (approximately 
 * 1 to 2 milli seconds).
 * @see INPUT_SCALE
 * @see PWM_NEUTRAL_TICK
 * To avoid servo jitter at neutral a small dead zone exists around 0. 
 * @see DEAD_ZONE
 * @param channel The channel (pin) of the pwm board to send to. 
 * @param in_value Value, between -127 and 127. to send.
 */ 
inline void setPwmDriver(uint8_t channel, int8_t actuation_value){
  if (abs_difference(actuation_value, ACTUATION_NEUTRAL) < DEAD_ZONE) {
    actuation_value = ACTUATION_NEUTRAL;
  }
  uint16_t off_tick = PWM_OUT_NEUTRAL_TICK[channel] + 
                      OUTPUT_SCALE[channel] * actuation_value;
  ACTUATED_TICKS[channel] = off_tick;
  analogWrite(PWM_OUT_PINS[channel], off_tick);
}

/*! @brief Send settings to the pwm board through setPwmDriver()
 * If any setting or actuation code have changed, the current 
 * actuation values and flags will be published on /lli/ctrl_actuated.
 * If nothing have been changed, nothing will be sent to the
 * pwm board or /lli/ctrl_actuated.
 * @see setPwmDriver
 * @param actuation_values array containg 5 values. 
 */
void actuate(const int8_t actuation_values[]){
  /* Set steering and velocity */
  static int8_t previous_setting[5] = {IDLE_ACTUATION[0],
                                       IDLE_ACTUATION[1],
                                       MSG_TO_ACT_OFF[0],
                                       IDLE_ACTUATION[3],
                                       IDLE_ACTUATION[4]};
  static uint8_t last_actuated_code = 0; // Code that was last sent to ROS
  int8_t has_changed = 0;
  for (int i=0; i<5; i++){
    if (actuation_values[i] != previous_setting[i] && actuation_values[i] != -128) {
      setPwmDriver(i, actuation_values[i]);
      previous_setting[i] = actuation_values[i];
      has_changed++;
    }
  }
  // Send actuated values to ROS
  uint8_t actuated_code = getActuatedCode(); 
  if (has_changed > 0 || actuated_code^last_actuated_code) {
    MSG_ACTUATED.steering = STEERING_DIRECTION*previous_setting[0];
    MSG_ACTUATED.velocity = previous_setting[1];
    MSG_ACTUATED.trans_diff = bit(ENABLE_GEARCHANGE_BIT)
                            | bit(ENABLE_FDIFCHANGE_BIT)
                            | bit(ENABLE_RDIFCHANGE_BIT);
    for (int i=0; i<3; i++){
      MSG_ACTUATED.trans_diff += previous_setting[i+2] == MSG_TO_ACT_ON[i] ? bit(i):0;
    }
    MSG_ACTUATED.ctrl = actuated_code;
    ctrl_actuated_pub.publish(&MSG_ACTUATED);
    last_actuated_code = actuated_code;
  }
}

/*!
 * @brief set the control code in messages sent to the computer
 */
inline uint8_t getActuatedCode() {
  return SW_IDLE 
         | pwm_reader::REM_IDLE << 1 
         | pwm_reader::REM_OVERRIDE << 2 
         | SW_EMERGENCY << 3;
}

/*!
 * @brief Publish actuation messages to remote node
 */
void publishRemoteReading(int8_t actuation_values[5]){
  MSG_REMOTE.steering = actuation_values[0];
  MSG_REMOTE.velocity = actuation_values[1];
  // Remote messages should always enforce change
  MSG_REMOTE.trans_diff = bit(ENABLE_GEARCHANGE_BIT)
                         | bit(ENABLE_FDIFCHANGE_BIT)
                         | bit(ENABLE_RDIFCHANGE_BIT);
  for (int i=0; i<3; i++){
    if (actuation_values[i+2] == MSG_TO_ACT_ON[i]){
      MSG_REMOTE.trans_diff |= bit(i);
    }
  }
  MSG_REMOTE.ctrl = getActuatedCode();
  remote_pub.publish(&MSG_REMOTE); 
}

/*!
 * @brief Publish encoder reading to remote node
 */
void EncoderReadingToMsg(const encoders::encoder_reading_t& reading, lli_encoder_t& msg){
  msg.right_ticks = reading.right_ticks;
  msg.left_ticks = reading.left_ticks;
  msg.right_time_delta = reading.right_time_delta;
  msg.left_time_delta = reading.left_time_delta;
}

// END OF ACTUATION AND OUTPUT FUNCTIONS
#endif