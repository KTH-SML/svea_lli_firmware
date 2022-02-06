#ifndef CHECK_E_BRAKE_STEERING_CALIB
#define CHECK_E_BRAKE_STEERING_CALIB

#include <Arduino.h>
#include "svea_teensy.h"
#include "buttons.h"
#include "pwm_reader.h"
#include "led_control.h"

/*!
 * @brief Check if the emergency brake should be engaged.
 * Should be called every update loop.
 * The emergency brake will be activated if the SW_EMERGENCY
 * flag is true. A braking sequence is then initiated.
 * The sequence first make sures that the ESC is not in
 * a reverse state, and then applies full brakes. 
 * For proper functionality all other actuation sources
 * must respect the SW_EMERGENCY flag and not send actuation
 * signals until it is cleared.
 * 
 * @return true if the emergency brake is engaged, false otherwise.
 */
bool checkEmergencyBrake(){
  enum States {
    NO_EMERGENCY,
    EMERGENCY_SET,
    WAIT_FOR_UNSET_REVERSE,
    BRAKING,
    DONE_BRAKING,
  };
  static States state = NO_EMERGENCY;
  static unsigned long last_time = millis();
  const unsigned long reverse_wait_time = 50; // (ms)
  const int8_t init_brake_actuation[] = {-128,15,-128,-128,-128};
  const int8_t brake_actuation[] = {-128,-127,-128,-128,-128};
  // const unsigned long minimum_emergency_duration = 500;
  unsigned long wait_duration = (millis() - last_time);
  if (SW_EMERGENCY == false) {// && 
      //wait_duration > minimum_emergency_duration){
    state = NO_EMERGENCY;
  }
  switch (state)
  {
  case NO_EMERGENCY:
    if (SW_EMERGENCY){
      state = EMERGENCY_SET;
    } else {
      break;
    }
  case EMERGENCY_SET:
    actuate(init_brake_actuation);
    last_time = millis();
    state = WAIT_FOR_UNSET_REVERSE;
    break;
  case WAIT_FOR_UNSET_REVERSE:
    if (wait_duration > reverse_wait_time) {
      state = BRAKING;
    } else {
      break;
    }
  case BRAKING:
    actuate(brake_actuation);
    state = DONE_BRAKING;
    break;
  case DONE_BRAKING:
    break;
  default:
    break;
  }
  return state != NO_EMERGENCY;
}

/*!
 * @brief Steering callibration functionality. Should be called in every loop update.
 *
 * Initiate callibration by holding down button 0 for 1 second.
 * The LEDs should turn yellow. Now turn the tires as far to the left
 * as they can go without pushing against the chassis. 
 * Push button 0 again. The LEDs should turn blue. 
 * Turn the tire as far to the right as they can go without
 * pushing against the chassis. 
 * Push button 0 again and the LEDs should blink for a short while.
 * The callibration is complet and the values have been saved to flash.
 * 
 * The calibration process can be aborted by pushing button 1.
 * 
 * @return true if a calibration is ongoing, false otherwise
 */
bool callibrateSteering(){
  enum CalibState {
    NOT_CALIBRATING,
    TURN_LEFT,
    TURN_RIGHT,
    DONE,
  };
  const uint8_t calib_button = 0;
  const uint8_t abort_button = 1;
  static CalibState state = NOT_CALIBRATING;
  static float max_pwm = DEFAULT_PWM_OUT_MAX_PW[0];
  static float min_pwm = DEFAULT_PWM_OUT_MIN_PW[0];
  static unsigned long done_time;
  const unsigned long done_duration = 1500; //ms

  if (buttons::readEvent(abort_button) == buttons::PRESSED){
    state = NOT_CALIBRATING;
    if(loadSteeringValues(min_pwm, max_pwm)){
      setSteeringPwm(min_pwm, max_pwm);
    }
  }
  // State flow
  switch (state)
  {
    case NOT_CALIBRATING:
      if (buttons::readEvent(calib_button) == buttons::LONG_PRESSED
          && !pwm_reader::REM_IDLE){
        state = TURN_LEFT;
        int steer_ix = 0;  // Index for steering PWM value
        max_pwm = DEFAULT_PWM_OUT_MAX_PW[steer_ix];
        min_pwm = DEFAULT_PWM_OUT_MIN_PW[steer_ix];
        setSteeringPwm(min_pwm, max_pwm);
        led::setLEDs(led::color_yellow);
      }
      break;
    case TURN_LEFT:
      if (buttons::readEvent(calib_button) == buttons::LONG_PRESSED){
        min_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES*PWM_OUT_FREQUENCY);
        state = TURN_RIGHT;
        led::setLEDs(led::color_blue);
      }
      break;
    case TURN_RIGHT:
      if (buttons::readEvent(calib_button) == buttons::LONG_PRESSED){
        max_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES*PWM_OUT_FREQUENCY);
        done_time = millis();
        led::pushLEDs(led::color_blue);
        state = DONE;
      }
      break;
    case DONE:
    // Once entered DONE wait for 1.5 secs until exit the calibration
      if(millis() - done_time < done_duration){
        setSteeringPwm(min_pwm, max_pwm);
        saveSteeringValues(min_pwm, max_pwm);
        led::blinkColour(led::color_green, 50, 50);
      } else {
        state = NOT_CALIBRATING;
      }
      break;
  }
  return (state != NOT_CALIBRATING);
}

#endif