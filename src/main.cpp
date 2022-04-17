#include <Arduino.h>
#include <ros.h>
#include <limits>
#include <i2c_driver.h>
#include <svea_msgs/lli_ctrl.h>
#include <svea_msgs/lli_encoder.h>
#include <svea_msgs/lli_emergency.h>
#include "encoders.h"
#include "settings.h"
#include "svea_teensy.h"
#include "pwm_reader.h"
#include "utility.h"
#include "Adafruit_MCP23008.h"
#include "led_control.h"
#include "buttons.h"
#include "output_func.h"
#include "ros_setup.h"
#include "check_e_brake_steering_calib.h"
#include "can_setup.h"

/*! @file svea_arduino_src.ino*/ 

/* GPIO extender variables */
constexpr uint8_t GPIO_ADDRESS = 0;
constexpr uint8_t SERVO_PWR_ENABLE_PIN = 3;
Adafruit_MCP23008 gpio_extender(Master1);

//! Arduino setup function
void setup() {
  setupActuation();
  /* ROS setup */
  rosSetup();
  pinMode(LED_BUILTIN, OUTPUT);
  gpio_extender.begin(GPIO_ADDRESS);
  gpio_extender.pinMode(SERVO_PWR_ENABLE_PIN, OUTPUT);
  buttons::setup(gpio_extender);
  led::setup(gpio_extender);
  pwm_reader::setup();
  encoders::setup();
  can_setup::setup();
}

//! Main loop
void loop() {
  static bool all_idle = false;
  int sw_status = nh.spinOnce();
  unsigned long d_since_last_msg = millis() - SW_T_RECIEVED;

  // Check if ROS is active
  if (sw_status != ros::SPIN_OK || d_since_last_msg > SW_TIMEOUT) {
    SW_IDLE = true;
  }

  checkEmergencyBrake();

  if (pwm_reader::REM_IDLE && SW_IDLE && !SW_EMERGENCY) {
    if (!all_idle){
      actuate(IDLE_ACTUATION);
      gpio_extender.digitalWrite(SERVO_PWR_ENABLE_PIN, LOW);
    }
    all_idle = true;
  } else {
    gpio_extender.digitalWrite(SERVO_PWR_ENABLE_PIN, HIGH);
    all_idle = false;
  }

  // Remote reading
  int8_t remote_actuations[5];
  if (pwm_reader::processPwm(remote_actuations)){
    if (!pwm_reader::REM_IDLE){
      publishRemoteReading(remote_actuations);
      if ((SW_IDLE && !SW_EMERGENCY) || pwm_reader::REM_OVERRIDE){
        actuate(remote_actuations);
      }
      if (d_since_last_msg > EMERGENCY_T_CLEAR_LIMIT
          && pwm_reader::REM_OVERRIDE 
          && SW_EMERGENCY) {
        SW_EMERGENCY = false;
      }
    }
  }

  // Encoders update
  encoders::encoder_reading_t reading;
  if (encoders::processEncoderTicks(reading)){
    EncoderReadingToMsg(reading, MSG_ENCODER);
    encoder_pub.publish(&MSG_ENCODER);
  }
  if (gpio_extender.update() == DONE){
    ;
  }

  // Read VESC CAN messages
  can_setup::read_can_data();

  // Buttons update
  buttons::updateButtons();
  
  // Steering calib
  bool is_calibrating = callibrateSteering();

  // LED logic
  // If calibration is not ongoing then enter
  if (is_calibrating == false){
    // 
    if (all_idle && !SW_EMERGENCY) {
      // Both are active
      if(all_idle && !SW_EMERGENCY){
        led::blinkColour(led::color_blue, 1000, 1000);
      }
      // Only no SW emergency
      else if (!SW_EMERGENCY){
        led::blinkColour(led::color_green, 1000, 1000);
      }
      // Only all_idle
      else{
        led::blinkColour(led::color_yellow, 1000, 1000);
      }
    }
    else {
      if(SW_IDLE){
        led::setLED(0, led::color_red);
      } else {
        led::setLED(0, led::color_green);
      }
      if(pwm_reader::REM_IDLE){
        led::setLED(1, led::color_red);
      } else {
        led::setLED(1, led::color_green);
      }
      if(!pwm_reader::REM_OVERRIDE){
        led::setLED(2, led::color_red);
      } else {
        led::setLED(2, led::color_green);
      }
      if(!SW_EMERGENCY){
        led::setLED(3, led::color_red);
      } else {
        led::setLED(3, led::color_green);
      }
    }
  }
  else{
    //Do nothing
  }
  
  led::updateLEDs();

  
}