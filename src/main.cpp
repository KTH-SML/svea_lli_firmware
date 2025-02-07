#include <Arduino.h>
#include <limits>
#include <ros.h>

#include "svea_msgs/lli_ctrl.h"
#include "svea_msgs/lli_emergency.h"
#include "svea_msgs/lli_encoder.h"

#include "control/buttons.h"
#include "control/encoders.h"
#include "control/pwm_reader.h"

#include "external_ic/IMU.h"
#include "external_ic/gpio_ext.h"

#include "settings.h"
#include "svea_teensy.h"
#include "utility.h"

ros::NodeHandle nh;

void rosSetup() {
    nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
    nh.initNode();
    // Delay to avoid "Tried to publish before configured" errors
    delay(2000);

    // Subscribe first, then advertise topics.
    nh.subscribe(ctrl_request);
    nh.subscribe(emergency_request);

    nh.advertise(remote_pub);
    nh.advertise(ctrl_actuated_pub);
    nh.advertise(encoder_pub);
    nh.advertise(debug_pub);
}

void scani2c() {
    byte error, address;
    int nDevices = 0;

    Serial.println("Scanning...");

    for (address = 1; address < 127; address++) {
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}

void setup() {
    delay(1000);
    rosSetup();

    setupActuation();
    Encoders::setupEncoders();

    pinMode(LED_BUILTIN, OUTPUT);
    Wire1.begin();

    setup_gpio();
    pwm_reader::setup();

    if (!IMU::init(nh))
        Serial.println("IMU init failed");
    Serial.println("Setup done");
}
int sw_status = 0;
void loop() {

    sw_status = nh.spinOnce();
    unsigned long d_since_last_msg = millis() - SW_T_RECIEVED;
    checkEmergencyBrake();

    int8_t remote_actuations[5];
    if (pwm_reader::processPwm(remote_actuations)) {
        if (!pwm_reader::REM_IDLE) {
            publishRemoteReading(remote_actuations);
            if ((SW_IDLE && !SW_EMERGENCY) || pwm_reader::REM_OVERRIDE) {
                actuate(remote_actuations);
            }
            if (d_since_last_msg > EMERGENCY_T_CLEAR_LIMIT && pwm_reader::REM_OVERRIDE && SW_EMERGENCY) {
                SW_EMERGENCY = false;
            }
        }
    }

    if (sw_status != ros::SPIN_OK || d_since_last_msg > SW_TIMEOUT)
        SW_IDLE = true;

    IMU::update();
    // Encoder pub is not working
    // Encoders::publishEncoder();
}