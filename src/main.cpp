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

SVEA::IMU imu_sensor(nh);

//! Setup ROS
void rosSetup() {
    nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
    nh.initNode();
    // NOTE: Putting advertise before subscribe destroys
    //       EVERYTHING :DDDD~~~~~

    nh.subscribe(ctrl_request);

    nh.subscribe(emergency_request);

    nh.advertise(remote_pub);
    nh.negotiateTopics();

    nh.advertise(ctrl_actuated_pub);
    nh.negotiateTopics();

    nh.advertise(encoder_pub);
    nh.negotiateTopics();

    nh.advertise(debug_pub);
    nh.negotiateTopics();
}

void scani2c() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
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

//! Arduino setup function
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Starting setup");

    while (nh.connected()) {
        nh.spinOnce();
    }
    setupActuation();
    Encoders::setupEncoders();

    pinMode(LED_BUILTIN, OUTPUT);
    Wire1.begin();

    // scani2c();

    setup_gpio();
    pwm_reader::setup();

    if (!imu_sensor.open()) {
        // TODO: Handle error
    }

    rosSetup();

    Serial.println("Setup done");
}

// Servo turned on by default
static bool servo_idle = false;
int l = 0;
//! Main loop
void loop() {
    int sw_status = nh.spinOnce();
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

    if (sw_status != ros::SPIN_OK || d_since_last_msg > SW_TIMEOUT) {
        SW_IDLE = true;
    }
    encoder_pub.publish(&Encoders::encoder_msg);
    // Serial.printf("RDelta%d\n", &Encoders::encoder_msg.right_time_delta);
    imu_sensor.update();
}
