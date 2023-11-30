#pragma once

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include "svea_msgs/lli_encoder.h"
#include "svea_teensy.h"
#include <ros.h>
#include <svea_msgs/lli_encoder.h>
#include <RotaryEncoder.h>

#define ENCODER_SAMPLE_INTERVAL 100000 // in micro seconds

#define ENCODER_PIN_R1 22 // 22 Right wheel encoder tick pin 1
#define ENCODER_PIN_R2 23 // 23 Right wheel encoder tick pin 2
#define ENCODER_PIN_L1 20 // 20 Left wheel encoder tick pin 1
#define ENCODER_PIN_L2 21 // 21 Left wheel encoder tick pin 2

#define SVEA_WheelEncoders
#include <Arduino.h>

RotaryEncoder lEncoder(ENCODER_PIN_R2, ENCODER_PIN_R1, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder rEncoder(ENCODER_PIN_L2, ENCODER_PIN_L1, RotaryEncoder::LatchMode::TWO03);

namespace WheelEncoders
{
    // RotaryEncoder leftEncoder(ENCODER_PIN_R2, ENCODER_PIN_R1, RotaryEncoder::LatchMode::TWO03);
    // RotaryEncoder rightEncoder(ENCODER_PIN_R2, ENCODER_PIN_R1, RotaryEncoder::LatchMode::TWO03);

    svea_msgs::lli_encoder wheel_encoders_msg;
    ros::Publisher wheel_encoders_pub("lli/encoder", &wheel_encoders_msg);

    ///*!
    // * @defgroup EncoderVariables Wheel encoder variables
    // */
    ///*@{*/
    // static uint8_t ENC_TICK_COUNT[2][2]; //!< Right and left wheel tick count
    // static uint8_t ENC_BUFFER_IX;        //!< Encoder buffer index
    // static const uint8_t RIGHT_ENC_IX = 0;
    // static const uint8_t LEFT_ENC_IX = 1;
    ////! Time of last right and left tick event
    // static uint32_t ENC_TICK_TIME[2][2];
    ///*@}*/
    //
    ///*!
    // * @brief Switch to the next pwm duration buffer and return the previous buffer index
    // */
    // inline uint8_t switchEncoderBuffer()
    //{
    //    ENC_BUFFER_IX = ENC_BUFFER_IX ^ 1;
    //    return ENC_BUFFER_IX ^ 1;
    //}
    //
    // struct encoder_reading_t
    //{
    //    uint8_t right_ticks = 0;
    //    uint8_t left_ticks = 0;
    //    uint32_t right_time_delta = 0;
    //    uint32_t left_time_delta = 0;
    //};
    ///*!
    // * @brief Count the encoder ticks since last call and send the result to ROS
    // *
    // * The time_delta is given in micro second resolution.
    // */
    // bool processEncoderTicks(encoder_reading_t &reading)
    //{
    //    static uint32_t last_sent_time = micros();
    //
    //    // Calculate and record duration
    //    uint32_t current_time = micros();
    //    uint32_t dt = current_time - last_sent_time;
    //    bool WheelEncoders_processed = false;
    //    if (dt > ENCODER_SAMPLE_INTERVAL)
    //    {
    //        noInterrupts(); // Temporarily disable interrupts on encoder pins
    //        uint8_t buffer_ix = switchEncoderBuffer();
    //        uint32_t previous_right_tick_time = ENC_TICK_TIME[ENC_BUFFER_IX][RIGHT_ENC_IX];
    //        uint32_t previous_left_tick_time = ENC_TICK_TIME[ENC_BUFFER_IX][LEFT_ENC_IX];
    //        interrupts(); // Enable interrupts on the encoder pins again
    //        // Calculate and record tick changes
    //        reading.right_ticks = ENC_TICK_COUNT[buffer_ix][RIGHT_ENC_IX];
    //        reading.left_ticks = ENC_TICK_COUNT[buffer_ix][LEFT_ENC_IX];
    //        // Update the times for the last ticks
    //        reading.right_time_delta = ENC_TICK_TIME[buffer_ix][RIGHT_ENC_IX] - previous_right_tick_time;
    //        reading.left_time_delta = ENC_TICK_TIME[buffer_ix][LEFT_ENC_IX] - previous_left_tick_time;
    //        if (reading.right_ticks == 0)
    //        {
    //            reading.right_time_delta = 0;
    //            ENC_TICK_TIME[buffer_ix][RIGHT_ENC_IX] = current_time - ENCODER_SAMPLE_INTERVAL;
    //        }
    //        if (reading.left_ticks == 0)
    //        {
    //            reading.left_time_delta = 0;
    //            ENC_TICK_TIME[buffer_ix][LEFT_ENC_IX] = current_time - ENCODER_SAMPLE_INTERVAL;
    //        }
    //        last_sent_time = current_time;
    //        ENC_TICK_COUNT[buffer_ix][RIGHT_ENC_IX] = 0;
    //        ENC_TICK_COUNT[buffer_ix][LEFT_ENC_IX] = 0;
    //        WheelEncoders_processed = true;
    //    }
    //    return WheelEncoders_processed;
    //}
    // static void rightEncoderIsr()
    //{
    //    ENC_TICK_COUNT[ENC_BUFFER_IX][RIGHT_ENC_IX]++;
    //    ENC_TICK_TIME[ENC_BUFFER_IX][RIGHT_ENC_IX] = micros();
    //}
    //
    // static void leftEncoderIsr()
    //{
    //    ENC_TICK_COUNT[ENC_BUFFER_IX][LEFT_ENC_IX]++;
    //    ENC_TICK_TIME[ENC_BUFFER_IX][LEFT_ENC_IX] = micros();
    //}
    void leftCheckPos()
    {
        lEncoder.tick();
    }
    void rightCheckPos()
    {
        rEncoder.tick();
    }

    void setup()
    {
        // ENC_BUFFER_IX = 0;
        nh.advertise(wheel_encoders_pub);
        nh.negotiateTopics();
        // pinMode(ENCODER_PIN_R1, OUTPUT);
        // pinMode(ENCODER_PIN_L1, OUTPUT);
        // digitalWrite(ENCODER_PIN_R1, LOW);
        // digitalWrite(ENCODER_PIN_L1, LOW);
        //
        // pinMode(ENCODER_PIN_R2, INPUT_PULLDOWN);
        // pinMode(ENCODER_PIN_L2, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R2), leftCheckPos, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L2), rightCheckPos, CHANGE);

        // ROS stuff
        // ...
    }
    unsigned long lastUpdateTime = 0;
    void update()
    {
        lEncoder.tick();
        rEncoder.tick();
        unsigned long currTime = millis();
        //if (currTime - lastUpdateTime >= ENCODER_SAMPLE_INTERVAL)
        //{
            lastUpdateTime = currTime;
            wheel_encoders_msg.right_ticks = rEncoder.getPosition();
            wheel_encoders_msg.left_ticks = lEncoder.getPosition();
            wheel_encoders_msg.right_time_delta = rEncoder.getMillisBetweenRotations();
            wheel_encoders_msg.left_time_delta = lEncoder.getMillisBetweenRotations();
            wheel_encoders_pub.publish(&wheel_encoders_msg);
            // Serial.println("RIGHT: " + String(wheel_encoders_msg.right_ticks) + " LEFT: " + String(wheel_encoders_msg.left_ticks) + " RIGHT TIME: " + String(wheel_encoders_msg.right_time_delta) + " LEFT TIME: " + String(wheel_encoders_msg.left_time_delta));
        //}
    }
    // Rest of the code...
}