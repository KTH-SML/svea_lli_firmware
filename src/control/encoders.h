#include "svea_msgs/lli_encoder.h"
#include <Arduino.h>
#include <ros.h>

// CURRENTLY BROKEN

namespace Encoders {

// Publish interval (in milliseconds)
#define ENCODER_PUBLISH_INTERVAL_MS 100

// Tick counters updated in interrupts.
volatile uint8_t L_ticks = 0;
volatile uint8_t R_ticks = 0;

// Timestamps for computing delta times.
volatile uint32_t last_encoder_update_us = 0;  // using micros()
volatile uint32_t last_encoder_publish_ms = 0; // using millis()

// Persistent encoder message instance.
svea_msgs::lli_encoder encoder_msg;

// ROS publisher for the encoder message.
ros::Publisher encoder_pub("lli/encoder", &encoder_msg);

// Interrupt Service Routines (keep them minimal).
void L_TICK() {
    L_ticks++;
}

void R_TICK() {
    R_ticks++;
}

// Update the encoder message in a brief critical section.
// Copy the tick counts and compute the delta time.
void updateEncoder() {
    uint8_t localL, localR;
    uint32_t current_us, delta_us;

    noInterrupts();
    localL = L_ticks;
    localR = R_ticks;
    // Reset tick counters for the next interval.
    L_ticks = 0;
    R_ticks = 0;
    current_us = micros();
    delta_us = current_us - last_encoder_update_us;
    last_encoder_update_us = current_us;
    interrupts();

    // According to the .msg definition (order matters):
    // right_ticks, left_ticks, right_time_delta, left_time_delta.
    encoder_msg.right_ticks = localR;
    encoder_msg.left_ticks = localL;
    encoder_msg.right_time_delta = delta_us;
    encoder_msg.left_time_delta = delta_us;

    Serial.print("[Encoders] updateEncoder: localL=");
    Serial.print(localL);
    Serial.print(" localR=");
    Serial.print(localR);
    Serial.print(" delta_us=");
    Serial.println(delta_us);
}

// Publish the encoder message if the publish interval has elapsed.
// A local copy is made (with interrupts briefly disabled) to avoid concurrent modification.
void publishEncoder() {
    if (millis() - last_encoder_publish_ms >= ENCODER_PUBLISH_INTERVAL_MS) {
        Serial.println("[Encoders] publishEncoder: starting publish sequence");

        updateEncoder();

        Serial.print("[Encoders] publishEncoder: local_encoder: right_ticks=");
        Serial.println(encoder_msg.right_ticks);
        encoder_pub.publish(&encoder_msg);
        last_encoder_publish_ms = millis();

        Serial.println("[Encoders] publishEncoder: publish complete");
    }
}

// Set up encoder pins and attach interrupts.
void setupEncoders() {
    last_encoder_update_us = micros(); // Initialize the timestamp.
    Serial.println("[Encoders] setupEncoders: configuring pins and interrupts");
    pinMode(20, INPUT_PULLUP);
    pinMode(21, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(20), L_TICK, CHANGE);
    attachInterrupt(digitalPinToInterrupt(21), R_TICK, CHANGE);
    Serial.println("[Encoders] setupEncoders: interrupts attached");
}

} // namespace Encoders
