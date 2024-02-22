#include "svea_msgs/lli_encoder.h"
#include <Arduino.h>

namespace Encoders {
svea_msgs::lli_encoder encoder_msg;

#define ENCODER_L_1 20
#define ENCODER_L_2 21
#define ENCODER_R_1 22
#define ENCODER_R_2 23

static unsigned long last_L_interrupt_time = 0;
static unsigned long last_R_interrupt_time = 0;
static uint8_t debouce_Time = 10;

void R_TICK() {
    unsigned long R_interrupt_time = millis();
    if (R_interrupt_time - last_R_interrupt_time > debouce_Time) {
        // Do your thing
        noInterrupts();
        encoder_msg.right_ticks = encoder_msg.right_time_delta + 1;
        interrupts();
        encoder_msg.right_time_delta = micros() - encoder_msg.right_time_delta;
    }
    last_R_interrupt_time = R_interrupt_time;
}

void L_TICK() {
    unsigned long L_interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (L_interrupt_time - last_L_interrupt_time > debouce_Time) {
        // Do your thing
        noInterrupts();
        encoder_msg.left_ticks = encoder_msg.left_time_delta + 1;
        interrupts();
        encoder_msg.left_time_delta = micros() - encoder_msg.left_time_delta;
    }
    last_L_interrupt_time = L_interrupt_time;
}

void setupEncoders() {
    /* Same thing for the encoder pins */
    pinMode(ENCODER_L_1, OUTPUT);
    pinMode(ENCODER_R_1, OUTPUT);
    digitalWrite(ENCODER_L_1, LOW);
    digitalWrite(ENCODER_R_1, LOW);
    pinMode(ENCODER_L_2, INPUT_PULLUP);
    pinMode(ENCODER_R_2, INPUT_PULLUP);

    // Settings for pin change interrupts for detecting wheel encoder ticks
    attachInterrupt(ENCODER_L_2, L_TICK, CHANGE);
    attachInterrupt(ENCODER_R_2, R_TICK, CHANGE);
}

} // namespace Encoders