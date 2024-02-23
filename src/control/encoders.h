#include "svea_msgs/lli_encoder.h"
#include <Arduino.h>

namespace Encoders {
svea_msgs::lli_encoder encoder_msg;

#define ENCODER_L_1 20
#define ENCODER_L_2 21
#define ENCODER_R_1 22
#define ENCODER_R_2 23

static uint32_t last_L_interrupt_time = 0;
static uint32_t last_R_interrupt_time = 0;

static uint32_t curr_L_interrupt_time = 0;
static uint32_t curr_R_interrupt_time = 0;

static uint32_t last_L_publish_time = 0;
static uint32_t last_R_publish_time = 0;

static uint32_t last_L_tick = 0;
static uint32_t last_R_tick = 0;

static uint32_t curr_L_tick = 0;
static uint32_t curr_R_tick = 0;

// This variable is probably important for the debouncing of the encoder ticks, might get crazy values otherwise
// We have measured down to CA 1ms, but this introduces noise
// 100 comes from trial and error
static uint32_t debouce_Time = 100;

void R_TICK() {
    noInterrupts();
    unsigned long R_interrupt_time = micros();
    if (R_interrupt_time - last_R_interrupt_time > debouce_Time) {
        curr_R_tick++;
        last_R_interrupt_time = R_interrupt_time;
        // Serial.println("Encoder_msg.right_time_delta: " + String(encoder_msg.right_time_delta / 1e3) + "us");
        // Serial.println("R ticks: " + String(encoder_msg.right_ticks));
    }
    interrupts();
}

void L_TICK() {
    noInterrupts();
    unsigned long L_interrupt_time = micros();
    if (L_interrupt_time - last_L_interrupt_time > debouce_Time) {
        curr_L_tick++;
        last_L_interrupt_time = L_interrupt_time;
    }
    interrupts();
}

svea_msgs::lli_encoder process_encoder(){
    noInterrupts();
    svea_msgs::lli_encoder encoder_msg;
    encoder_msg.right_ticks = curr_R_tick - last_R_tick;
    encoder_msg.left_ticks = curr_L_tick - last_L_tick;
    uint32_t current_time = micros();
    encoder_msg.right_time_delta = current_time - last_R_publish_time;
    encoder_msg.left_time_delta = current_time - last_L_publish_time;
    last_R_publish_time = current_time;
    last_L_publish_time = current_time;
    last_R_tick = curr_R_tick;
    last_L_tick = curr_L_tick;
    interrupts();
    return encoder_msg;
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
    attachInterrupt(ENCODER_L_2, L_TICK, RISING);
    attachInterrupt(ENCODER_R_2, R_TICK, RISING);
}

} // namespace Encoders