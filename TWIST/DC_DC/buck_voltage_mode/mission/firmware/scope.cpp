/*
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  The firmware scope (see scope.h).
 *
 * The computer arms the scope (Scope/wArm = 1); the next write to Converter
 * (Mode or wVoltageRef_V) triggers it. wArm = 2 triggers at once. ScopeMimicry
 * keeps recording into its ring until the trigger, then records 80 % of the
 * window more and stops: the capture holds 20 % of samples from before the
 * trigger. The computer then reads the capture through wChunk and rData.
 */
#include "scope.h"

#include <stdio.h>
#include <string.h>

#include <ScopeMimicry.h>

#include "data_objects.h"

#if defined(CONFIG_THINGSET)
#include <thingset.h>
#endif

static const uint16_t LENGTH = 1024;
static const uint16_t CHANNELS = 5;
static const float PRETRIGGER = 0.2F;

static ScopeMimicry scope(LENGTH, CHANNELS);

/* Data objects of the Scope group */
uint8_t scope_arm = 0;
uint8_t scope_state = 0;
uint16_t scope_length = LENGTH;
uint16_t scope_pretrigger = 0;
float scope_period = 0;
char scope_channels[64] = "rV1Low_V,rI1Low_A,rVHigh_V,rDuty,wVoltageRef_V";
uint16_t scope_chunk = 0;
float scope_data[64];

enum { IDLE = 0, ARMED = 1, RECORDING = 2, READY = 3 };

/* What Converter held when the scope was armed: a change is the trigger. */
static volatile bool armed = false;
static volatile bool trigger_now = false;
static uint8_t armed_mode;
static float armed_reference;

static bool trigger()
{
    if (!armed) {
        return false;
    }
    return trigger_now || mode != armed_mode || voltage_reference != armed_reference;
}

void scope_init(float control_period_s)
{
    scope_period = control_period_s;
    scope.connectChannel(V1_low_value, "rV1Low_V");
    scope.connectChannel(I1_low_value, "rI1Low_A");
    scope.connectChannel(V_high, "rVHigh_V");
    scope.connectChannel(duty_cycle, "rDuty");
    scope.connectChannel(voltage_reference, "wVoltageRef_V");
    scope.set_trigger(&trigger);
    scope.set_delay(PRETRIGGER);
    /* ScopeMimicry's sample written in the trigger period lands one after its delay. */
    scope_pretrigger = (uint16_t)(PRETRIGGER * LENGTH) + 1;
}

void scope_sample()
{
    uint16_t result = scope.acquire();
    if (result == 2 && scope_state == RECORDING) {
        scope_state = READY;
        armed = false;
        trigger_now = false;
    } else if (result == 1 && scope_state == ARMED) {
        scope_state = RECORDING;
    }
}

/* Value `index` of the capture, channel after channel, oldest sample first. */
static float capture_value(uint32_t index)
{
    uint32_t channel = index / LENGTH;
    uint32_t sample = index % LENGTH;
    uint32_t oldest = (scope.get_final_idx() + 1U) % LENGTH;
    float *memory = (float *)scope.get_buffer();
    return memory[((oldest + sample) % LENGTH) * CHANNELS + channel];
}

#if defined(CONFIG_THINGSET)
void scope_thingset_callback(enum thingset_callback_reason reason)
{
    if (reason != THINGSET_CALLBACK_POST_WRITE) {
        return;
    }
    if (scope_arm == 1 || scope_arm == 2) {
        armed_mode = mode;
        armed_reference = voltage_reference;
        trigger_now = (scope_arm == 2);
        scope_state = ARMED;
        scope.start();
        armed = true;
    }
    scope_arm = 0;
    /* Fill rData for the chunk asked; values past the end read 0. */
    uint32_t first = (uint32_t)scope_chunk * 64U;
    for (uint32_t k = 0; k < 64U; k++) {
        uint32_t index = first + k;
        scope_data[k] = (scope_state == READY && index < (uint32_t)LENGTH * CHANNELS) ? capture_value(index) : 0.0F;
    }
}
#endif
