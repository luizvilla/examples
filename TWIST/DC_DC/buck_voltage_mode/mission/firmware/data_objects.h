/*
 * GENERATED from data_objects.yaml by tools/gen_data_objects.py.
 * Do not edit: change data_objects.yaml and run the generator again.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ID_ROOT 0x00
#define SUBSET_SER (1U << 0)

/* Measurements */
#define ID_MEASUREMENTS 0x05
#define ID_MEASUREMENTS_V1_LOW 0x50
#define ID_MEASUREMENTS_V2_LOW 0x51
#define ID_MEASUREMENTS_VHIGH 0x52
#define ID_MEASUREMENTS_I1_LOW 0x53
#define ID_MEASUREMENTS_I2_LOW 0x54
#define ID_MEASUREMENTS_IHIGH 0x55
#define ID_MEASUREMENTS_TEMP1 0x56
#define ID_MEASUREMENTS_TEMP2 0x57
#define ID_MEASUREMENTS_DUTY 0x5A

/* Control */
#define ID_CONTROL 0x06
#define ID_CONTROL_MODE 0x61
#define ID_CONTROL_VOLTAGE_REF 0x62

/* Build */
#define ID_BUILD 0x07
#define ID_BUILD_COMMIT 0x70
#define ID_BUILD_DIRTY 0x71
#define ID_BUILD_SOURCE_HASH 0x72

/* Variables exposed above; defined in main.cpp (Build: build_identity.cpp). */
extern float V1_low_value;
extern float V2_low_value;
extern float V_high;
extern float I1_low_value;
extern float I2_low_value;
extern float I_high;
extern float temp_1_value;
extern float temp_2_value;
extern float duty_cycle;
extern uint8_t mode;
extern float voltage_reference;
extern char build_commit[41];
extern bool build_dirty;
extern char build_source_hash[17];
