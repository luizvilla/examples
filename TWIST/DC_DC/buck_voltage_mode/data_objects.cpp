/*
 * GENERATED from data_objects.yaml by tools/gen_data_objects.py.
 * Do not edit: change data_objects.yaml and run the generator again.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "data_objects.h"

/* Compiled only when app.conf enables ThingSet (CONFIG_THINGSET=y). */
#if defined(CONFIG_THINGSET)

#include <thingset.h>

THINGSET_ADD_GROUP(ID_ROOT, ID_MEASUREMENTS, "Measurements", THINGSET_NO_CALLBACK);
/* Low-side voltage of leg 1 */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_V1_LOW, "rV1Low_V", &V1_low_value, 2, THINGSET_ANY_R, SUBSET_SER);
/* Low-side voltage of leg 2 */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_V2_LOW, "rV2Low_V", &V2_low_value, 2, THINGSET_ANY_R, SUBSET_SER);
/* High-side voltage */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_VHIGH, "rVHigh_V", &V_high, 2, THINGSET_ANY_R, SUBSET_SER);
/* Low-side current of leg 1 */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_I1_LOW, "rI1Low_A", &I1_low_value, 2, THINGSET_ANY_R, SUBSET_SER);
/* Low-side current of leg 2 */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_I2_LOW, "rI2Low_A", &I2_low_value, 2, THINGSET_ANY_R, SUBSET_SER);
/* High-side current */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_IHIGH, "rIHigh_A", &I_high, 2, THINGSET_ANY_R, SUBSET_SER);
/* Temperature of leg 1 */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_TEMP1, "rTemp1_degC", &temp_1_value, 1, THINGSET_ANY_R, SUBSET_SER);
/* Temperature of leg 2 */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_TEMP2, "rTemp2_degC", &temp_2_value, 1, THINGSET_ANY_R, SUBSET_SER);
/* Duty cycle applied to both legs, 0 to 1 */
THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_DUTY, "rDuty", &duty_cycle, 4, THINGSET_ANY_R, SUBSET_SER);

THINGSET_ADD_GROUP(ID_ROOT, ID_CONVERTER, "Converter", THINGSET_NO_CALLBACK);
/* 0 idle (PWM off), 1 power (PWM on, PI running) */
THINGSET_ADD_ITEM_UINT8(ID_CONVERTER, ID_CONVERTER_MODE, "Mode", &mode, THINGSET_ANY_RW, SUBSET_SER);
/* Output voltage reference of the PI */
THINGSET_ADD_ITEM_FLOAT(ID_CONVERTER, ID_CONVERTER_VOLTAGE_REF, "wVoltageRef_V", &voltage_reference, 2, THINGSET_ANY_RW, SUBSET_SER);

/* Called by ThingSet before a read and after a write of Scope (defined in the firmware). */
void scope_thingset_callback(enum thingset_callback_reason reason);
THINGSET_ADD_GROUP(ID_ROOT, ID_SCOPE, "Scope", scope_thingset_callback);
/* Write 1 to record from the next write to Converter, 2 to record now; reads back 0 */
THINGSET_ADD_ITEM_UINT8(ID_SCOPE, ID_SCOPE_ARM, "wArm", &scope_arm, THINGSET_ANY_RW, SUBSET_SER);
/* 0 idle, 1 armed, 2 recording, 3 ready to read */
THINGSET_ADD_ITEM_UINT8(ID_SCOPE, ID_SCOPE_STATE, "rState", &scope_state, THINGSET_ANY_R, SUBSET_SER);
/* Samples per channel */
THINGSET_ADD_ITEM_UINT16(ID_SCOPE, ID_SCOPE_LENGTH, "rLength", &scope_length, THINGSET_ANY_R, SUBSET_SER);
/* Samples recorded before the trigger */
THINGSET_ADD_ITEM_UINT16(ID_SCOPE, ID_SCOPE_PRETRIGGER, "rPretrigger", &scope_pretrigger, THINGSET_ANY_R, SUBSET_SER);
/* Time between two samples */
THINGSET_ADD_ITEM_FLOAT(ID_SCOPE, ID_SCOPE_PERIOD, "rPeriod_s", &scope_period, 6, THINGSET_ANY_R, SUBSET_SER);
/* Recorded data objects, comma-separated, in data order */
THINGSET_ADD_ITEM_STRING(ID_SCOPE, ID_SCOPE_CHANNELS, "rChannels", scope_channels, 64, THINGSET_ANY_R, SUBSET_SER);
/* Which 64 values rData holds: channel after channel, oldest sample first */
THINGSET_ADD_ITEM_UINT16(ID_SCOPE, ID_SCOPE_CHUNK, "wChunk", &scope_chunk, THINGSET_ANY_RW, SUBSET_SER);
/* 64 recorded values, selected by wChunk */
THINGSET_DEFINE_FLOAT_ARRAY(scope_data_ts, 3, scope_data, 64);
THINGSET_ADD_ITEM_ARRAY(ID_SCOPE, ID_SCOPE_DATA, "rData", &scope_data_ts, THINGSET_ANY_R, SUBSET_SER);

THINGSET_ADD_GROUP(ID_ROOT, ID_BUILD, "Build", THINGSET_NO_CALLBACK);
/* Git commit of the mission folder */
THINGSET_ADD_ITEM_STRING(ID_BUILD, ID_BUILD_COMMIT, "rCommit", build_commit, 41, THINGSET_ANY_R, SUBSET_SER);
/* True if the sources differed from the commit */
THINGSET_ADD_ITEM_BOOL(ID_BUILD, ID_BUILD_DIRTY, "rDirty", &build_dirty, THINGSET_ANY_R, SUBSET_SER);
/* Hash of firmware/ and data_objects.yaml, 16 hex digits */
THINGSET_ADD_ITEM_STRING(ID_BUILD, ID_BUILD_SOURCE_HASH, "rSourceHash", build_source_hash, 17, THINGSET_ANY_R, SUBSET_SER);

#endif /* CONFIG_THINGSET */
