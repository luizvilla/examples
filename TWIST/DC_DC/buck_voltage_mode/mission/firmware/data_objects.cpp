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

THINGSET_ADD_GROUP(ID_ROOT, ID_CONTROL, "Control", THINGSET_NO_CALLBACK);
/* 0 idle (PWM off), 1 power (PWM on, PI running) */
THINGSET_ADD_ITEM_UINT8(ID_CONTROL, ID_CONTROL_MODE, "Mode", &mode, THINGSET_ANY_RW, SUBSET_SER);
/* Output voltage reference of the PI */
THINGSET_ADD_ITEM_FLOAT(ID_CONTROL, ID_CONTROL_VOLTAGE_REF, "wVoltageRef_V", &voltage_reference, 2, THINGSET_ANY_RW, SUBSET_SER);

#endif /* CONFIG_THINGSET */
