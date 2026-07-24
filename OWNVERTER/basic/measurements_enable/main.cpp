/*
 * Copyright (c) 2026-present OwnTech Foundation
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This example initializes the OwnVerter power shield in Buck mode
 *         and enables the default measurements, without ever starting the
 *         PWM or driving a duty cycle. There is no power flow: it only
 *         reads back and logs the leg currents/voltages, the high side
 *         current/voltage, and the three leg temperatures.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

#include "ShieldAPI.h"
#include "TaskAPI.h"

void setup_routine();
void loop_application_task();
void loop_critical_task();

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t V3_low_value;
static float32_t V_high;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I3_low_value;
static float32_t I_high;
static float32_t meas_data;

/* OwnVerter multiplexes its three leg temperatures on a single ADC
 * channel: only one of TEMP_1/TEMP_2/TEMP_3 can be selected at a time,
 * so it takes several background task iterations to cycle through all
 * three and let the analog mux settle between switches. */
static float32_t T1_value;
static float32_t T2_value;
static float32_t T3_value;
static uint32_t temp_counter;
static const uint32_t TEMP_SWITCH_PERIOD = 5;

void retrieve_measurements()
{
	meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (meas_data != NO_VALUE) {
		I1_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (meas_data != NO_VALUE) {
		I2_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I3_LOW);
	if (meas_data != NO_VALUE) {
		I3_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (meas_data != NO_VALUE) {
		I_high = -meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V1_LOW);
	if (meas_data != NO_VALUE) {
		V1_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V2_LOW);
	if (meas_data != NO_VALUE) {
		V2_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V3_LOW);
	if (meas_data != NO_VALUE) {
		V3_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_high = meas_data;
	}
}

/* Reads back the temperature selected on a previous call, then moves the
 * mux on to the next one. Called once per background task iteration. */
void retrieve_temperatures()
{
	meas_data = shield.sensors.getLatestValue(TEMP_SENSOR);

	temp_counter++;
	if (temp_counter == TEMP_SWITCH_PERIOD) {
		if (meas_data != NO_VALUE) {
			T3_value = meas_data;
		}
		shield.sensors.setOwnverterTempMeas(TEMP_1);
	} else if (temp_counter == 2 * TEMP_SWITCH_PERIOD) {
		if (meas_data != NO_VALUE) {
			T1_value = meas_data;
		}
		shield.sensors.setOwnverterTempMeas(TEMP_2);
	} else if (temp_counter == 3 * TEMP_SWITCH_PERIOD) {
		if (meas_data != NO_VALUE) {
			T2_value = meas_data;
		}
		shield.sensors.setOwnverterTempMeas(TEMP_3);
		temp_counter = 0;
	}
}

void setup_routine()
{
	shield.power.initBuck(ALL);
	shield.sensors.enableDefaultOwnverterSensors();

	uint32_t application_task_number = task.createBackground(loop_application_task);
	task.createCritical(loop_critical_task, 100);

	task.startBackground(application_task_number);
	task.startCritical();
}

void loop_application_task()
{
	retrieve_temperatures();

	printk("Vhigh:%.2f Ihigh:%.2f V1:%.2f V2:%.2f V3:%.2f I1:%.2f I2:%.2f I3:%.2f T1:%.2f T2:%.2f T3:%.2f\n",
	       (double)V_high,
	       (double)I_high,
	       (double)V1_low_value,
	       (double)V2_low_value,
	       (double)V3_low_value,
	       (double)I1_low_value,
	       (double)I2_low_value,
	       (double)I3_low_value,
	       (double)T1_value,
	       (double)T2_value,
	       (double)T3_value);

	task.suspendBackgroundMs(200);
}

void loop_critical_task()
{
	retrieve_measurements();
}

int main(void)
{
	setup_routine();
	return 0;
}
