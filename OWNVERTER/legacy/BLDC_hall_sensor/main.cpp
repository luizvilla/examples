/*
 * Copyright (c) 2024-present OwnTech Foundation
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file is an application example for
 *         BLDC motor control using OwnVerter.
 *
 * @author Régis Ruelland <regis.ruelland@laas.fr>
 * @author Jean Alinei <jean.alinei@owntech.org>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "ScopeMimicry.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

#include "zephyr/console/console.h"

#define HALL1 PC6
#define HALL2 PC7
#define HALL3 PD2

#define PHASE_A LEG1
#define PHASE_B LEG2
#define PHASE_C LEG3

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in the application task */
void application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

/* [bool] state of the PWM (control task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;
uint16_t angle_index;
uint8_t HALL1_value;
uint8_t HALL2_value;
uint8_t HALL3_value;

/* Temporary storage for measured value (control task) */
static float meas_data;

float32_t duty_cycle = 0.5;

/* Variables mirrored as float values for ScopeMimicry */
static float32_t angle_index_f;
static float32_t HALL1_value_f;
static float32_t HALL2_value_f;
static float32_t HALL3_value_f;
static float32_t mode_f;

/* Decimation is used to limit the plotted measurement rate */
const static uint32_t decimation = 10;
static uint32_t counter_time;

/* ScopeMimicry capture state */
const uint16_t SCOPE_SIZE = 512;
uint16_t k_app_idx;
ScopeMimicry scope(SCOPE_SIZE, 10);
static bool is_downloading;
static bool memory_print;

void chopper(leg_t high_phase, leg_t low_phase)
{
	shield.power.setDutyCycle(low_phase, 1.0 - duty_cycle);
	shield.power.start(low_phase);
	shield.power.setDutyCycle(high_phase, duty_cycle);
	shield.power.start(high_phase);
}

/* --------------------------------------------------------------- */

/* List of possible modes for the OwnTech board */
enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE
};

uint8_t mode = IDLEMODE;

bool mytrigger()
{
	return (mode == POWERMODE);
}

void dump_scope_datas(ScopeMimicry &scope)
{
	printk("begin record\n");
	scope.reset_dump();
	while (scope.get_dump_state() != finished) {
		printk("%s", scope.dump_datas());
		task.suspendBackgroundUs(200);
	}
	printk("end record\n");
}

/* --------------SETUP FUNCTIONS-------------------------------*/

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin,
 * your power shield, data and/or tasks.
 *
 * In this example :
 * We setup the OwnVerter power shield in buck mode.
 * We enable its defaults sensors.
 * We create a critical task running at 10kHz.
 * We configure the GPIOS needed for measuring the position using hall sensors.
 */
void setup_routine()
{

	/* Set the high switch convention for all legs */
	shield.power.initBuck(ALL);

	/* Setup all the measurements */
	shield.sensors.enableDefaultOwnverterSensors();

	/* Declare tasks */
	uint32_t background_task_number = task.createBackground(loop_background_task);
	uint32_t app_task_number = task.createBackground(application_task);
	task.createCritical(loop_critical_task, 100);

	/* Start tasks */
	task.startBackground(background_task_number);
	task.startBackground(app_task_number);
	task.startCritical();

	spin.gpio.configurePin(HALL1, INPUT);
	spin.gpio.configurePin(HALL2, INPUT);
	spin.gpio.configurePin(HALL3, INPUT);

	/* Scope configuration */
	scope.connectChannel(duty_cycle, "duty_cycle");       /* 0 */
	scope.connectChannel(I1_low_value, "I1_low_value");   /* 1 */
	scope.connectChannel(I2_low_value, "I2_low_value");   /* 2 */
	scope.connectChannel(I_high, "I_high");               /* 3 */
	scope.connectChannel(V1_low_value, "V1_low_value");   /* 4 */
	scope.connectChannel(V2_low_value, "V2_low_value");   /* 5 */
	scope.connectChannel(V_high, "V_high");               /* 6 */
	scope.connectChannel(angle_index_f, "angle_index");   /* 7 */
	scope.connectChannel(HALL1_value_f, "HALL1_value");   /* 8 */
	scope.connectChannel(mode_f, "mode");                 /* 9 */
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0);
	scope.start();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This background task handles USB serial commands.
 *
 * In this example a simple duty cycle control is implemented:
 * - When pressing U and D keys, we increase or decrease the duty cycle.
 */
void loop_background_task()
{
	while (1) {
		received_serial_char = console_getchar();
		switch (received_serial_char) {
		case 'h':
			/* ----------SERIAL INTERFACE MENU----------------------- */

			printk(" ________________________________________ \n"
				   "|     ------- MENU ---------             |\n"
				   "|     press i : idle mode                |\n"
				   "|     press p : power mode               |\n"
				   "|     press u : duty cycle UP            |\n"
				   "|     press d : duty cycle DOWN          |\n"
				   "|     press r : dump scope capture       |\n"
				   "|     press q : restart scope capture    |\n"
				   "|     press m : replay scope memory      |\n"
				   "|________________________________________|\n\n");

			/* ------------------------------------------------------ */
			break;
		case 'i':
			printk("idle mode\n");
			duty_cycle = 0.5;
			mode = IDLEMODE;
			break;
		case 'p':
			printk("power mode\n");
			mode = POWERMODE;
			scope.start();
			break;
		case 'u':
			duty_cycle += 0.01;
			break;
		case 'd':
			duty_cycle -= 0.01;
			break;
		case 'r':
			is_downloading = true;
			break;
		case 'q':
			scope.start();
			break;
		case 'm':
			memory_print = !memory_print;
			break;
		default:
			break;
		}
	}
}

/**
 * This application task sends back measurements through USB serial.
 */
void application_task()
{
	if (memory_print) {
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.2f:", scope.get_channel_value(k_app_idx, 0));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 1));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 2));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 3));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 4));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 5));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 6));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 7));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 8));
		printk("%.2f\n", scope.get_channel_value(k_app_idx, 9));
	} else if (mode == IDLEMODE) {
		spin.led.turnOff();
		printk("%f:", V1_low_value);
		printk("%f:", I2_low_value);
		printk("%f:", V2_low_value);
		printk("%f:", I_high);
		printk("%f", V_high);
		printk("%5d\n", angle_index);

	} else if (mode == POWERMODE) {
		spin.led.turnOn();
		printk("%5.5f:", duty_cycle);
		printk("%f:", V1_low_value);
		printk("%f:", I2_low_value);
		printk("%f:", V2_low_value);
		printk("%f:", I_high);
		printk("%f", V_high);
		printk("%5d\n", angle_index);
	}

	if (is_downloading) {
		dump_scope_datas(scope);
		is_downloading = false;
	}

	task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task. It is executed
 * every 100 micro-seconds as defined in the setup_software function.
 *
 * In this example :
 * - Individual Hall sensors state are read
 * - Hall State is computed
 * - Measurements are retrieved
 * - In power mode, a switch case is launched implementing 6 step logic.
 */
void loop_critical_task()
{
	counter_time++;

	/* Retrieve the rotor position from hall sensors */
	HALL1_value = spin.gpio.readPin(HALL1);
	HALL2_value = spin.gpio.readPin(HALL2);
	HALL3_value = spin.gpio.readPin(HALL3);

	/* Compute the sector from hall values */
	angle_index = HALL1_value + 2 * HALL2_value + 4 * HALL3_value;

	/* Retrieve sensor values */
	meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (meas_data != NO_VALUE) {
		I1_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V1_LOW);
	if (meas_data != NO_VALUE) {
		V1_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V2_LOW);
	if (meas_data != NO_VALUE) {
		V2_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (meas_data != NO_VALUE) {
		I2_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (meas_data != NO_VALUE) {
		I_high = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_high = meas_data;
	}

	if (mode == IDLEMODE) {
		if (pwm_enable == true) {
			shield.power.stop(ALL);
		}
		pwm_enable = false;
	} else if (mode == POWERMODE) {
		switch (angle_index) {

		/* This switch case implements classic BLDC logic */

		case 0b001:
			shield.power.stop(PHASE_B);
			chopper(PHASE_A, PHASE_C);
			break;
		case 0b010:
			shield.power.stop(PHASE_C);
			chopper(PHASE_B, PHASE_A);
			break;
		case 0b011:
			shield.power.stop(PHASE_A);
			chopper(PHASE_B, PHASE_C);
			break;
		case 0b100:
			shield.power.stop(PHASE_A);
			chopper(PHASE_C, PHASE_B);
			break;
		case 0b101:
			shield.power.stop(PHASE_C);
			chopper(PHASE_A, PHASE_B);
			break;
		case 0b110:
			shield.power.stop(PHASE_B);
			chopper(PHASE_C, PHASE_A);
			break;
		}

		/* Set POWER ON */
		if (!pwm_enable) {
			pwm_enable = true;
			shield.power.start(ALL);
		}
	}

	if (counter_time % decimation == 0) {
		angle_index_f = angle_index;
		HALL1_value_f = HALL1_value;
		HALL2_value_f = HALL2_value;
		HALL3_value_f = HALL3_value;
		mode_f = mode;
		scope.acquire();
	}
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
	setup_routine();
	return 0;
}

