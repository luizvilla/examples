/*
 * Copyright (c) 2021-present LAAS-CNRS
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
 * @brief  This example initializes the Twist power shield in Buck mode and
 *         enables the default measurements, without ever starting the PWM
 *         or driving a duty cycle. There is no power flow: it only reads
 *         back and logs the leg currents/voltages and the high side
 *         current/voltage.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

/*--------------OWNTECH APIs---------------------------------- */
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

/* Temporary storage fore measured value (critical task) */
static float meas_data;

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here the setup :
 *  - Initializes the power shield in Buck mode (PWM is never started,
 *    so there is no power flow)
 *  - Initializes the power shield sensors
 *  - Spawns two tasks.
 */
void setup_routine()
{
    /* Buck voltage mode, PWM output is never started */
    shield.power.initBuck(ALL);

    shield.sensors.enableDefaultTwistSensors();

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startCritical();
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the code loop of the background task
 * This task logs back measurements to the USB serial interface.
 */
void loop_application_task()
{
    printk("%.3f:", (double)I1_low_value);
    printk("%.3f:", (double)V1_low_value);
    printk("%.3f:", (double)I2_low_value);
    printk("%.3f:", (double)V2_low_value);
    printk("%.3f:", (double)I_high);
    printk("%.3f:", (double)V_high);
    printk("\n");

    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensors values
 */
void loop_critical_task()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;
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
