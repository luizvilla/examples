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
 * @brief  This example demonstrates how to deploy a Buck converter with
 *         voltage mode control on the Twist power shield.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "control.h"

/*--------------Data objects (ThingSet)------------------------ */
/* Generated from data_objects.yaml: the variables below marked
 * "data object" are read and written by the computer over ThingSet,
 * on the board's second USB serial port (see app.overlay). */
#include "data_objects.h"

/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

/* Measure variables (data objects, read-only) */

float32_t V1_low_value;
float32_t V2_low_value;
float32_t I1_low_value;
float32_t I2_low_value;
float32_t I_high;
float32_t V_high;

float32_t temp_1_value;
float32_t temp_2_value;

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

/* Duty cycle applied to both legs (data object, read-only) */
float32_t duty_cycle = 0.3;

/* Voltage reference (data object, writable) */
float32_t voltage_reference = 15;

/* Control law period, in seconds (the PI itself lives in control.cpp) */
static float32_t Ts = control_task_period * 1e-6;

/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum converter_mode
{
    IDLEMODE = 0,
    POWERMODE
};

/* Converter mode (data object, writable). Any value other than POWERMODE
 * is treated as IDLEMODE, so an invalid write turns the PWM off. */
uint8_t mode = IDLEMODE;

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here the setup :
 *  - Initializes the power shield in Buck mode
 *  - Initializes the power shield sensors
 *  - Initializes the PID controller
 *  - Spawns three tasks.
 */
void setup_routine()
{
    /* Buck voltage mode */
    shield.power.initBuck(ALL);

    shield.sensors.enableDefaultTwistSensors();

    control_init(Ts);

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
 * It shows the mode on the LED and measures temperatures. Measurements reach
 * the computer as ThingSet data objects, so nothing is printed here.
 */
void loop_application_task()
{
    if (mode == POWERMODE)
    {
        spin.led.turnOn();
    }
    else
    {
        spin.led.turnOff();
    }

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);
    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);

    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
    if (meas_data != NO_VALUE) temp_1_value = meas_data;

    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
    if (meas_data != NO_VALUE) temp_2_value = meas_data;

    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensors values
 *  - It runs the PID controller
 *  - It update the PWM signals
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

    if (mode != POWERMODE)
    {
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
        }
        pwm_enable = false;
    }
    else
    {
        Measurements m = {V1_low_value, I1_low_value, V2_low_value,
                          I2_low_value, V_high, I_high};
        duty_cycle = control_step(voltage_reference, m);
        shield.power.setDutyCycle(ALL,duty_cycle);

        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(ALL);
        }
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
