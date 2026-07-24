/*
 *
 * Copyright (c) 2024-present LAAS-CNRS
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
 * @brief  This file is the main application code for the open-loop inverter
 *         example. It implements a sinusoidal open-loop PWM strategy that
 *         drives an H-bridge inverter using a local oscillator without any
 *         closed-loop feedback or grid synchronization.
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

 /*--------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

/* From control library */
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F

/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the slow communication task */
void loop_communication_task();
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/*--------------SUPPORT FUNCTIONS DECLARATION----------------- */
/* Reports whether the scope capture should trigger */
bool a_trigger();
/* Clamps a floating-point value inside the requested range */
float32_t saturate(float32_t value, float32_t min, float32_t max);
/* Returns the DC bus voltage, or a fallback before sensing is valid */
float32_t control_bus_voltage();
/* Starts both PWM legs once and records the output state */
void start_pwm_outputs();
/* Stops both PWM legs once and records the output state */
void stop_pwm_outputs();
/* Advances the local oscillator and updates the sine voltage reference */
void update_teaching_sine();
/* Applies clamped complementary duty cycles to the H-bridge legs */
void apply_complementary_duty(float32_t duty);
/* Streams a completed ScopeMimicry capture over the serial console */
void dump_scope_datas(ScopeMimicry &scope_to_dump);
/* Adjusts the local sine voltage amplitude used by the open-loop PWM */
void adjust_amplitude(float32_t step);
/* Registers scope channels and starts capture for the open-loop example */
void setup_scope();
/* Reads sensor values and derives the measured grid voltage */
void read_measurements();
/* Checks both measured currents against the protection threshold */
bool overcurrent_detected();

/*--------------USER VARIABLES DECLARATIONS------------------- */

enum ConverterState : uint8_t /* Holds the current state of the inverter */
{
    IDLEMODE = 0,  /* Idle mode: stops the converter power */
    POWERMODE = 1, /* Power mode: drives the H-bridge with sine PWM */
    ERRORMODE = 3  /* Error mode: indicates an error condition */
};

/* Control task period in microseconds */
static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
/* Control task period in seconds */
static constexpr float32_t TS = CONTROL_TASK_PERIOD_US * 1.0e-6F;
/* Fallback DC bus voltage in volts before sensing is valid */
static constexpr float32_t DC_BUS_FALLBACK = 20.0F;
/* Grid frequency in hertz */
static constexpr float32_t F0 = 50.0F;
/* Grid pulsation in radians per second */
static constexpr float32_t W0 = 2.0F * PI * F0;
/* Maximum current for overcurrent protection in amps */
static constexpr float32_t MAX_CURRENT = 8.0F;
/* Size of the scope buffer for data recording */
static constexpr uint32_t SCOPE_BUFFER_SIZE = 1024;
/* Number of channels recorded in the scope for diagnostics */
static constexpr uint8_t SCOPE_CHANNEL_COUNT = 10;

/* State of the PWM outputs */
static bool pwm_enable = false;
/* State of scope data downloading */
static bool is_downloading = false;
/* State to trigger the scope capture */
static bool trigger = false;

/* Variable to store the last received serial character */
static uint8_t received_serial_char;
/* Current operating mode of the inverter */
static uint8_t mode = IDLEMODE;
/* Last requested operating mode from the serial interface */
static uint8_t mode_asked = IDLEMODE;

/* [V] Low-side filtered measurement of voltage 1 */
static float32_t V1_low_value;
/* [V] Low-side filtered measurement of voltage 2 */
static float32_t V2_low_value;
/* [A] Low-side filtered measurement of current 1 */
static float32_t I1_low_value;
/* [A] Low-side filtered measurement of current 2 */
static float32_t I2_low_value;
/* [V] High-side raw measurement of voltage */
static float32_t V_high;
/* [V] Further filtered high-side voltage for control */
static float32_t V_high_filt;
/* [V] Measured grid voltage from the difference of low-side measurements */
static float32_t Vgrid_meas;
/* [V] Temporary variable for storing sensor measurements */
static float32_t meas_data;

/* [V] Amplitude of the local teaching sine wave */
static float32_t local_voltage_amplitude = 20.0F;
/* [rad] Phase angle of the local teaching sine wave */
static float32_t teaching_theta;
/* [No unit] Instantaneous value of the local teaching sine wave */
static float32_t sine;
/* [V] Instantaneous local grid voltage from the teaching sine */
static float32_t local_vgrid;

/* [No unit] Duty cycle computed from the local sine and DC bus voltage */
static float32_t delta_duty_cycle = 0.5F;
/* [No unit] Duty cycle for leg 1 of the H-bridge */
static float32_t duty_cycle_1 = 0.5F;
/* [No unit] Duty cycle for leg 2 of the H-bridge */
static float32_t duty_cycle_2 = 0.5F;
/* [No unit] Scope variable for the current operating mode */
static float32_t state_mode_scope;
/* Counter for the number of critical task iterations */
static uint32_t critical_task_counter;

/* First-order low-pass filter for the high-side voltage measurement */
static LowPassFirstOrderFilter vHighFilter(TS, 0.1F);
/* ScopeMimicry instance for recording control variables and diagnostics */
static ScopeMimicry scope(SCOPE_BUFFER_SIZE, SCOPE_CHANNEL_COUNT);
/*--------------------------------------------------------------- */

/**********************  SUPPORT FUNCTIONS  ***************************/

/**
 * @brief Reports whether the scope capture should trigger.
 */
bool a_trigger()
{
    return trigger;
}

/**
 * @brief Clamps a floating-point value inside the requested range.
 */
float32_t saturate(float32_t value, float32_t min, float32_t max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

/**
 * @brief Returns the measured DC bus voltage, or a fallback before sensing is valid.
 */
float32_t control_bus_voltage()
{
    if (V_high_filt > 1.0F) {
        return V_high_filt;
    }
    return DC_BUS_FALLBACK;
}

/**
 * @brief Starts both PWM legs once and records the output state.
 */
void start_pwm_outputs()
{
    if (!pwm_enable) {
        shield.power.start(ALL);
        pwm_enable = true;
    }
}

/**
 * @brief Stops both PWM legs once and records the output state.
 */
void stop_pwm_outputs()
{
    if (pwm_enable) {
        shield.power.stop(ALL);
        pwm_enable = false;
    }
}

/**
 * @brief Advances the local teaching oscillator and updates the sine voltage reference.
 */
void update_teaching_sine()
{
    teaching_theta = ot_modulo_2pi(teaching_theta + W0 * TS);
    sine = ot_sin(teaching_theta);
    local_vgrid = local_voltage_amplitude * sine;
}

/**
 * @brief Applies clamped complementary duty cycles to the H-bridge legs.
 */
void apply_complementary_duty(float32_t duty)
{
    duty_cycle_1 = saturate(duty, DUTY_MIN, DUTY_MAX);
    duty_cycle_2 = saturate(1.0F - duty, DUTY_MIN, DUTY_MAX);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
}

/**
 * @brief Streams a completed ScopeMimicry capture over the serial console.
 */
void dump_scope_datas(ScopeMimicry &scope_to_dump)
{
    scope_to_dump.reset_dump();
    printk("begin record\n");
    while (scope_to_dump.get_dump_state() != finished) {
        printk("%s", scope_to_dump.dump_datas());
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

/**
 * @brief Adjusts the local sine voltage amplitude used by the open-loop PWM.
 */
void adjust_amplitude(float32_t step)
{
    local_voltage_amplitude =
        saturate(local_voltage_amplitude + step, 0.0F, 30.0F);
}

/**
 * @brief Registers scope channels and starts capture for the open-loop example.
 */
void setup_scope()
{
    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(Vgrid_meas, "Vgrid");
    scope.connectChannel(V_high_filt, "Vdc");
    scope.connectChannel(local_vgrid, "local_vgrid");
    scope.connectChannel(sine, "sine");
    scope.connectChannel(delta_duty_cycle, "duty_cycle");
    scope.connectChannel(duty_cycle_1, "duty_cycle_1");
    scope.connectChannel(duty_cycle_2, "duty_cycle_2");
    scope.connectChannel(state_mode_scope, "state");
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();
}

/**
 * @brief Reads the latest sensor values and derives the measured grid voltage.
 */
void read_measurements()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

    V_high_filt = vHighFilter.calculateWithReturn(V_high);
    Vgrid_meas = V1_low_value - V2_low_value;
}

/**
 * @brief Checks both measured currents against the protection threshold.
 */
bool overcurrent_detected()
{
    return I1_low_value > MAX_CURRENT ||
           I1_low_value < -MAX_CURRENT ||
           I2_low_value > MAX_CURRENT ||
           I2_low_value < -MAX_CURRENT;
}

/*----------------------- END OF SUPPORT FUNCTIONS ------------------------- */

/*----------------------- MAIN APPLICATION CODE ---------------------------- */

/**
 * @brief Configures hardware, scope capture, and OwnTech task scheduling.
 */
void setup_routine()
{
    spin.pwm.initFixedFrequency(50000);
    shield.power.setDeadTime(LEG1, 20, 20);
    shield.power.setDeadTime(LEG2, 20, 20);
    shield.sensors.enableDefaultTwistSensors();
    shield.power.connectCapacitor(LEG1);
    shield.power.connectCapacitor(LEG2);
    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);

    setup_scope();

    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/**
 * @brief Handles serial commands for mode changes, tuning, and scope capture.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char) {
    case 'h':
        printk(" ________________________________________\n");
        printk("|     open-loop sine PWM                 |\n");
        printk("|     i : idle                           |\n");
        printk("|     p : power                          |\n");
        printk("|     u/j : amplitude +/- 1 V            |\n");
        printk("|     d/c : amplitude +/- 5 V            |\n");
        printk("|     r : retrieve scope data            |\n");
        printk("|     t : trigger scope data             |\n");
        printk("|________________________________________|\n\n");
        break;
    case 'i':
        mode_asked = IDLEMODE;
        break;
    case 'p':
        if (!is_downloading) {
            scope.start();
            mode_asked = POWERMODE;
        }
        break;
    case 'u':
        adjust_amplitude(1.0F);
        break;
    case 'j':
        adjust_amplitude(-1.0F);
        break;
    case 'd':
        adjust_amplitude(5.0F);
        break;
    case 'c':
        adjust_amplitude(-5.0F);
        break;
    case 'r':
        is_downloading = true;
        trigger = false;
        break;
    case 't':
        trigger = true;
        break;
    default:
        break;
    }
}

/**
 * @brief Runs the low-rate state machine and status reporting.
 */
void loop_application_task()
{
    switch (mode) {
    case IDLEMODE:
        if (mode_asked == POWERMODE) {
            mode = POWERMODE;
        }
        spin.led.turnOn();
        break;
    case POWERMODE:
        if (mode_asked == IDLEMODE) {
            mode = IDLEMODE;
        }
        spin.led.toggle();
        break;
    case ERRORMODE:
        break;
    default:
        mode = ERRORMODE;
        break;
    }

    if (mode_asked == IDLEMODE) {
        mode = IDLEMODE;
    }

    if (is_downloading) {
        dump_scope_datas(scope);
        is_downloading = false;
    } else {
        printk("%d:", mode);
        printk("%.2f:", static_cast<double>(V_high_filt));
        printk("%.2f:", static_cast<double>(Vgrid_meas));
        printk("%.2f:", static_cast<double>(local_vgrid));
        printk("%.2f:", static_cast<double>(local_voltage_amplitude));
        printk("%.3f:", static_cast<double>(duty_cycle_1));
        printk("%.3f:", static_cast<double>(duty_cycle_2));
        printk("\n");
    }

    task.suspendBackgroundMs(100);
}

/**
 * @brief Runs the 10 kHz control loop, protection checks, and PWM updates.
 */
void loop_critical_task()
{
    critical_task_counter++;
    read_measurements();
    update_teaching_sine();

    if (overcurrent_detected()) {
        mode = ERRORMODE;
    }

    if (mode == POWERMODE) {
        delta_duty_cycle = 0.5F + local_vgrid / (2.0F * control_bus_voltage());
        apply_complementary_duty(delta_duty_cycle);
        start_pwm_outputs();
    } else {
        stop_pwm_outputs();
        spin.led.turnOff();
    }

    state_mode_scope = static_cast<float32_t>(mode);
    scope.acquire();
}

/**
 * @brief Application entry point.
 */
int main(void)
{
    setup_routine();
    return 0;
}
