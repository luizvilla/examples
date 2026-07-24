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
 * @brief  This file is the main application code for the grid-following
 *         inverter example. It implements a single-phase grid-following control
 *         strategy using a PLL to synchronize with the grid and regulate the
 *         current injection.
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
#include "ThreePhaseInverter.h"

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

/*--------------USER ENUM DECLARATIONS-------------------- */

enum ConverterState : uint8_t /* Holds the current state of the inverter */
{
    IDLEMODE = 0,   /* Idle mode: stops the converter power */
    POWERMODE = 1,  /* Power mode: injects power into the grid */
    ERRORMODE = 3,  /* Error mode: indicates an error condition */
    STARTUPMODE = 4 /* Startup mode: initializes the converter */
};

/* Selects the input source for the PLL synchronization */
enum FollowingInputMode : uint8_t
{
    LOCAL_SINE_INPUT = 1,   /* Local teaching sine wave as PLL input */
    MEASURED_GRID_INPUT = 2 /* Measured grid voltage/current as PLL input */
};

/*--------------SUPPORT FUNCTIONS DECLARATION------------------- */
/* Returns whether the scope capture should trigger */
bool a_trigger();
/* Clamps a floating-point value inside the requested range */
float32_t saturate(float32_t value, float32_t min, float32_t max);
/* Returns the DC bus voltage, or a fallback before sensing is valid */
float32_t control_bus_voltage();
/* Clamps one duty-cycle command to the allowed modulation range */
float32_t clamp_duty(float32_t duty);
/* Starts both PWM legs once and records the output state */
void start_pwm_outputs();
/* Stops both PWM legs once and records the output state */
void stop_pwm_outputs();
/* Applies clamped complementary duty cycles to the H-bridge legs */
void apply_complementary_duty(three_phase_t duty);

/* Advances the local teaching oscillator and derives voltage/current inputs */
void update_teaching_sine();
/* Selects PLL voltage/current inputs from local or measured signals */
void update_pll_inputs();
/* Checks whether the PLL frequency estimate is close enough to 50 Hz */
bool following_frequency_in_range();
/* Copies the inverter controller diagnostics into scope variables */
void refresh_inverter_data();
/* Switches the PLL input source and resets synchronization state safely */
void configure_following_input(FollowingInputMode requested_mode);
/* Streams a completed ScopeMimicry capture over the serial console */
void dump_scope_datas(ScopeMimicry &scope_to_dump);
/* Returns to idle when PLL synchronization is lost for a sustained period */
void handle_following_desync();

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* Control task period in microseconds */
static constexpr uint32_t CONTROL_TASK_PERIOD_US = 100;
/* Control task period in seconds */
static constexpr float32_t TS = CONTROL_TASK_PERIOD_US * 1.0e-6F;
/* Fallback DC bus voltage in volts before sensing is valid */
static constexpr float32_t DC_BUS_FALLBACK = 20.0F;
/* DC bus voltage during startup mode in volts */
static constexpr float32_t UDC_STARTUP = 0.0F;
/* Grid frequency in hertz */
static constexpr float32_t F0 = 50.0F;
/* Grid pulsation in radians per second */
static constexpr float32_t W0 = 2.0F * PI * F0;
/* Allowed frequency deviation for synchronization in radians per second */
static constexpr float32_t SYNC_POWER_TOLERANCE = 0.01F * W0;
/* Load resistance for local sinewave  in ohms */
static constexpr float32_t LOAD_RESISTANCE = 10.0F;
/* Maximum current for overcurrent protection in amps */
static constexpr float32_t MAX_CURRENT = 8.0F;
/* Size of the scope buffer for data recording */
static constexpr uint32_t SCOPE_BUFFER_SIZE = 512;
/* Number of channels recorded in the scope for diagnostics */
static constexpr uint8_t SCOPE_CHANNEL_COUNT = 12;

/* State of the PWM outputs */
static bool pwm_enable = false;

/* State of scope data downloading */
static bool is_downloading = false;
/* State to trigger the scope capture */
static bool trigger = false;
/* State of grid synchronization */
static bool is_net_synchronized = false;

/* Variable to store the last received serial character */
static uint8_t received_serial_char;
/* Current operating mode of the inverter */
static uint8_t mode = IDLEMODE;
/* Last requested operating mode from the serial interface */
static uint8_t mode_asked = IDLEMODE;
/* Current PLL input source selection */
static FollowingInputMode following_input_mode = LOCAL_SINE_INPUT;
/* [V] Low-side filtered measurement of voltages 1, 2 and 3 */
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t V3_low_value;
/* [V] Neutral point potential*/
static float32_t Vn;
/* [A] Low-side filtered measurement of current 1, 2 and 3 */
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I3_low_value;
/* [V] High-side filtered measurement of voltage */
static float32_t V_high;
/* [V] Further filtered measurement of the high-side voltage for control */
static float32_t V_high_filt;
/* [V] Phase voltage (ou Line-to-neutral voltage) of low-side measurements */
static three_phase_t Vgrid_meas;
static three_phase_t Vgrid_e;
/* [A] Measured grid current from the low-side current measurement */
static three_phase_t Igrid_meas;
static three_phase_t Igrid_e;
/* [V] Temporary variable for storing sensor measurements */
static float32_t meas_data;
/* [V] Amplitude of the local teaching sine wave for the PLL input */
static float32_t local_voltage_amplitude = 20.0F;
/* [rad] Phase angle of the local teaching sine wave */
static float32_t teaching_theta;
/* [rad] Estimated phase angle of the grid from the inverter's PLL */
static float32_t inverter_theta;
/* [V] Instantaneous value of the local teaching sine wave */
static three_phase_t sine;
/* [V] Instantaneous local grid voltage from the teaching sine */
static three_phase_t local_vgrid;
/* [A] Instantaneous local grid current from the teaching sine */
static three_phase_t local_igrid;
/* [V] Voltage input to the PLL, selected from local or measured grid */
static three_phase_t pll_vgrid_input;
/* [A] Current input to the PLL, selected from local or measured grid */
static three_phase_t pll_igrid_input;

/* [V] DQ-axis voltage in the synchronous reference frame */
static dqo_t Vdq;
/* [V] DQ-axis voltage output from the inverter */
static dqo_t Vdq_output;
/* [A] DQ-axis current in the synchronous reference frame */
static dqo_t Idq;
/* [A] DQ-axis current reference in the synchronous reference frame */
static dqo_t Idq_ref;
/* [A] DQ-axis current reference delta for control adjustments */
static dqo_t Idq_ref_delta;

/* [No unit] Base duty cycle ,  */
static float32_t duty_cycle_1 = 0.5F;
static float32_t duty_cycle_2 = 0.5F;
static float32_t duty_cycle_3 = 0.5F;
/* [No unit] Adjusted duty cycle by the controller*/
static three_phase_t delta_duty_cycle;
/* [rad/s] Estimated grid frequency from the PLL */
static float32_t omega = W0;
/* [No unit] Scope variable for the current PLL input mode */
static float32_t input_mode_scope = 1.0F;
/* [No unit] Scope variable for the current operating mode */
static float32_t state_mode_scope;
/* [No unit] Scope variable for the PLL synchronization state */
static float32_t sync_scope;
/* Counter for the number of critical task iterations */
static uint32_t critical_task_counter;
/* Counter for consecutive desync iterations, triggers safe shutdown */
static uint32_t desync_counter;

/* Instance of the Three-phase inverter control class */
static ThreePhaseInverter inverter;
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
 * @brief Returns the measured DC bus voltage, or a fallback before
 *        sensing is valid.
 */
float32_t control_bus_voltage()
{
    if (V_high_filt > 1.0F) {
        return V_high_filt;
    }
    return DC_BUS_FALLBACK;
}

/**
 * @brief Clamps one duty-cycle command to the allowed modulation range.
 */
float32_t clamp_duty(float32_t duty)
{
    return saturate(duty, DUTY_MIN, DUTY_MAX);
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
 * @brief Applies clamped complementary duty cycles to the H-bridge legs.
 */
void apply_complementary_duty(three_phase_t duty)
{
    duty_cycle_1 = clamp_duty(duty.a);
    duty_cycle_2 = clamp_duty(duty.b);
    duty_cycle_3 = clamp_duty(duty.c);
    shield.power.setDutyCycle(LEG1, duty_cycle_1);
    shield.power.setDutyCycle(LEG2, duty_cycle_2);
    shield.power.setDutyCycle(LEG3, duty_cycle_3);
}

/**
 * @brief Advances the local teaching oscillator and derives
 *        voltage/current inputs.
 */
void update_teaching_sine()
{
    teaching_theta = ot_modulo_2pi(teaching_theta + W0 * TS);
    sine.a =  ot_sin(teaching_theta);
    sine.b =  ot_sin(teaching_theta - 2.0F * PI / 3.0F);
    sine.c =  ot_sin(teaching_theta - 4.0F * PI / 3.0F);
    local_vgrid.a = local_voltage_amplitude * sine.a;
    local_vgrid.b = local_voltage_amplitude * sine.b;
    local_vgrid.c = local_voltage_amplitude * sine.c;
    local_igrid.a = local_vgrid.a / LOAD_RESISTANCE;
    local_igrid.b = local_vgrid.b / LOAD_RESISTANCE;
    local_igrid.c = local_vgrid.c / LOAD_RESISTANCE;
}

/**
 * @brief Selects PLL voltage/current inputs from local or measured signals.
 */
void update_pll_inputs()
{
    if (following_input_mode == LOCAL_SINE_INPUT) {
        pll_vgrid_input = local_vgrid;
        pll_igrid_input = Igrid_meas;  //from local_igrid to Igrid_meas because if we use local_igrid, we will always have an error since local_igrid is constant
    } else {
        pll_vgrid_input = Vgrid_e;
        pll_igrid_input = Igrid_e;
    }
}

/**
 * @brief Checks whether the PLL frequency estimate is close enough to 50 Hz.
 */
bool following_frequency_in_range()
{
    return omega <= W0 + SYNC_POWER_TOLERANCE &&
           omega >= W0 - SYNC_POWER_TOLERANCE;
}

/**
 * @brief Copies the inverter controller diagnostics into scope variables.
 */
void refresh_inverter_data()
{
    Vdq = inverter.getVdq();  //just to plot it
    Vdq_output = inverter.getVdqOut();  //just to plot it
    Idq = inverter.getIdq(); //just to plot it
    Idq_ref = inverter.getIdqRef();
    Idq_ref_delta = inverter.getIdqRefDelta(); //no use in grid following ?? what is it doing here
    inverter_theta = inverter.getTheta();  //I guess only to plot too
    omega = inverter.getw(); // just to plot it
    sync_scope = is_net_synchronized ? 1.0F : 0.0F;
}

/**
 * @brief Switches the PLL input source and resets synchronization state safely.
 */
void configure_following_input(FollowingInputMode requested_mode)
{
    following_input_mode = requested_mode;
    input_mode_scope = static_cast<float32_t>(requested_mode);
    mode = IDLEMODE;
    mode_asked = IDLEMODE;
    is_net_synchronized = false;
    sync_scope = 0.0F;
    desync_counter = 0;
    delta_duty_cycle.a = 0.5F;
    delta_duty_cycle.b = 0.5F;
    delta_duty_cycle.c = 0.5F;
    inverter.init(FOLLOWING, DC_BUS_FALLBACK, local_voltage_amplitude, W0, TS);
    inverter.setPowerOn(false);
    stop_pwm_outputs();
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
 * @brief Returns to idle when PLL synchronization is lost for a
 *        sustained period.
 */
void handle_following_desync()
{
    if (is_net_synchronized) {      //if PLL is synchronized then we reset the desync counter
        desync_counter = 0;
        return;
    }

    desync_counter++;
    if (desync_counter > 200) {     //if the PLL isn't synchronized for a defined amount of time (when desync_counter > 200), it changes to IDLE Mode
        desync_counter = 0;
        mode_asked = IDLEMODE;
        mode = IDLEMODE;
        inverter.setPowerOn(false);
        stop_pwm_outputs();
        printk("System no longer synchronized\n");
    }
}

/**
 * @brief Adjusts the following-mode d-axis current reference.
 */
void adjust_current_reference(float32_t step)
{
    Idq_ref.d = saturate(Idq_ref.d + step, -0.1F, 8.0F);
    //inverter.setIdqRef(Idq_ref);
    
}

/**
 * @brief Registers scope channels and starts capture for the
 *        grid-following example.
 */
void setup_scope()
{
    /*scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(I3_low_value, "I2_low_value");*/
    scope.connectChannel(Vgrid_meas.a, "Va");
    scope.connectChannel(Vgrid_meas.b, "Vb");
    scope.connectChannel(Vgrid_meas.c, "Vc");
    scope.connectChannel(Igrid_meas.a, "Ia");
    scope.connectChannel(Igrid_meas.b, "Ib");
    scope.connectChannel(Igrid_meas.c, "Ic");
    /*scope.connectChannel(local_vgrid.a, "sine a");
    scope.connectChannel(local_vgrid.b, "sine b");
    scope.connectChannel(local_vgrid.c, "sine c");*/
    scope.connectChannel(delta_duty_cycle.a, "duty_cycle a");
    scope.connectChannel(delta_duty_cycle.b, "duty_cycle b");
    scope.connectChannel(delta_duty_cycle.c, "duty_cycle c");
    /*scope.connectChannel(V_high_filt, "Vdc");
    scope.connectChannel(Idq_ref.d, "Id_ref");
    scope.connectChannel(Idq_ref.q, "Iq_ref");
    scope.connectChannel(Vdq.d, "Vd_in");
    scope.connectChannel(Vdq.q, "Vq_in");*/
    scope.connectChannel(Idq.d, "Id_in");
    scope.connectChannel(Idq.q, "Iq_in");
    //scope.connectChannel(Vdq_output.d, "Vd_out");
    scope.connectChannel(Vdq_output.q, "Vq_out");
    /*scope.connectChannel(inverter_theta, "theta");
    scope.connectChannel(omega, "omega");
    scope.connectChannel(sync_scope, "sync");
    scope.connectChannel(input_mode_scope, "input_mode");
    scope.connectChannel(state_mode_scope, "state");*/
    scope.set_delay(0.5F);
    scope.set_trigger(a_trigger);
    scope.start();
}

/**
 * @brief Reads the latest sensor values and derives grid
 *        voltage/current measurements.
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

    meas_data = shield.sensors.getLatestValue(V3_LOW);
    if (meas_data != NO_VALUE) V3_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I3_LOW);
    if (meas_data != NO_VALUE) I3_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

    V_high_filt = vHighFilter.calculateWithReturn(V_high);

    Vn = (V1_low_value + V2_low_value + V3_low_value)/3.0F;

    // Line to Neutral point voltages 
    Vgrid_meas.a = V1_low_value - Vn;
    Vgrid_meas.b = V2_low_value - Vn;
    Vgrid_meas.c = V3_low_value - Vn;

    // Lines current
    Igrid_meas.a = I1_low_value;
    Igrid_meas.b = I2_low_value;
    Igrid_meas.c = I3_low_value;

}

/**
 * @brief Checks both measured currents against the protection threshold.
 */
bool overcurrent_detected()
{
    return I1_low_value > MAX_CURRENT ||
           I1_low_value < -MAX_CURRENT ||
           I2_low_value > MAX_CURRENT ||
           I2_low_value < -MAX_CURRENT ||
           I3_low_value > MAX_CURRENT ||
           I3_low_value < -MAX_CURRENT;
}

/**
 * @brief Runs PLL acquisition with PWM disabled until synchronization is valid.
 */
void run_startup_mode()
{
    inverter.setVBus(control_bus_voltage()); //used for protection against really low Vhigh
                                             //if Vhigh < 1 then Vbus is set to DC_BUS_FULLBACK = 20
    inverter.setIabcRef(local_igrid);  //We set Iqd_ref here to make calculateDuty function calculate the error between Idq_ref and pll_igrid_input
    inverter.setPowerOn(false);
    delta_duty_cycle = inverter.calculateDuty(pll_vgrid_input, pll_igrid_input);  /*pll_vgrid_input is mainly here to calculate the duty cycles from the PI correction 
                                                                                    using pll_igrid_input for a cascade loop*/
    refresh_inverter_data(); //only used to plot on mimicryscope
    is_net_synchronized = inverter.getSync() && following_frequency_in_range();   // Condition to change from STARTUP Mode to POWER Mode
    sync_scope = is_net_synchronized ? 1.0F : 0.0F;
    stop_pwm_outputs();
}

/**
 * @brief Runs synchronized following control and applies PWM only while locked.
 */
void run_power_mode()
{
    inverter.setVBus(control_bus_voltage()); //used for protection against really low Vhigh
                                             //if Vhigh < 1 then Vbus is set to DC_BUS_FULLBACK = 20
    inverter.setIabcRef(local_igrid);  //We set Iqd_ref here to make calculateDuty function calculate the error between Idq_ref and pll_igrid_input, 
                                  //we start with Idq_ref = 0
    inverter.setPowerOn(is_net_synchronized);
    delta_duty_cycle = inverter.calculateDuty(pll_vgrid_input, pll_igrid_input);  /*pll_vgrid_input is mainly here to calculate the duty cycles from the PI correction 
                                                                                    using pll_igrid_input because we can only do it with voltages*/
    refresh_inverter_data(); //only used to plot on mimicryscope
    is_net_synchronized = inverter.getSync() && following_frequency_in_range();     //inverter.getSync() is true if -0.1 < Vdq.q < 0.1 and sync_delay_counter > sync_min_delay
                                                                                    //sync_delay_counter++ if PLL is not synced and need to be superior to sync_min_delay = 1000
                                                                                    //the PLL needs to be stable for an amount of time (until sync_delay_counter > sync_min_delay)
                                                                                    //before we can consider it as stable able to sync.
    sync_scope = is_net_synchronized ? 1.0F : 0.0F;

    if (!is_net_synchronized) {     
        handle_following_desync();  //if the PLL is not synchronized anymore then it goes into idle mode.
        return;
    }

    /*desync_counter = 0;*/  // i will disable this line, because I think that it is redundant, desync_counter = 0 already happens in handle_following_desync
    apply_complementary_duty(delta_duty_cycle);
    start_pwm_outputs();
}

/*----------------------- END OF SUPPORT FUNCTIONS ------------------------- */

/*----------------------- MAIN APPLICATION CODE ---------------------------- */

/**
 * @brief Configures hardware, scope capture, inverter control,
 *        and task scheduling.
 */
void setup_routine()
{
    shield.power.setDeadTime(LEG1, 20, 20);
    shield.power.setDeadTime(LEG2, 20, 20);
    shield.power.setDeadTime(LEG3, 20, 20);
    shield.sensors.enableDefaultOwnverterSensors();
    shield.power.initBuck(ALL);

    Idq_ref.d = 1.0F;
    Idq_ref.q = 0.0F;
    Idq_ref_delta.d = 0.0F;     //I don't know the meaning of these 2 lines of code, they don't appear in ThreePhaseInverter.cpp in following mode
    Idq_ref_delta.q = 0.0F;     //this one also
    setup_scope();
    inverter.init(FOLLOWING, DC_BUS_FALLBACK, local_voltage_amplitude, W0, TS); 
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/**
 * @brief Handles serial commands for PLL input selection, tuning,
 *        and scope capture.
 */
void loop_communication_task()
{
    while (1) {
        received_serial_char = console_getchar();
        switch (received_serial_char) {
        case 'h':
            printk(" ________________________________________\n");
            printk("|     grid-following PLL                 |\n");
            printk("|     i : idle                           |\n");
            printk("|     p : power after PLL sync           |\n");
            printk("|     1 : PLL input from local sine      |\n");
            printk("|     2 : PLL input from V1_LOW - V2_LOW |\n");
            printk("|     u/j : Id reference +/- 0.1 A       |\n");
            printk("|     d/c : Id reference +/- 1 A         |\n");
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
        case '1':
            configure_following_input(LOCAL_SINE_INPUT);
            printk("PLL input: local sine\n");
            break;
        case '2':
            configure_following_input(MEASURED_GRID_INPUT);
            printk("PLL input: measured V1_LOW - V2_LOW\n");
            break;
        case 'u':
            adjust_current_reference(0.1F);
            break;
        case 'j':
            adjust_current_reference(-0.1F);
            break;
        case 'd':
            adjust_current_reference(1.0F);
            break;
        case 'c':
            adjust_current_reference(-1.0F);
            break;
        case 'q':
            scope.start();
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
}

/**
 * @brief Runs the low-rate state machine and status reporting.
 */
void loop_application_task()
{
    switch (mode) {     // STATE Machine between IDLEMODE, STARTUPMODE, POWERMODE and ERRORMODE.
    case IDLEMODE:
        if (mode_asked == POWERMODE && V_high_filt >= UDC_STARTUP) {
            mode = STARTUPMODE;
        }
        spin.led.turnOn();
        break;
    case STARTUPMODE:
        if (is_net_synchronized) {
            mode = POWERMODE;
        }
        break;
    case POWERMODE:
        if (mode_asked == IDLEMODE) {
            mode = IDLEMODE;
        }
        if (is_net_synchronized) {
            spin.led.toggle();
        }
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
        printk("%d:", following_input_mode);
        printk("%d:", is_net_synchronized ? 1 : 0);
        printk("%.2f:", static_cast<double>(V_high_filt));
        printk("%.2f:", static_cast<double>(pll_vgrid_input.a));
        printk("%.2f:", static_cast<double>(Vgrid_meas.a));
        printk("%.2f:", static_cast<double>(local_vgrid.a));
        printk("%.2f:", static_cast<double>(Idq_ref.d));
        printk("%.2f:", static_cast<double>(Idq.d));
        printk("%.2f:", static_cast<double>(Vdq.d));
        /* Vdq_output is plotted instead of Vdq directly, because Vdq_output
         * is the one we are supposed to calculate, and Vdq is just a
         * sinewave converted to dc */
        printk("%.2f:", static_cast<double>(Vdq_output.d));
        printk("%.2f:", static_cast<double>(Vdq_output.q));
        printk("\n");
    }

    task.suspendBackgroundMs(100);
}

/**
 * @brief Runs the 10 kHz following control loop, protection checks,
 *        and PWM updates.
 */
void loop_critical_task()
{
    critical_task_counter++;
    read_measurements();
    update_teaching_sine();
    update_pll_inputs();

    if (overcurrent_detected()) {
        mode = ERRORMODE;
    }

    if (mode == IDLEMODE || mode == ERRORMODE) {
        stop_pwm_outputs();
        inverter.setPowerOn(false);
        is_net_synchronized = false;
        desync_counter = 0;
        spin.led.turnOff();
    } else if (mode == STARTUPMODE) {
        run_startup_mode();
    } else if (mode == POWERMODE) {
        run_power_mode();
    }

    refresh_inverter_data();
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


