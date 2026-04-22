/*
 * Copyright (c) 2026-present OwnTech Foundation
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "ScopeMimicry.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "transform.h"
#include "trigo.h"
#include "zephyr/console/console.h"

void setup_routine();
void loop_background_task();
void application_task();
void loop_critical_task();

static const float32_t AC_CURRENT_LIMIT = 3.0F;
static const float32_t DC_CURRENT_LIMIT = 2.0F;
static const float32_t V_HIGH_MIN = 5.0F;
static const float32_t MIN_DC_VOLTAGE = 30.0F;
static const float32_t Ts = 100.0e-6F;
static const uint32_t control_task_period = (uint32_t)(Ts * 1.0e6F);
static const float32_t NB_OFFSET = 2000.0F;

static float32_t V_HIGH_STEP = 1.0F;
static float32_t W_OPEN_LOOP_STEP = 5.0F;
static float32_t voltage_q_ref = 2.0F;
static float32_t open_loop_speed_ref = 20.0F;
static float32_t open_loop_angle;
static float32_t measured_electrical_angle;
static float32_t measured_electrical_speed;

static float32_t meas_data;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;
static float32_t I1_offset;
static float32_t I2_offset;
static float32_t tmpI1_offset;
static float32_t tmpI2_offset;
static float32_t V_high_filtered;

static three_phase_t Vabc;
static three_phase_t duty_abc;
static dqo_t Vdq;

static float32_t lower_bound = -MIN_DC_VOLTAGE * 0.4F;
static float32_t upper_bound = MIN_DC_VOLTAGE * 0.4F;
static LowPassFirstOrderFilter vHigh_filter =
	controlLibFactory.lowpassfilter(Ts, 5.0e-3F);

static bool position_sensor_initialized;
static bool position_data_valid;
static bool pwm_enable;
static uint16_t error_counter;
static uint32_t counter_time;
static uint8_t received_serial_char;
static bool is_downloading;
static bool memory_print;
static float32_t control_state_f;

enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

enum control_state_mode
{
	OFFSET_ST = 0,
	IDLE_ST = 1,
	POWER_ST = 2,
	ERROR_ST = 3
};

static uint8_t asked_mode = IDLEMODE;
static control_state_mode control_state;

const uint16_t SCOPE_SIZE = 512;
ScopeMimicry scope(SCOPE_SIZE, 8);
uint16_t k_app_idx;

static float32_t clamp(float32_t value, float32_t min_value, float32_t max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

bool mytrigger()
{
	return (control_state == POWER_ST);
}

void dump_scope_datas(ScopeMimicry &scope_buffer)
{
	printk("begin record\n");
	scope_buffer.reset_dump();
	while (scope_buffer.get_dump_state() != finished) {
		printk("%s", scope_buffer.dump_datas());
		task.suspendBackgroundUs(200);
	}
	printk("end record\n");
}

void init_runtime()
{
	counter_time = 0U;
	error_counter = 0U;
	I1_low_value = 0.0F;
	I2_low_value = 0.0F;
	I_high = 0.0F;
	V_high = 0.0F;
	I1_offset = 0.0F;
	I2_offset = 0.0F;
	tmpI1_offset = 0.0F;
	tmpI2_offset = 0.0F;
	open_loop_angle = 0.0F;
	measured_electrical_angle = 0.0F;
	measured_electrical_speed = 0.0F;
	pwm_enable = false;
	asked_mode = IDLEMODE;
	control_state = OFFSET_ST;
	vHigh_filter.reset(V_HIGH_MIN);
}

void stop_pwm_if_needed()
{
	if (pwm_enable) {
		shield.power.stop(ALL);
		pwm_enable = false;
	}
}

void start_pwm_if_needed()
{
	if (!pwm_enable) {
		shield.power.start(ALL);
		pwm_enable = true;
	}
}

void restart_offset_calibration()
{
	stop_pwm_if_needed();
	counter_time = 0U;
	I1_offset = 0.0F;
	I2_offset = 0.0F;
	tmpI1_offset = 0.0F;
	tmpI2_offset = 0.0F;
	asked_mode = IDLEMODE;
	control_state = OFFSET_ST;
}

void retrieve_measurements()
{
	meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (meas_data != NO_VALUE) {
		I1_low_value = meas_data + I1_offset;
	}

	meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (meas_data != NO_VALUE) {
		I2_low_value = meas_data + I2_offset;
	}

	if ((control_state == OFFSET_ST) && (counter_time < (uint32_t)NB_OFFSET)) {
		tmpI1_offset += I1_low_value;
		tmpI2_offset += I2_low_value;
	}

	meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (meas_data != NO_VALUE) {
		I_high = -meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_high = meas_data;
	}

	V_high_filtered = vHigh_filter.calculateWithReturn(V_high);
}

void update_position()
{
	position_data_valid = false;
	if (!position_sensor_initialized) {
		return;
	}

	if (!shield.position.update(Ts)) {
		return;
	}

	measured_electrical_angle = shield.position.getElectricalAngle();
	measured_electrical_speed = shield.position.getElectricalSpeed();
	position_data_valid = true;
}

void overcurrent_management()
{
	if (I1_low_value > AC_CURRENT_LIMIT || I1_low_value < -AC_CURRENT_LIMIT ||
	    I2_low_value > AC_CURRENT_LIMIT || I2_low_value < -AC_CURRENT_LIMIT ||
	    I_high > DC_CURRENT_LIMIT) {
		error_counter++;
	}
	if (error_counter > 1000U) {
		control_state = ERROR_ST;
	}
}

void compute_open_loop_control()
{
	open_loop_angle = ot_modulo_2pi(open_loop_angle + open_loop_speed_ref * Ts);
	Vdq.d = 0.0F;
	Vdq.q = voltage_q_ref;
	Vdq.o = 0.0F;
	Vabc = Transform::to_threephase(Vdq, open_loop_angle);
	duty_abc.a = clamp(Vabc.a / MIN_DC_VOLTAGE + 0.5F, 0.02F, 0.98F);
	duty_abc.b = clamp(Vabc.b / MIN_DC_VOLTAGE + 0.5F, 0.02F, 0.98F);
	duty_abc.c = clamp(Vabc.c / MIN_DC_VOLTAGE + 0.5F, 0.02F, 0.98F);
}

void apply_duties()
{
	shield.power.setDutyCycle(LEG1, duty_abc.a);
	shield.power.setDutyCycle(LEG2, duty_abc.b);
	shield.power.setDutyCycle(LEG3, duty_abc.c);
}

void setup_routine()
{
	shield.power.initBuck(ALL);
	shield.sensors.enableDefaultOwnverterSensors();
	position_sensor_initialized = shield.position.initDefault();

	scope.connectChannel(voltage_q_ref, "Vq_ref");
	scope.connectChannel(open_loop_speed_ref, "w_open_loop");
	scope.connectChannel(open_loop_angle, "angle_command");
	scope.connectChannel(measured_electrical_angle, "angle_meas");
	scope.connectChannel(measured_electrical_speed, "speed_meas");
	scope.connectChannel(I1_low_value, "I1_low");
	scope.connectChannel(V_high, "V_high");
	scope.connectChannel(control_state_f, "control_state");
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0F);
	scope.start();

	init_runtime();

	uint32_t background_task_number = task.createBackground(loop_background_task);
	uint32_t application_task_number = task.createBackground(application_task);
	task.createCritical(loop_critical_task, control_task_period);

	task.startBackground(background_task_number);
	task.startBackground(application_task_number);
	task.startCritical();
}

void loop_background_task()
{
	received_serial_char = console_getchar();
	switch (received_serial_char) {
	case 'h':
		printk(" ________________________________________ \n"
			   "|     ------- MENU ---------             |\n"
			   "|     press i : idle mode                |\n"
			   "|     press p : power mode               |\n"
			   "|     press o : offset calibration       |\n"
			   "|     press u : Vq UP                    |\n"
			   "|     press d : Vq DOWN                  |\n"
			   "|     press f : open-loop speed UP       |\n"
			   "|     press g : open-loop speed DOWN     |\n"
			   "|     press r : dump scope capture       |\n"
			   "|     press q : restart scope capture    |\n"
			   "|     press m : replay scope memory      |\n"
			   "|________________________________________|\n\n");
		break;
	case 'i':
		asked_mode = IDLEMODE;
		break;
	case 'p':
		asked_mode = POWERMODE;
		scope.start();
		break;
	case 'o':
		restart_offset_calibration();
		break;
	case 'u':
		voltage_q_ref = clamp(voltage_q_ref + V_HIGH_STEP, -8.0F, 8.0F);
		break;
	case 'd':
		voltage_q_ref = clamp(voltage_q_ref - V_HIGH_STEP, -8.0F, 8.0F);
		break;
	case 'f':
		open_loop_speed_ref = clamp(open_loop_speed_ref + W_OPEN_LOOP_STEP, -300.0F, 300.0F);
		break;
	case 'g':
		open_loop_speed_ref = clamp(open_loop_speed_ref - W_OPEN_LOOP_STEP, -300.0F, 300.0F);
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

void application_task()
{
	if (!memory_print) {
		printk("state:%d init:%u valid:%u Vq:%.2f wcmd:%.2f angle_cmd:%.2f angle_meas:%.2f w_meas:%.2f Vhigh:%.2f\n",
		       control_state,
		       position_sensor_initialized ? 1U : 0U,
		       position_data_valid ? 1U : 0U,
		       (double)voltage_q_ref,
		       (double)open_loop_speed_ref,
		       (double)open_loop_angle,
		       (double)measured_electrical_angle,
		       (double)measured_electrical_speed,
		       (double)V_high);
	} else {
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.2f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f:%.1f\n",
		       (double)scope.get_channel_value(k_app_idx, 0),
		       (double)scope.get_channel_value(k_app_idx, 1),
		       (double)scope.get_channel_value(k_app_idx, 2),
		       (double)scope.get_channel_value(k_app_idx, 3),
		       (double)scope.get_channel_value(k_app_idx, 4),
		       (double)scope.get_channel_value(k_app_idx, 5),
		       (double)scope.get_channel_value(k_app_idx, 6),
		       (double)scope.get_channel_value(k_app_idx, 7));
	}

	if (is_downloading) {
		dump_scope_datas(scope);
		is_downloading = false;
	}

	if ((control_state == OFFSET_ST) && (counter_time > (uint32_t)NB_OFFSET)) {
		I1_offset = -tmpI1_offset / NB_OFFSET;
		I2_offset = -tmpI2_offset / NB_OFFSET;
		control_state = IDLE_ST;
	} else if ((control_state == IDLE_ST) &&
		   (asked_mode == POWERMODE) &&
		   position_sensor_initialized &&
		   position_data_valid &&
		   (V_high_filtered > V_HIGH_MIN)) {
		control_state = POWER_ST;
	} else if ((control_state == POWER_ST) && (asked_mode == IDLEMODE)) {
		control_state = IDLE_ST;
	} else if ((control_state == ERROR_ST) && (asked_mode == IDLEMODE)) {
		error_counter = 0U;
		control_state = IDLE_ST;
	}

	task.suspendBackgroundMs(200);
}

void loop_critical_task()
{
	counter_time++;
	retrieve_measurements();
	update_position();
	overcurrent_management();

	if ((control_state == POWER_ST) && !position_data_valid) {
		control_state = ERROR_ST;
	}

	switch (control_state) {
	case OFFSET_ST:
	case IDLE_ST:
	case ERROR_ST:
		stop_pwm_if_needed();
		break;
	case POWER_ST:
		compute_open_loop_control();
		apply_duties();
		start_pwm_if_needed();
		break;
	}

	control_state_f = (float32_t)control_state;
	scope.acquire();
}

int main(void)
{
	setup_routine();
	return 0;
}
