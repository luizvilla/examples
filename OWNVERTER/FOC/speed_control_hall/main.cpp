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
static const uint32_t speed_loop_decimation = 10U;
static const float32_t Ts_speed = Ts * speed_loop_decimation;
static const float32_t NB_OFFSET = 2000.0F;
static const float32_t IQ_REF_MAX = 2.0F;
static const float32_t SPEED_REF_STEP = 10.0F;
static const float32_t SPEED_REF_MAX = 300.0F;

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

static bool position_sensor_initialized;
static bool position_data_valid;
static float32_t electrical_angle;
static float32_t electrical_speed;
static float32_t speed_ref;

static three_phase_t Vabc;
static three_phase_t Iabc;
static three_phase_t duty_abc;
static dqo_t Vdq;
static dqo_t Idq;
static dqo_t Idq_ref;

static float32_t lower_bound = -MIN_DC_VOLTAGE * 0.4F;
static float32_t upper_bound = MIN_DC_VOLTAGE * 0.4F;
static float32_t Kp = 30.0F * 0.035F;
static float32_t Ti = 0.002029F;
static Pid pi_d = controlLibFactory.pid(Ts, Kp, Ti, 0.0F, 1.0F, lower_bound, upper_bound);
static Pid pi_q = controlLibFactory.pid(Ts, Kp, Ti, 0.0F, 1.0F, lower_bound, upper_bound);
static Pid pi_speed = controlLibFactory.pid(Ts_speed, 0.02F, 0.05F, 0.0F, 1.0F,
					    -IQ_REF_MAX, IQ_REF_MAX);
static LowPassFirstOrderFilter vHigh_filter =
	controlLibFactory.lowpassfilter(Ts, 5.0e-3F);
static LowPassFirstOrderFilter speed_filter =
	controlLibFactory.lowpassfilter(Ts, 5.0e-3F);

static bool pwm_enable;
static uint16_t error_counter;
static uint32_t counter_time;
static uint8_t received_serial_char;
static bool is_downloading;
static bool memory_print;
static float32_t control_state_f;
static float32_t iq_meas_print;
static float32_t iq_ref_print;
static float32_t vd_print;
static float32_t vq_print;
static float32_t speed_meas_print;

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
	electrical_angle = 0.0F;
	electrical_speed = 0.0F;
	speed_ref = 0.0F;
	pwm_enable = false;
	asked_mode = IDLEMODE;
	control_state = OFFSET_ST;
	vHigh_filter.reset(V_HIGH_MIN);
	speed_filter.reset(0.0F);
	pi_d.reset();
	pi_q.reset();
	pi_speed.reset();
}

void stop_pwm_if_needed()
{
	if (pwm_enable) {
		shield.power.stop(ALL);
		pwm_enable = false;
		pi_d.reset();
		pi_q.reset();
		pi_speed.reset();
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

	electrical_angle = shield.position.getElectricalAngle();
	electrical_speed = speed_filter.calculateWithReturn(shield.position.getElectricalSpeed());
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

void control_speed()
{
	if ((counter_time % speed_loop_decimation) == 0U) {
		Idq_ref.q = pi_speed.calculateWithReturn(speed_ref, electrical_speed);
	}

	Idq_ref.q = clamp(Idq_ref.q, -IQ_REF_MAX, IQ_REF_MAX);
	Idq_ref.d = 0.0F;

	Iabc.a = I1_low_value;
	Iabc.b = I2_low_value;
	Iabc.c = -(Iabc.a + Iabc.b);

	Idq = Transform::to_dqo(Iabc, electrical_angle);
	Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);
	Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);
	Vdq.o = 0.0F;

	Vabc = Transform::to_threephase(Vdq, electrical_angle);
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

	scope.connectChannel(speed_ref, "speed_ref");
	scope.connectChannel(speed_meas_print, "speed_meas");
	scope.connectChannel(iq_ref_print, "Iq_ref");
	scope.connectChannel(iq_meas_print, "Iq_meas");
	scope.connectChannel(vd_print, "Vd");
	scope.connectChannel(vq_print, "Vq");
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
			   "|     press u : speed ref UP             |\n"
			   "|     press d : speed ref DOWN           |\n"
			   "|     press r : dump scope capture       |\n"
			   "|     press q : restart scope capture    |\n"
			   "|     press m : replay scope memory      |\n"
			   "|________________________________________|\n\n");
		break;
	case 'i':
		asked_mode = IDLEMODE;
		speed_ref = 0.0F;
		break;
	case 'p':
		asked_mode = POWERMODE;
		scope.start();
		break;
	case 'o':
		restart_offset_calibration();
		break;
	case 'u':
		speed_ref = clamp(speed_ref + SPEED_REF_STEP, -SPEED_REF_MAX, SPEED_REF_MAX);
		break;
	case 'd':
		speed_ref = clamp(speed_ref - SPEED_REF_STEP, -SPEED_REF_MAX, SPEED_REF_MAX);
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
		printk("state:%d init:%u valid:%u w_ref:%.2f w_meas:%.2f Iq_ref:%.2f Iq_meas:%.2f Vhigh:%.2f\n",
		       control_state,
		       position_sensor_initialized ? 1U : 0U,
		       position_data_valid ? 1U : 0U,
		       (double)speed_ref,
		       (double)electrical_speed,
		       (double)Idq_ref.q,
		       (double)Idq.q,
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
		control_speed();
		apply_duties();
		start_pwm_if_needed();
		break;
	}

	iq_ref_print = Idq_ref.q;
	iq_meas_print = Idq.q;
	vd_print = Vdq.d;
	vq_print = Vdq.q;
	speed_meas_print = electrical_speed;
	control_state_f = (float32_t)control_state;
	scope.acquire();
}

int main(void)
{
	setup_routine();
	return 0;
}
