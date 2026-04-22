/*
 * Copyright (c) 2026-present OwnTech Foundation
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "ScopeMimicry.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"
#include "trigo.h"
#include "zephyr/console/console.h"

void setup_routine();
void loop_background_task();
void application_task();
void loop_critical_task();

static const float32_t Ts = 100.0e-6F;
static float32_t modulation_index = 0.20F;
static float32_t electrical_frequency_hz = 5.0F;
static float32_t electrical_angle = 0.0F;
static float32_t duty_a = 0.5F;
static float32_t duty_b = 0.5F;
static float32_t duty_c = 0.5F;
static float32_t mode_f;
static bool pwm_enable;
static uint8_t received_serial_char;

const uint16_t SCOPE_SIZE = 256;
ScopeMimicry scope(SCOPE_SIZE, 5);
uint16_t k_app_idx;
static bool is_downloading;
static bool memory_print;

enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

static uint8_t mode = IDLEMODE;

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
	return (mode == POWERMODE);
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

void setup_routine()
{
	shield.power.initBuck(ALL);

	scope.connectChannel(duty_a, "duty_a");
	scope.connectChannel(duty_b, "duty_b");
	scope.connectChannel(duty_c, "duty_c");
	scope.connectChannel(electrical_angle, "electrical_angle");
	scope.connectChannel(mode_f, "mode");
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0F);
	scope.start();

	uint32_t background_task_number = task.createBackground(loop_background_task);
	uint32_t application_task_number = task.createBackground(application_task);
	task.createCritical(loop_critical_task, 100);

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
			   "|     press u : modulation index UP      |\n"
			   "|     press d : modulation index DOWN    |\n"
			   "|     press f : frequency UP             |\n"
			   "|     press g : frequency DOWN           |\n"
			   "|     press r : dump scope capture       |\n"
			   "|     press q : restart scope capture    |\n"
			   "|     press m : replay scope memory      |\n"
			   "|________________________________________|\n\n");
		break;
	case 'i':
		mode = IDLEMODE;
		break;
	case 'p':
		mode = POWERMODE;
		scope.start();
		break;
	case 'u':
		modulation_index = clamp(modulation_index + 0.02F, 0.0F, 0.45F);
		break;
	case 'd':
		modulation_index = clamp(modulation_index - 0.02F, 0.0F, 0.45F);
		break;
	case 'f':
		electrical_frequency_hz = clamp(electrical_frequency_hz + 1.0F, 0.0F, 40.0F);
		break;
	case 'g':
		electrical_frequency_hz = clamp(electrical_frequency_hz - 1.0F, 0.0F, 40.0F);
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
		printk("mode:%u angle:%.2f freq:%.2f mod:%.2f da:%.3f db:%.3f dc:%.3f\n",
		       mode,
		       (double)electrical_angle,
		       (double)electrical_frequency_hz,
		       (double)modulation_index,
		       (double)duty_a,
		       (double)duty_b,
		       (double)duty_c);
	} else {
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.3f:%.3f:%.3f:%.2f:%.1f\n",
		       (double)scope.get_channel_value(k_app_idx, 0),
		       (double)scope.get_channel_value(k_app_idx, 1),
		       (double)scope.get_channel_value(k_app_idx, 2),
		       (double)scope.get_channel_value(k_app_idx, 3),
		       (double)scope.get_channel_value(k_app_idx, 4));
	}

	if (is_downloading) {
		dump_scope_datas(scope);
		is_downloading = false;
	}

	task.suspendBackgroundMs(200);
}

void loop_critical_task()
{
	if (mode == POWERMODE) {
		electrical_angle = ot_modulo_2pi(electrical_angle + 2.0F * PI * electrical_frequency_hz * Ts);
		duty_a = 0.5F + modulation_index * ot_sin(electrical_angle);
		duty_b = 0.5F + modulation_index * ot_sin(electrical_angle - 2.0F * PI / 3.0F);
		duty_c = 0.5F + modulation_index * ot_sin(electrical_angle - 4.0F * PI / 3.0F);

		shield.power.setDutyCycle(LEG1, duty_a);
		shield.power.setDutyCycle(LEG2, duty_b);
		shield.power.setDutyCycle(LEG3, duty_c);
		start_pwm_if_needed();
	} else {
		duty_a = 0.5F;
		duty_b = 0.5F;
		duty_c = 0.5F;
		stop_pwm_if_needed();
	}

	mode_f = (float32_t)mode;
	scope.acquire();
}

int main(void)
{
	setup_routine();
	return 0;
}
