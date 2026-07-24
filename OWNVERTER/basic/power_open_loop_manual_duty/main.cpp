/*
 * Copyright (c) 2026-present OwnTech Foundation
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "ScopeMimicry.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"
#include "zephyr/console/console.h"

void setup_routine();
void loop_background_task();
void application_task();
void loop_critical_task();

static float32_t duty_cycle = 0.50F;
static float32_t duty_cycle_print = 0.50F;
static float32_t mode_f;
static bool pwm_enable;
static uint8_t received_serial_char;

const uint16_t SCOPE_SIZE = 256;
ScopeMimicry scope(SCOPE_SIZE, 2);
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

	scope.connectChannel(duty_cycle_print, "duty_cycle");
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
			   "|     press u : duty cycle UP            |\n"
			   "|     press d : duty cycle DOWN          |\n"
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
		duty_cycle = clamp(duty_cycle + 0.01F, 0.05F, 0.95F);
		break;
	case 'd':
		duty_cycle = clamp(duty_cycle - 0.01F, 0.05F, 0.95F);
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
		printk("%u:", mode);
		printk("%.3f:", (double)duty_cycle);
		printk("\n");
	} else {
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.3f:", (double)scope.get_channel_value(k_app_idx, 0));
		printk("%.1f:", (double)scope.get_channel_value(k_app_idx, 1));
		printk("\n");
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
		shield.power.setDutyCycle(ALL, duty_cycle);
		start_pwm_if_needed();
	} else {
		stop_pwm_if_needed();
	}

	duty_cycle_print = duty_cycle;
	mode_f = (float32_t)mode;
	scope.acquire();
}

int main(void)
{
	setup_routine();
	return 0;
}
