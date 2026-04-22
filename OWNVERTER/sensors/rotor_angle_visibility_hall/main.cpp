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
static float32_t electrical_angle;
static float32_t mechanical_angle;
static float32_t electrical_speed;
static float32_t mechanical_speed;
static float32_t hall_sector;
static float32_t sensor_valid_f;
static bool position_sensor_initialized;
static bool position_data_valid;
static bool stream_enabled;
static uint8_t received_serial_char;

const uint16_t SCOPE_SIZE = 256;
ScopeMimicry scope(SCOPE_SIZE, 6);
uint16_t k_app_idx;
static bool is_downloading;
static bool memory_print;

enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

static uint8_t mode = IDLEMODE;

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

void setup_routine()
{
	position_sensor_initialized = shield.position.initDefault();

	scope.connectChannel(electrical_angle, "electrical_angle");
	scope.connectChannel(mechanical_angle, "mechanical_angle");
	scope.connectChannel(electrical_speed, "electrical_speed");
	scope.connectChannel(mechanical_speed, "mechanical_speed");
	scope.connectChannel(hall_sector, "hall_sector");
	scope.connectChannel(sensor_valid_f, "sensor_valid");
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
			   "|     press i : stop sensor stream       |\n"
			   "|     press p : start sensor stream      |\n"
			   "|     press r : dump scope capture       |\n"
			   "|     press q : restart scope capture    |\n"
			   "|     press m : replay scope memory      |\n"
			   "|________________________________________|\n\n");
		break;
	case 'i':
		mode = IDLEMODE;
		stream_enabled = false;
		break;
	case 'p':
		mode = POWERMODE;
		stream_enabled = true;
		scope.start();
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
		printk("mode:%u init:%u valid:%u mech:%.3f elec:%.3f w_mech:%.3f w_elec:%.3f sector:%.0f\n",
		       mode,
		       position_sensor_initialized ? 1U : 0U,
		       position_data_valid ? 1U : 0U,
		       (double)mechanical_angle,
		       (double)electrical_angle,
		       (double)mechanical_speed,
		       (double)electrical_speed,
		       (double)hall_sector);
	} else {
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.3f:%.3f:%.3f:%.3f:%.1f:%.1f\n",
		       (double)scope.get_channel_value(k_app_idx, 0),
		       (double)scope.get_channel_value(k_app_idx, 1),
		       (double)scope.get_channel_value(k_app_idx, 2),
		       (double)scope.get_channel_value(k_app_idx, 3),
		       (double)scope.get_channel_value(k_app_idx, 4),
		       (double)scope.get_channel_value(k_app_idx, 5));
	}

	if (is_downloading) {
		dump_scope_datas(scope);
		is_downloading = false;
	}

	task.suspendBackgroundMs(200);
}

void loop_critical_task()
{
	if (stream_enabled && position_sensor_initialized && shield.position.update(Ts)) {
		position_data_valid = true;
		mechanical_angle = shield.position.getMechanicalAngle();
		electrical_angle = shield.position.getElectricalAngle();
		mechanical_speed = shield.position.getMechanicalSpeed();
		electrical_speed = shield.position.getElectricalSpeed();
		hall_sector = (float32_t)((uint32_t)(electrical_angle / (PI / 3.0F)) % 6U);
	} else if (!stream_enabled) {
		position_data_valid = false;
	}

	sensor_valid_f = position_data_valid ? 1.0F : 0.0F;
	scope.acquire();
}

int main(void)
{
	setup_routine();
	return 0;
}
