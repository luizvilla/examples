/*
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file control.h
 * @brief The control law of the buck voltage-mode mission.
 *
 * Everything in control.h and control.cpp is plain C++: no OwnTech API,
 * no Zephyr, no hardware. That is what lets OwnStudio run the same code
 * inside the Pulsim simulator (test level L1) and on the Spin board (L2).
 * Keep it that way: read sensors and drive PWM in main.cpp, compute here.
 */
#pragma once

/** Latest sensor values, in volts and amperes, as main.cpp read them. */
struct Measurements {
    float v1_low;
    float i1_low;
    float v2_low;
    float i2_low;
    float v_high;
    float i_high;
};

/** Prepare the control law. period_s is the control task period in seconds. */
void control_init(float period_s);

/** Forget the controller's memory (integral and previous error). */
void control_reset();

/**
 * One control step: called once per control task period while the
 * converter is in power mode.
 *
 * @param reference  output voltage reference, in volts
 * @param m          latest measurements
 * @return duty cycle to apply to both legs, between 0 and 1
 */
float control_step(float reference, const Measurements &m);
