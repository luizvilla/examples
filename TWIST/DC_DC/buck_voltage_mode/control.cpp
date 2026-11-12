/*
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file control.cpp
 * @brief PI voltage control of the Twist buck, hardware-free (see control.h).
 */
#include "control.h"
#include "pid.h"

/* PI coefficients for an 8.6 ms step response (from the original example). */
static const float kp = 0.000215F;
static const float Ti = 7.5175e-5F;
static const float Td = 0.0F;
static const float N = 0.0F;
static const float lower_bound = 0.0F;
static const float upper_bound = 1.0F;

static Pid pid;

void control_init(float period_s)
{
    PidParams params(period_s, kp, Ti, Td, N, lower_bound, upper_bound);
    pid.init(params);
}

void control_reset()
{
    pid.reset();
}

float control_step(float reference, const Measurements &m)
{
    /* Regulate the low-side voltage of leg 1; both legs get the same duty. */
    return pid.calculateWithReturn(reference, m.v1_low);
}
