/*
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file control_block.cpp
 * @brief C entry points around the unchanged control.cpp, for the simulator target (L1).
 *
 * Compiled with firmware/control.cpp and control_library into a shared
 * library that the L1 runner calls once per control period. No control
 * logic lives here: only type conversion.
 */
#include "control.h"

extern "C" {

void block_init(double period_s) { control_init(static_cast<float>(period_s)); }

void block_reset(void) { control_reset(); }

double block_step(double reference, double v1_low, double i1_low, double v2_low,
                  double i2_low, double v_high, double i_high)
{
    Measurements m = {static_cast<float>(v1_low), static_cast<float>(i1_low),
                      static_cast<float>(v2_low), static_cast<float>(i2_low),
                      static_cast<float>(v_high), static_cast<float>(i_high)};
    return control_step(static_cast<float>(reference), m);
}

}
