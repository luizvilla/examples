/*
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file test_control.cpp
 * @brief L0 test cases of the mission's control law (control.cpp) on the PC.
 *
 * L0 = one object with synthetic inputs. Expected values follow from the
 * control_library PI: duty = Kp * (e + integral(e) / Ti), with the
 * integral advanced by Ts * e each step, saturated to [0, 1] with
 * anti-windup.
 */
#include <cmath>
#include <cstdio>
#include "control.h"

static int failures = 0;

static void check(const char *id, bool ok, const char *what, double got, double expected)
{
    std::printf("%s %s: %s (got %.9g, expected %.9g)\n", ok ? "PASS" : "FAIL", id, what, got, expected);
    if (!ok) failures++;
}

static const float Ts = 100e-6F;      /* control task period of the mission */
static const double Kp = 0.000215;    /* from control.cpp */
static const double Ti = 7.5175e-5;   /* from control.cpp */
static const float reference = 15.0F; /* the example's default reference */

static Measurements at(float v1_low)
{
    Measurements m = {v1_low, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    return m;
}

int main()
{
    control_init(Ts);

    /* L0-CTL-01: no error from a fresh start gives zero duty. */
    float d = control_step(reference, at(reference));
    check("L0-CTL-01", d == 0.0F, "zero error from reset gives duty 0", d, 0.0);

    /* L0-CTL-02: first step with an error is Kp * e * (1 + Ts / Ti). */
    control_reset();
    const double e = 15.0;
    const double first = Kp * e * (1.0 + Ts / Ti);
    d = control_step(reference, at(0.0F));
    check("L0-CTL-02", std::fabs(d - first) < 1e-6, "first step equals Kp*e*(1+Ts/Ti)", d, first);

    /* L0-CTL-03: a constant error ramps the duty up, never above 1,
     * reaching exactly 1 at the step the PI formula predicts. */
    control_reset();
    const int predicted = (int)std::ceil((1.0 / (Kp * e) - 1.0) / (Ts / Ti));
    int reached = -1;
    bool monotonic = true, bounded = true;
    float previous = 0.0F;
    for (int n = 0; n < 1000; n++) {
        d = control_step(reference, at(0.0F));
        if (d < previous) monotonic = false;
        if (d > 1.0F) bounded = false;
        if (reached < 0 && d >= 1.0F) reached = n;
        previous = d;
    }
    check("L0-CTL-03a", monotonic, "duty never decreases under constant positive error", monotonic, 1);
    check("L0-CTL-03b", bounded, "duty never exceeds 1", previous, 1.0);
    check("L0-CTL-03c", std::abs(reached - predicted) <= 1, "duty saturates at the predicted step", reached, predicted);

    /* L0-CTL-04: anti-windup. After 1000 saturated steps, a small negative
     * error brings the duty below 1 at the very next step. In saturation the
     * PI holds integral / Ti = 1 / Kp - e (e = 15 V), so one step at -1 V
     * gives 1 - Kp * (e + 1 + Ts / Ti). */
    const double expected_drop = 1.0 - Kp * (e + 1.0 + Ts / Ti);
    d = control_step(reference, at(reference + 1.0F));
    check("L0-CTL-04", d < 1.0F && std::fabs(d - expected_drop) < 1e-4,
          "no windup: one negative-error step leaves saturation", d, expected_drop);

    /* L0-CTL-05: a large negative error saturates at exactly 0. */
    control_reset();
    for (int n = 0; n < 10; n++) d = control_step(reference, at(60.0F));
    check("L0-CTL-05", d == 0.0F, "large negative error gives duty 0", d, 0.0);

    /* L0-CTL-06: reset forgets everything; the first step matches L0-CTL-02. */
    control_reset();
    d = control_step(reference, at(0.0F));
    check("L0-CTL-06", std::fabs(d - first) < 1e-6, "after reset the first step equals L0-CTL-02", d, first);

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
