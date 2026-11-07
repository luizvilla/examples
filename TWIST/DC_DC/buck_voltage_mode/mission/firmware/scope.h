/*
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  The firmware scope: OwnTech's ScopeMimicry behind the Scope data
 *         objects (data_objects.yaml), so the computer can record what the
 *         critical task sees, sample by sample, and read it over ThingSet.
 *
 * Nothing here touches the converter: the scope only reads variables.
 */
#pragma once

/* Connect the channels. Call once, in setup_routine(). */
void scope_init(float control_period_s);

/* Record one sample of every channel. Call at the end of each critical task period. */
void scope_sample();
