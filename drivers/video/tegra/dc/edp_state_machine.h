/*
 * edp_state_machine.h
 *
 * HDMI library support functions for Nvidia Tegra processors.
 *
 * Copyright (C) 2013 Google - http://www.google.com/
 * Copyright (C) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 * Authors:	John Grossman <johngro@google.com>
 * Authors:	Mike J. Chen <mjchen@google.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA_EDP_STATE_MACHINE_H
#define __TEGRA_EDP_STATE_MACHINE_H

#include "dp.h"

enum {
	/* The initial state for the state machine.  When entering RESET, we
	 * shut down all output and then proceed to the CHECK_PLUG state after a
	 * short debounce delay.
	 */
	EDP_STATE_RESET = 0,

	/* After the debounce delay, check the status of the HPD line.  If its
	 * low, then the cable is unplugged and we go directly to DONE_DISABLED.
	 * If it is high, then the cable is plugged and we proceed to CHECK_EDID
	 * in order to read the EDID and figure out the next step.
	 */
	EDP_STATE_CHECK_PLUG_STATE,

	/* CHECK_EDID is the state we stay in attempting to read the EDID
	 * information after we check the plug state and discover that we are
	 * plugged in.  If we max out our retries and fail to read the EDID, we
	 * move to DONE_DISABLED.  If we successfully read the EDID, we move on
	 * to DONE_ENABLE, set an initial video mode, then signal to the high
	 * level that we are ready for final mode selection.
	 */
	EDP_STATE_CHECK_EDID,

	/* DONE_DISABLED is the state we stay in after being reset and either
	 * discovering that no cable is plugged in or after we think a cable is
	 * plugged in but fail to read EDID.
	 */
	EDP_STATE_DONE_DISABLED,

	/* DONE_ENABLED is the state we say in after being reset and disovering
	 * a valid EDID at the other end of a plugged cable.
	 */
	EDP_STATE_DONE_ENABLED,

	/* Initial state at boot that checks if EDP is already initialized
	 * by bootloader and not go to EDP_STATE_RESET which would disable
	 * EDP and cause blanking of the bootloader displayed image.
	 */
	EDP_STATE_INIT_FROM_BOOTLOADER,

	/* STATE_COUNT must be the final state in the enum.
	 * 1) Do not add states after STATE_COUNT.
	 * 2) Do not assign explicit values to the states.
	 * 3) Do not reorder states in the list without reordering the dispatch
	 *    table in hdmi_state_machine.c
	 */
	EDP_STATE_COUNT,
};

void edp_state_machine_init(struct tegra_dc_dp_data *edp);
void edp_state_machine_reset(void);
void edp_state_machine_shutdown(void);
void edp_state_machine_set_pending_hpd(void);
int edp_state_machine_get_state(void);

#endif  /* __TEGRA_EDP_STATE_MACHINE_H */
