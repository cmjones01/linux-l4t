/*
 * edp_state_machine.c
 *
 * EDP library support functions for Nvidia Tegra processors.
 *
 * Copyright (C) 2012-2013 Google - http://www.google.com/
 * Copyright (C) 2013-2014 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/kernel.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <linux/console.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <video/tegrafb.h>
#include "dc_priv.h"

#ifdef CONFIG_ADF_TEGRA
#include "tegra_adf.h"
#endif

#include "edp_state_machine.h"

/************************************************************
 *
 * state machine internal state
 *
 ************************************************************/
static DEFINE_RT_MUTEX(work_lock);
static struct edp_state_machine_worker_data {
	struct delayed_work dwork;
	struct tegra_dc_dp_data *edp;
	int shutdown;
	int state;
	int pending_hpd_evt;
} work_state;

/************************************************************
 *
 * state machine internal methods
 *
 ************************************************************/
static void edp_state_machine_sched_work_l(int resched_time)
{
	cancel_delayed_work(&work_state.dwork);
	if ((resched_time >= 0) && !work_state.shutdown)
		queue_delayed_work(system_nrt_wq,
				&work_state.dwork,
				msecs_to_jiffies(resched_time));
}

static const char * const state_names[] = {
	"Reset",
	"Disabled",
	"Enabled",
	"Takeover from bootloader",
};

static void edp_state_machine_set_state_l(int target_state, int resched_time)
{
	rt_mutex_lock(&work_lock);

	pr_info("%s: switching from state %d (%s) to state %d (%s)\n",
		__func__, work_state.state, state_names[work_state.state],
		target_state, state_names[target_state]);
	work_state.state = target_state;

	/* If the pending_hpd_evt flag is already set, don't bother to
	 * reschedule the state machine worker.  We should be able to assert
	 * that there is a worker callback already scheduled, and that it is
	 * scheduled to run immediately.  This is particularly important when
	 * making the transition to the steady state ENABLED or DISABLED states.
	 * If an HPD event occurs while the worker is in flight, after the
	 * worker checks the state of the pending HPD flag, and then the state
	 * machine transitions to ENABLE or DISABLED, the system would end up
	 * canceling the callback to handle the HPD event were it not for this
	 * check.
	 */
	if (!work_state.pending_hpd_evt)
		edp_state_machine_sched_work_l(resched_time);

	rt_mutex_unlock(&work_lock);
}

static void edp_state_machine_handle_hpd_l(int cur_hpd)
{
	int tgt_state = work_state.state;

	if ((EDP_STATE_DONE_ENABLED == work_state.state) && !cur_hpd) {
		/* Did HPD drop while we were in DONE_ENABLED?  
		 */
		tgt_state = EDP_STATE_RESET;
	} else if(cur_hpd) {
		/* Looks like there was HPD activity while we were neither
		 * waiting for it to go away during steady state output, nor
		 * looking for it to come back after such an event.  Wait until
		 * HPD has been steady for at least 40 mSec, then restart the
		 * state machine.
		 */
		tgt_state = EDP_STATE_DONE_ENABLED;
	}

	edp_state_machine_set_state_l(tgt_state, 0);
}

/************************************************************
 *
 * internal state handlers and dispatch table
 *
 ************************************************************/
static void handle_enable_l(struct tegra_dc_dp_data *edp)
{
	struct fb_event event;
	struct fb_info *pfb = edp->dc->fb->info;
	int blank = 1;

	pr_err("%s\n",__func__);
	event.info = pfb;
	event.data = &blank;

	//tegra_dc_enable(edp->dc);

	console_lock();
	/* blank */
	fb_notifier_call_chain(FB_EVENT_BLANK, &event);
	blank = 0;
	/* unblank */
	fb_notifier_call_chain(FB_EVENT_BLANK, &event);
	console_unlock();
}

static void edp_disable_l(struct tegra_dc_dp_data *edp)
{
	pr_err("%s\n",__func__);
	if (edp->dc->enabled) {
		pr_err("EDP from connected to disconnected\n");
		edp->dc->connected = false;
		//tegra_dc_disable(edp->dc);
#ifdef CONFIG_ADF_TEGRA
		tegra_adf_process_hotplug_disconnected(edp->dc->adf);
#else
		tegra_fb_update_monspecs(edp->dc->fb, NULL, NULL);
#endif
		//tegra_dc_ext_process_hotplug(edp->dc->ndev->id);
	}
}

static void handle_reset_l(struct tegra_dc_dp_data *edp)
{
	/* Were we just reset?  If so, shut everything down, then schedule a
	 * check of the plug state in the near future.
	 */
	edp_disable_l(edp);
	edp_state_machine_set_state_l(EDP_STATE_DONE_DISABLED, -1);
}

typedef void (*dispatch_func_t)(struct tegra_dc_dp_data *edp);
static const dispatch_func_t state_machine_dispatch[] = {
	handle_reset_l,			/* STATE_RESET */
	NULL,				/* STATE_DONE_DISABLED */
	handle_enable_l,		/* STATE_DONE_ENABLED */
	NULL,				/* STATE_INIT_FROM_BOOTLOADER */
};

/************************************************************
 *
 * main state machine worker function
 *
 ************************************************************/
static void edp_state_machine_worker(struct work_struct *work)
{
	int pending_hpd_evt, cur_hpd;

	/* Check if the DC and FB are setup, if not just abort */
	if (!work_state.edp->dc || !work_state.edp->dc->fb) {
		pr_warn("%s (tid %p): %s is not yet set!\n",
				__func__, current, work_state.edp->dc ? "FB" : "DC");
		return;
	}

	/* Observe and clear the pending flag and latch the current HPD state.
	 */
	rt_mutex_lock(&work_lock);
	pending_hpd_evt = work_state.pending_hpd_evt;
	work_state.pending_hpd_evt = 0;
	rt_mutex_unlock(&work_lock);
	cur_hpd = work_state.edp->cur_hpd;
	
	pr_err("%s (tid %p): state %d (%s), hpd %d, pending_hpd_evt %d\n",
		__func__, current, work_state.state,
		state_names[work_state.state], cur_hpd, pending_hpd_evt);

	if (pending_hpd_evt) {
		/* If we were woken up because of HPD activity, just schedule
		 * the next appropriate task and get out.
		 */
		edp_state_machine_handle_hpd_l(cur_hpd);
	} else if (work_state.state < ARRAY_SIZE(state_machine_dispatch)) {
		dispatch_func_t func = state_machine_dispatch[work_state.state];

		if (NULL == func)
			pr_warn("NULL state machine handler while in state %d; how did we end up here?",
				work_state.state);
		else
			func(work_state.edp);
	} else {
		pr_warn("edp state machine worker scheduled unexpected state %d",
			work_state.state);
	}
}

/************************************************************
 *
 * state machine API implementation
 *
 ************************************************************/
void edp_state_machine_init(struct tegra_dc_dp_data *edp)
{
	work_state.edp = edp;
	work_state.state = EDP_STATE_INIT_FROM_BOOTLOADER;
	work_state.pending_hpd_evt = 1;
	work_state.shutdown = 0;
	INIT_DELAYED_WORK(&work_state.dwork, edp_state_machine_worker);
}

void edp_state_machine_shutdown(void)
{
	work_state.shutdown = 1;
	cancel_delayed_work_sync(&work_state.dwork);
}

void edp_state_machine_set_pending_hpd(void)
{
	rt_mutex_lock(&work_lock);

	/* We always schedule work any time there is a pending HPD event */
	/* But only if the state machine has been inited */
	if (likely(work_state.edp)) {
		work_state.pending_hpd_evt = 1;
		edp_state_machine_sched_work_l(0);
	}

	rt_mutex_unlock(&work_lock);
}

int edp_state_machine_get_state(void)
{
	int ret;

	rt_mutex_lock(&work_lock);
	ret = work_state.state;
	rt_mutex_unlock(&work_lock);

	return ret;
}
