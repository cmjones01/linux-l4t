/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <generated/mach-types.h>
#include "board.h"

#include <media/soc_camera.h>
#include <media/tegra_v4l2_camera.h>
#include "devices.h"

#include "board-p1852.h"

static struct board_info board_info;

static int p1852_camera_init(void)
{
	return 0;
}

static int p1852_camera_enable(struct nvhost_device *ndev)
{
	return 0;
}

static void p1852_camera_disable(struct nvhost_device *ndev)
{
}

static struct i2c_board_info p1852_i2c8_board_info[] = {
	{
		I2C_BOARD_INFO("adv7180", 0x21),
	},
};

static struct soc_camera_link adv7180_iclink = {
	.bus_id		= -1, /* This must match the .id of tegra_vi01_device */
	.i2c_adapter_id	= 0,
	.board_info	= p1852_i2c8_board_info,
	.module_name	= "adv7180",
};

static struct platform_device soc_camera = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.platform_data = &adv7180_iclink,
	},
};

static struct tegra_camera_platform_data p1852_camera_platform_data = {
	.enable_camera		= p1852_camera_enable,
	.disable_camera		= p1852_camera_disable,
	.flip_v			= 0,
	.flip_h			= 0,
	.port			= TEGRA_CAMERA_PORT_VIP,
};

int __init p1852_sensors_init(void)
{
	tegra_camera_device.dev.platform_data = &p1852_camera_platform_data;

	p1852_camera_init();

	/* V4L2 initialization stuff. */
	nvhost_device_register(&tegra_camera_device);

	platform_device_register(&soc_camera);

	return 0;
}