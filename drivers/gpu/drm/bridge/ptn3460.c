/*
 * NXP PTN3460 DP/LVDS bridge driver
 *
 * Copyright (C) 2013 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define PTN3460_EDID_ADDR			0x0
#define PTN3460_EDID_EMULATION_ADDR		0x84
#define PTN3460_EDID_ENABLE_EMULATION		0
#define PTN3460_EDID_EMULATION_SELECTION	1
#define PTN3460_EDID_SRAM_LOAD_ADDR		0x85

struct ptn3460_bridge {
	struct i2c_client *client;
	int gpio_pd_n;
	int gpio_rst_n;
	int edid_emulation;
	bool enabled;
};

static int ptn3460_read_bytes(struct ptn3460_bridge *ptn_bridge, char addr,
		u8 *buf, int len)
{
	int ret;

	ret = i2c_master_send(ptn_bridge->client, &addr, 1);
	if (ret <= 0) {
		dev_err(&ptn_bridge->client->dev,"Failed to send i2c command, ret=%d\n", ret);
		return ret;
	}

	ret = i2c_master_recv(ptn_bridge->client, buf, len);
	if (ret <= 0) {
		dev_err(&ptn_bridge->client->dev,"Failed to recv i2c data, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ptn3460_write_byte(struct ptn3460_bridge *ptn_bridge, char addr,
		char val)
{
	int ret;
	char buf[2];

	buf[0] = addr;
	buf[1] = val;

	ret = i2c_master_send(ptn_bridge->client, buf, ARRAY_SIZE(buf));
	if (ret <= 0) {
		dev_err(&ptn_bridge->client->dev,"Failed to send i2c command, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ptn3460_select_edid(struct ptn3460_bridge *ptn_bridge)
{
	int ret;
	char val;

	/* Load the selected edid into SRAM (accessed at PTN3460_EDID_ADDR) */
	ret = ptn3460_write_byte(ptn_bridge, PTN3460_EDID_SRAM_LOAD_ADDR,
			ptn_bridge->edid_emulation);
	if (ret) {
		dev_err(&ptn_bridge->client->dev,"Failed to transfer edid to sram, ret=%d\n", ret);
		return ret;
	}

	/* Enable EDID emulation and select the desired EDID */
	val = 1 << PTN3460_EDID_ENABLE_EMULATION |
		ptn_bridge->edid_emulation << PTN3460_EDID_EMULATION_SELECTION;

	ret = ptn3460_write_byte(ptn_bridge, PTN3460_EDID_EMULATION_ADDR, val);
	if (ret) {
		dev_err(&ptn_bridge->client->dev,"Failed to write edid value, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static void ptn3460_enable(struct ptn3460_bridge *ptn_bridge)
{
	int ret;

	if (ptn_bridge->enabled)
		return;

	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_set_value(ptn_bridge->gpio_pd_n, 1);

	if (gpio_is_valid(ptn_bridge->gpio_rst_n)) {
		gpio_set_value(ptn_bridge->gpio_rst_n, 0);
		udelay(10);
		gpio_set_value(ptn_bridge->gpio_rst_n, 1);
	}

	/*
	 * There's a bug in the PTN chip where it falsely asserts hotplug before
	 * it is fully functional. We're forced to wait for the maximum start up
	 * time specified in the chip's datasheet to make sure we're really up.
	 */
	msleep(90);

	ret = ptn3460_select_edid(ptn_bridge);
	if (ret)
		dev_err(&ptn_bridge->client->dev,"Select edid failed ret=%d\n", ret);

	ptn_bridge->enabled = true;
}

static void ptn3460_disable(struct ptn3460_bridge *ptn_bridge)
{
	if (!ptn_bridge->enabled)
		return;

	ptn_bridge->enabled = false;

	/* hold chip in reset to disable it because PD isn't connected */
	if (gpio_is_valid(ptn_bridge->gpio_rst_n))
		gpio_set_value(ptn_bridge->gpio_rst_n, 0);

	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_set_value(ptn_bridge->gpio_pd_n, 0);
}

static int ptn3460_remove(struct i2c_client *client)
{
	struct ptn3460_bridge *ptn_bridge = i2c_get_clientdata(client);

	ptn3460_disable(ptn_bridge);
	
	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_free(ptn_bridge->gpio_pd_n);
	if (gpio_is_valid(ptn_bridge->gpio_rst_n))
		gpio_free(ptn_bridge->gpio_rst_n);
	/* Nothing else to free, we've got devm allocated memory */
	return 0;
}

int ptn3460_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct ptn3460_bridge *ptn_bridge;
	struct device_node *node;

	dev_info(&client->dev,"%s starting\n",__func__);

	ptn_bridge = devm_kzalloc(&client->dev, sizeof(struct ptn3460_bridge), GFP_KERNEL);
	if (!ptn_bridge) {
		dev_err(&client->dev,"Failed to allocate ptn bridge\n");
		return -ENOMEM;
	}
	ptn_bridge->client = client;
	i2c_set_clientdata(client,ptn_bridge);
	
	node = client->dev.of_node;

	ptn_bridge->gpio_pd_n = of_get_named_gpio(node, "powerdown-gpio", 0);
	if (gpio_is_valid(ptn_bridge->gpio_pd_n)) {
		ret = gpio_request_one(ptn_bridge->gpio_pd_n,
				GPIOF_OUT_INIT_HIGH, "PTN3460_PD_N");
		if (ret) {
			dev_err(&client->dev,"Request powerdown-gpio failed (%d)\n", ret);
			return ret;
		}
	}

	ptn_bridge->gpio_rst_n = of_get_named_gpio(node, "reset-gpio", 0);
	if (gpio_is_valid(ptn_bridge->gpio_rst_n)) {
		/*
		 * Request the reset pin low to avoid the bridge being
		 * initialized prematurely
		 */
		ret = gpio_request_one(ptn_bridge->gpio_rst_n,
				GPIOF_OUT_INIT_LOW, "PTN3460_RST_N");
		if (ret) {
			dev_err(&client->dev,"Request reset-gpio failed (%d)\n", ret);
			gpio_free(ptn_bridge->gpio_pd_n);
			return ret;
		}
	}

	ret = of_property_read_u32(node, "edid-emulation",
			&ptn_bridge->edid_emulation);
	if (ret) {
		dev_err(&client->dev,"Can't read edid emulation value\n");
		goto err;
	}

	ptn3460_enable(ptn_bridge);
	
	dev_info(&client->dev,"%s completed OK\n",__func__);
	return 0;

err:
	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_free(ptn_bridge->gpio_pd_n);
	if (gpio_is_valid(ptn_bridge->gpio_rst_n))
		gpio_free(ptn_bridge->gpio_rst_n);
	dev_err(&client->dev,"%s failed\n",__func__);
	return ret;
}

static const struct i2c_device_id ptn3460_id_table[] = {
	{ "ptn3460", 0},
	{ }
}; 
MODULE_DEVICE_TABLE(i2c, ptn3460_id_table);

static struct i2c_driver ptn3460_driver = {
	.driver = {
		.name = "ptn3460",
	},
	.probe = ptn3460_probe,
	.remove = ptn3460_remove,
	.id_table = ptn3460_id_table,
};

module_i2c_driver(ptn3460_driver);

/* 
 * Get rid of taint message by declaring code as GPL. 
 */
MODULE_LICENSE("GPL");


MODULE_DESCRIPTION("NXP PTN3460 eDP to LVDS driver");	/* What does this module do */

