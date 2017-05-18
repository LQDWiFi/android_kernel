/*
 * IFC6640 generic led  gpio configuration driver
 * TBD as per requirement
 * Author: Arjun Prasad <arjun.prasad@inforcecomputing.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/slab.h>


static int __init dgpioinit(void) {

/* General purpose leds */
	gpio_request(146, "led-1");
	gpio_direction_output(146, 1);
	gpio_request(147, "led-2");
	gpio_direction_output(147, 1);
	gpio_request(148, "led-3");
	gpio_direction_output(148, 1);
	gpio_request(149, "led-4");
	gpio_direction_output(149, 1);
/* PoE */
	gpio_request(55, "PoE_1");
	gpio_export(55, true);
	gpio_direction_output(55, 0);

	return 0;
}

static void __exit dgpioexit(void) {
	/* TBD : Modify as per requirement */
	printk(KERN_INFO "%s:Module exit\n", __func__);
}

module_init(dgpioinit);
module_exit(dgpioexit);

MODULE_DESCRIPTION("GPIO configuration driver");
