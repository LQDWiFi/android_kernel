/* linux/himax8526a.h
 *
 * Copyright (C) 2013 MM Solutions.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef HIMAX8526a_H
#define HIMAX8526a_H
#include <linux/types.h>
#include <linux/i2c.h>

#define HIMAX8526A_NAME "himax8526a"

struct himax_regulator {
	const char *name;
	u32	max_uV;
	u32	min_uV;
	u32	hpm_load_uA;
	u32	lpm_load_uA;
};

struct himax_i2c_platform_data {
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_width_min;
	int abs_width_max;
	int inv_x;
	int inv_y;
	int (*power)(int on);
	struct himax_regulator *regulator_info;
	u8 num_regulators;
	struct himax_ts_config_data *cdata;
	int gpio_irq;
	void (*reset)(void);
    int reset_gpio;
	u32 reset_gpio_flags;
};

struct himax_ts_config_data {
	uint8_t c1[5];
	uint8_t c2[2];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[11];
	uint8_t c14[11];
	uint8_t c15[11];
	uint8_t c16[11];
	uint8_t c17[11];
	uint8_t c18[2];
	uint8_t c19[4];
	uint8_t c20[9];
	uint8_t c21[5];
	uint8_t c22[15];
	uint8_t c23[3];
	uint8_t c24[2];
	uint8_t c25[2];
	uint8_t c26[5];
	uint8_t c27[3];
	uint8_t c28[9];
	uint8_t c29[11];
	uint8_t c30[4];
	uint8_t c31[65];
	uint8_t c32[13];
	uint8_t c33[3];
	uint8_t c34[3];
	uint8_t c35[17];
	uint8_t c36[137];
};

#endif
