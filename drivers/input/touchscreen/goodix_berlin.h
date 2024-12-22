/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Goodix Touchscreen Driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 * Copyright (C) 2023 Linaro Ltd.
 *
 * Based on goodix_berlin_berlin driver.
 */

#ifndef __GOODIX_BERLIN_H_
#define __GOODIX_BERLIN_H_

#include <linux/pm.h>

enum goodix_berlin_ic_type {
	IC_TYPE_BERLIN_A,
	IC_TYPE_BERLIN_D,
};

struct goodix_berlin_ic_data {
	enum goodix_berlin_ic_type ic_type;
};

struct device;
struct input_id;
struct regmap;

int goodix_berlin_probe(struct device *dev, int irq, const struct input_id *id,
			struct regmap *regmap);

extern const struct dev_pm_ops goodix_berlin_pm_ops;
extern const struct attribute_group *goodix_berlin_groups[];

#endif
