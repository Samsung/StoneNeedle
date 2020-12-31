/*
 * StoneNeedle IO pattern module interface
 * Copyright (c) 2019, SAMSUNG ELECTRONICS.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef _STONENEEDLE_H
#define _STONENEEDLE_H

#define    DEV_NAME_LEN                     (20)

#include <linux/blkdev.h>
#include "nvme.h"

/* StoneNeedle operation interfaces */
struct stoneneedle_ops {
	/* statistic calculation function */
	void (*calc_stoneneedle) (struct nvme_ns *, struct request *, int);
	/* cmd and data interface via proc */
	int (*setup_stoneneedle) (struct gendisk *, const char *);
	void (*release_stoneneedle) (const char *);
};

#endif
