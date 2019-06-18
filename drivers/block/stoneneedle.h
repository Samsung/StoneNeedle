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

#include <linux/nvme.h>
#include <linux/blkdev.h>

/* StoneNeedle operation interfaces */
struct stoneneedle_ops {
	/* statistic calculation function */
	void (*calc_stoneneedle) (char *, struct nvme_command, struct bio *);
	/* cmd and data interface via proc */
	int (*setup_stoneneedle) (struct gendisk *, const char *);
	void (*release_stoneneedle) (const char *);
};

int register_stoneneedle(int, struct stoneneedle_ops *);

void unregister_stoneneedle(struct stoneneedle_ops *);

#endif
