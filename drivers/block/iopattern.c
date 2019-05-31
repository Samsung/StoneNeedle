/*
 * StoneNeedle IO pattern module interface
 * Copyright (c) 2019, SAMSUNG ELECTRONICS.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/blkdev.h>
#include <linux/bitops.h>

#include "iopattern.h"

#define    DEV_NAME_LEN                     (20)
#define    BUFFER_LEN                       (32)
#define    IOPATTERN_4KB_CHUNK              (4*1024)
#define    IOPATTERN_READ_SKEWNESS          (100)
#define    IOPATTERN_NR_IOSIZE              (128/4)
#define    IOPATTERN_TOTAL_IOSIZE           (IOPATTERN_NR_IOSIZE+2)	/* 32 common + 2 special */
#define    IOPATTERN_READ                   (0)
#define    IOPATTERN_WRITE                  (1)
#define    IOPATTERN_INTERVAL_BUCKET_NUM    (25)
#define    IOPATTERN_SUPPORT_MAX_DEV        (24+1)	/* controller + 24*ssd */

/* iopattern flags */
#define    IOPATTERN_MONITOR_STATUS         (0)	/* device monitor status bit:bit0 */
#define    IOPATTERN_FIRST_READ             (1)	/* first read flag on device: bit1 */
#define    IOPATTERN_FIRST_WRITE            (2)	/* first write flag on device: bit2 */

enum iopattern_index_item {
	IOPATTERN_WRITE_IO_COUNT = 0,
	IOPATTERN_READ_IO_COUNT,
	IOPATTERN_TOTAL_WRITE_BYTES,
	IOPATTERN_TOTAL_READ_BYTES,
	IOPATTERN_WRITE_STRIDE,
	IOPATTERN_READ_STRIDE,
	IOPATTERN_WRITE_IO_ARRIVAL_INTERVAL,
	IOPATTERN_READ_IO_ARRIVAL_INTERVAL,
	IOPATTERN_WRITE_SEQ_SECTOR_BYTES,
	IOPATTERN_READ_SEQ_SECTOR_BYTES,
	IOPATTERN_RANDOM_WRITE_IO_COUNT,
	IOPATTERN_RANDOM_READ_IO_COUNT,
	IOPATTERN_WRITE_CHUNKS,
	IOPATTERN_WRITE_128TO256_CHUNKS,
	IOPATTERN_WRITE_EX256KB_CHUNKS,
	IOPATTERN_READ_CHUNKS,
	IOPATTERN_READ_128TO256_CHUNKS,
	IOPATTERN_READ_EX256KB_CHUNKS,
	IOPATTERN_INTERVAL_IN_CHUNK,
	IOPATTERN_READ_SEQ_COUNT_PER_CHUNK,
	IOPATTERN_WRITE_SEQ_COUNT_PER_CHUNK,
	IOPATTERN_READ_SEQ_BYTES_PER_CHUNK,
	IOPATTERN_WRITE_SEQ_BYTES_PER_CHUNK,
	IOPATTERN_READ_COUNT_PER_CHUNK,
	IOPATTERN_WRITE_COUNT_PER_CHUNK,
	/* add other paramter here */
	IOPATTERN_MAX,
};

const char *const device_status[] = { "stop", "start" };

const char *const controller_file = "controller";
const char *const user_root_dir = "stoneneedle";
const char *const iopattern_text[] = {
	/* enum iopattern */
	"WRITE_IO_COUNT:",
	"READ_IO_COUNT:",
	"WRITE_TOTAL_BYTES:",
	"READ_TOTAL_BYTES:",
	"WRITE_STRIDE_SECTORS:",
	"READ_STRIDE_SECTORS:",
	"WRITE_IO_ARRIVAL_INTERVAL:",
	"READ_IO_ARRIVAL_INTERVAL:",
	"WRITE_SEQUETIAL_BYTES:",
	"READ_SEQUETIAL_BYTES:",
	"WRITE_RANDOM_IO_COUNT:",
	"READ_RANDOM_IO_COUNT:",
	"KB_WRITE_SKEWNESS:",
	"128KB_WRITE_SKEWNESS:",
	"256KB_WRITE_SKEWNESS:",
	"KB_READ_SKEWNESS:",
	"128KB_READ_SKEWNESS:",
	"256KB_READ_SKEWNESS:",
	"WRITE_IO_ARRIVAL_INTERVAL",
	"READ_SEQUETIAL_COUNT_PER_CHUNK:",
	"WRITE_SEQUETIAL_COUNT_PER_CHUNK:",
	"READ_SEQUETIAL_BYTES_PER_CHUNK:",
	"WRITE_SEQUETIAL_BYTES_PER_CHUNK:",
	"READ_COUNT_PER_CHUNK:",
	"WRITE_COUNT_PER_CHUNK:",
};

struct histlog2 {
	int first;
	int delta;
	int num;
};

struct iopattern_data {
	unsigned long iopattern_value[IOPATTERN_MAX];
	unsigned long iopattern_value_pre[IOPATTERN_MAX];
	unsigned long **read_skewness;
	unsigned long **write_skewness;
	unsigned long **interval_in_chunks;
	unsigned long *read_seq_count_per_chunk;
	unsigned long *write_seq_count_per_chunk;
	unsigned long *read_seq_bytes_per_chunk;
	unsigned long *write_seq_bytes_per_chunk;
	unsigned long *read_count_per_chunk;
	unsigned long *write_count_per_chunk;
	sector_t read_bucket_size;
	sector_t write_bucket_size;
	spinlock_t lock;
};

struct iopattern_dev {
	/* iopattern data */
	struct iopattern_data dev_data;
	char dev_name[DEV_NAME_LEN];
	unsigned long iopattern_flags;
	unsigned long previous_time;
	spinlock_t lock;
	struct list_head list;
};

struct iopattern_dev_mgmt {
	struct proc_dir_entry *iopattern_root;
	struct iopattern_dev *cur_open_dev;
	struct list_head iopattern_dev_list;
	struct histlog2 interval_hist;
	int iopattern_dev_num;
	int iopattern_chunk_size;
};

static struct iopattern_dev_mgmt *dev_mgmt;

static struct iopattern_dev *find_iopattern_dev(const char *dev_name)
{
	struct iopattern_dev *iopatt_dev;
	list_for_each_entry(iopatt_dev, &dev_mgmt->iopattern_dev_list, list) {
		if (strstr(dev_name, iopatt_dev->dev_name) != NULL)
			return iopatt_dev;
	}
	return NULL;
}

static void *iopattern_start(struct seq_file *m, loff_t *pos)
{
	unsigned long *base;
	int i;

	struct iopattern_dev *cur_iopattern_dev;
	struct iopattern_data *io_data;

	cur_iopattern_dev = dev_mgmt->cur_open_dev;
	io_data = &cur_iopattern_dev->dev_data;

	if (strstr(cur_iopattern_dev->dev_name, controller_file) != NULL) {
		if (*pos >= 1)
			return NULL;
	}
	if (*pos >= ARRAY_SIZE(iopattern_text))
		return NULL;

	base = kzalloc(sizeof(unsigned long) * IOPATTERN_MAX, GFP_ATOMIC);
	m->private = base;

	if (!base)
		return ERR_PTR(-ENOMEM);

	if (cur_iopattern_dev) {
		for (i = 0; i < IOPATTERN_MAX; i++)
			base[i] = io_data->iopattern_value[i];

		base += IOPATTERN_MAX;
	}

	return (unsigned long *)m->private + *pos;
}

static void *iopattern_next(struct seq_file *m, void *arg, loff_t *pos)
{
	struct iopattern_dev *cur_iopattern_dev;

	cur_iopattern_dev = dev_mgmt->cur_open_dev;
	(*pos)++;
	if (strstr(cur_iopattern_dev->dev_name, controller_file) != NULL) {
		if (*pos >= 1)
			return NULL;
	} else {
		if (*pos >= ARRAY_SIZE(iopattern_text))
			return NULL;
	}
	return (unsigned long *)m->private + *pos;
}

static int iopattern_row_show(struct seq_file *m, unsigned long *row_data,
			      int chunk_size)
{
	int i;
	for (i = 0; i < chunk_size - 1; i++)
		seq_printf(m, "%lu,", row_data[i]);

	seq_printf(m, "%lu\n", row_data[chunk_size - 1]);
	return 0;
}

static int iopattern_skeness_normal_show(struct seq_file *m, int offset,
					 struct iopattern_data *io_data)
{
	int j, chunk_size;
	unsigned long **skewness;

	if (IOPATTERN_WRITE_CHUNKS == offset) {
		skewness = io_data->write_skewness;
		chunk_size = dev_mgmt->iopattern_chunk_size;
	} else {
		skewness = io_data->read_skewness;
		chunk_size = IOPATTERN_READ_SKEWNESS;
	}

	for (j = 0; j < IOPATTERN_NR_IOSIZE; j++) {
		seq_printf(m, "%d%s ", j * 4, iopattern_text[offset]);
		iopattern_row_show(m, skewness[j], chunk_size);
	}
	return 0;
}

static int iopattern_128to256kb_show(struct seq_file *m, int offset,
				     struct iopattern_data *io_data)
{
	int chunk_size;
	unsigned long **skewness;

	if (IOPATTERN_WRITE_128TO256_CHUNKS == offset) {
		skewness = io_data->write_skewness;
		chunk_size = dev_mgmt->iopattern_chunk_size;
	} else {
		skewness = io_data->read_skewness;
		chunk_size = IOPATTERN_READ_SKEWNESS;
	}

	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, skewness[IOPATTERN_NR_IOSIZE], chunk_size);
	return 0;
}

static int iopattern_interval_bucket_show(struct seq_file *m, int offset,
					  struct iopattern_data *io_data)
{
	int j;
	int chunk_size;
	unsigned long **skewness;

	skewness = io_data->interval_in_chunks;
	chunk_size = dev_mgmt->iopattern_chunk_size;

	for (j = 0; j < IOPATTERN_INTERVAL_BUCKET_NUM; j++) {
		seq_printf(m, "%s_BUCKET%d: ", iopattern_text[offset], j);
		iopattern_row_show(m, skewness[j], chunk_size);
	}
	return 0;
}

static int iopattern_ex256kb_show(struct seq_file *m, int offset,
				  struct iopattern_data *io_data)
{
	int chunk_size;
	unsigned long **skewness;

	if (IOPATTERN_WRITE_EX256KB_CHUNKS == offset) {
		skewness = io_data->write_skewness;
		chunk_size = dev_mgmt->iopattern_chunk_size;
	} else {
		skewness = io_data->read_skewness;
		chunk_size = IOPATTERN_READ_SKEWNESS;
	}

	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, skewness[IOPATTERN_NR_IOSIZE + 1], chunk_size);
	return 0;
}

static int iopattern_read_seq_count_chunk_show(struct seq_file *m, int offset,
					       struct iopattern_data *io_data)
{
	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, io_data->read_seq_count_per_chunk,
			   IOPATTERN_READ_SKEWNESS);

	return 0;
}

static int iopattern_write_seq_count_chunk_show(struct seq_file *m, int offset,
						struct iopattern_data *io_data)
{
	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, io_data->write_seq_count_per_chunk,
			   dev_mgmt->iopattern_chunk_size);

	return 0;
}

static int iopattern_read_seq_bytes_chunk_show(struct seq_file *m, int offset,
					       struct iopattern_data *io_data)
{
	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, io_data->read_seq_bytes_per_chunk,
			   IOPATTERN_READ_SKEWNESS);

	return 0;
}

static int iopattern_write_seq_bytes_chunk_show(struct seq_file *m, int offset,
						struct iopattern_data *io_data)
{
	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, io_data->write_seq_bytes_per_chunk,
			   dev_mgmt->iopattern_chunk_size);

	return 0;
}

static int iopattern_read_count_chunk_show(struct seq_file *m, int offset,
					   struct iopattern_data *io_data)
{
	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, io_data->read_count_per_chunk,
			   IOPATTERN_READ_SKEWNESS);

	return 0;
}

static int iopattern_write_count_chunk_show(struct seq_file *m, int offset,
					    struct iopattern_data *io_data)
{
	seq_printf(m, "%s ", iopattern_text[offset]);
	iopattern_row_show(m, io_data->write_count_per_chunk,
			   dev_mgmt->iopattern_chunk_size);

	return 0;
}

static int iopattern_common_show(struct seq_file *m, int offset,
				 struct iopattern_data *io_data)
{
	seq_printf(m, "%s %lu\n", iopattern_text[offset],
		   io_data->iopattern_value[offset]);
	return 0;
}

typedef int (*oper_func[IOPATTERN_MAX]) (struct seq_file *, int,
					 struct iopattern_data *);
oper_func iopattern_show_list = {
	iopattern_common_show,	/*"WRITE_IO_COUNT:", */
	iopattern_common_show,	/*"READ_IO_COUNT:", */
	iopattern_common_show,	/*"WRITE_TOTAL_BYTES:", */
	iopattern_common_show,	/*"READ_TOTAL_BYTES:", */
	iopattern_common_show,	/*"WRITE_STRIDE_SECTORS:", */
	iopattern_common_show,	/*"READ_STRIDE_SECTORS:", */
	iopattern_common_show,	/*"WRITE_IO_ARRIVAL_INTERVAL:", */
	iopattern_common_show,	/*"READ_IO_ARRIVAL_INTERVAL:", */
	iopattern_common_show,	/*"WRITE_SEQUETIAL_SECTOR_COUNT:", */
	iopattern_common_show,	/*"READ_SEQUETIAL_SECTOR_COUNT:", */
	iopattern_common_show,	/*"WRITE_RANDOM_IOS:",    */
	iopattern_common_show,	/*"READ_RANDOM_IOS:",     */
	iopattern_skeness_normal_show,	/*"KB_WRITE_SKEWNESS:",   */
	iopattern_128to256kb_show,	/*"128KB_WRITE_SKEWNESS:", */
	iopattern_ex256kb_show,	/*"256KB_WRITE_SKEWNESS:", */
	iopattern_skeness_normal_show,	/*"KB_READ_SKEWNESS:",    */
	iopattern_128to256kb_show,	/*"128KB_READ_SKEWNESS:", */
	iopattern_ex256kb_show,	/*"256KB_READ_SKEWNESS:", */
	iopattern_interval_bucket_show,	/*"IO_ARRIVAL_INTERVAL",  */
	iopattern_read_seq_count_chunk_show,	/*"READ_SEQUETIAL_COUNT_PER_CHUNK:", */
	iopattern_write_seq_count_chunk_show,	/*"WRITE_SEQUETIAL_COUNT_PER_CHUNK:", */
	iopattern_read_seq_bytes_chunk_show,	/*"READ_SEQUETIAL_BYTES_PER_CHUNK:", */
	iopattern_write_seq_bytes_chunk_show,	/*"WRITE_SEQUETIAL_BYTES_PER_CHUNK:", */
	iopattern_read_count_chunk_show,	/*"READ_COUNT_PER_CHUNK:", */
	iopattern_write_count_chunk_show,	/*"WRITE_COUNT_PER_CHUNK:", */
};

static int iopattern_show(struct seq_file *m, void *arg)
{
	unsigned long *input = arg;
	unsigned long offset = input - (unsigned long *)m->private;
	struct iopattern_dev *cur_iopattern_dev;
	struct iopattern_data *io_data;

	cur_iopattern_dev = dev_mgmt->cur_open_dev;
	io_data = &cur_iopattern_dev->dev_data;

	if (strstr(cur_iopattern_dev->dev_name, controller_file)) {
		struct iopattern_dev *iopatt_dev;
		list_for_each_entry(iopatt_dev, &dev_mgmt->iopattern_dev_list,
				    list) {
			if (!strstr(iopatt_dev->dev_name, controller_file))
				seq_printf(m, "%s\t%s\n", iopatt_dev->dev_name,
					   device_status[test_bit
							 (IOPATTERN_MONITOR_STATUS,
							  &iopatt_dev->
							  iopattern_flags)]);
		}
	} else {
		iopattern_show_list[offset] (m, offset, io_data);
	}
	return 0;
}

static void iopattern_stop(struct seq_file *m, void *arg)
{
	kfree(m->private);
	m->private = NULL;
}

static const struct seq_operations iopattern_op = {
	.start = iopattern_start,
	.next = iopattern_next,
	.stop = iopattern_stop,
	.show = iopattern_show,
};

static void iopattern_dev_data_clear(struct iopattern_dev *iopatt_dev)
{
	int i;
	memset(iopatt_dev->dev_data.iopattern_value, 0,
	       sizeof(unsigned long) * IOPATTERN_MAX);
	memset(iopatt_dev->dev_data.iopattern_value_pre, 0,
	       sizeof(unsigned long) * IOPATTERN_MAX);
	for (i = 0; i < IOPATTERN_TOTAL_IOSIZE; i++) {
		memset(iopatt_dev->dev_data.read_skewness[i], 0,
		       sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS);
		memset(iopatt_dev->dev_data.write_skewness[i], 0,
		       sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size);
	}
	for (i = 0; i < IOPATTERN_INTERVAL_BUCKET_NUM; i++) {
		memset(iopatt_dev->dev_data.interval_in_chunks[i], 0,
		       sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size);
	}
	memset(iopatt_dev->dev_data.read_seq_count_per_chunk, 0,
	       sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS);
	memset(iopatt_dev->dev_data.write_seq_count_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size);
	memset(iopatt_dev->dev_data.read_seq_bytes_per_chunk, 0,
	       sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS);
	memset(iopatt_dev->dev_data.write_seq_bytes_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size);
	memset(iopatt_dev->dev_data.read_count_per_chunk, 0,
	       sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS);
	memset(iopatt_dev->dev_data.write_count_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size);
	return;
}

static ssize_t iopattern_controller(struct file *filp, const char __user *buf,
				    size_t count, loff_t *f_ops)
{
	char buffer[BUFFER_LEN] = { 0 };
	struct iopattern_dev *iopatt_dev;

	if (copy_from_user
	    (buffer, buf, (count > BUFFER_LEN ? BUFFER_LEN : count)))
		goto out;

	iopatt_dev = find_iopattern_dev(buffer);
	if (!iopatt_dev)
		goto out;

	spin_lock(&iopatt_dev->lock);
	if (strstr(buffer, device_status[1])
	    && !test_bit(IOPATTERN_MONITOR_STATUS,
			 &iopatt_dev->iopattern_flags)) {
		set_bit(IOPATTERN_MONITOR_STATUS, &iopatt_dev->iopattern_flags);
		set_bit(IOPATTERN_FIRST_READ, &iopatt_dev->iopattern_flags);
		set_bit(IOPATTERN_FIRST_WRITE, &iopatt_dev->iopattern_flags);
		iopatt_dev->previous_time = 0;

		/* clear iopattern dev data */
		spin_lock(&iopatt_dev->dev_data.lock);
		iopattern_dev_data_clear(iopatt_dev);
		spin_unlock(&iopatt_dev->dev_data.lock);
	} else if (strstr(buffer, device_status[0])) {
		clear_bit(IOPATTERN_MONITOR_STATUS,
			  &iopatt_dev->iopattern_flags);
	} else {
		pr_err("StoneNeedle: error input to %s controller.\n",
		       iopatt_dev->dev_name);
	}
	spin_unlock(&iopatt_dev->lock);
out:
	return count;
}

static int iopattern_open(struct inode *inode, struct file *file)
{
	dev_mgmt->cur_open_dev =
	    find_iopattern_dev(file->f_path.dentry->d_iname);
	if (dev_mgmt->cur_open_dev)
		pr_debug("StoneNeedle: open device %s proc file.\n",
			 dev_mgmt->cur_open_dev->dev_name);
	return seq_open(file, &iopattern_op);
}

static const struct file_operations proc_iopattern_file_operations = {
	.open = iopattern_open,
	.read = seq_read,
	.write = iopattern_controller,
	.llseek = seq_lseek,
	.release = seq_release,
};

static unsigned int calc_rw_stride(enum iopattern_index_item index,
				   struct nvme_command cmnd,
				   struct iopattern_dev *iopatt_dev)
{
	unsigned long stride_diff = 0;
	struct iopattern_data *io_data;

	io_data = &iopatt_dev->dev_data;

	spin_lock(&iopatt_dev->dev_data.lock);
	if (cmnd.rw.slba >= io_data->iopattern_value_pre[index])
		stride_diff =
		    cmnd.rw.slba - io_data->iopattern_value_pre[index];
	else
		stride_diff =
		    io_data->iopattern_value_pre[index] - cmnd.rw.slba;

	io_data->iopattern_value[index] += stride_diff;

	/* if the bs=512, cmnd.rw.length=0, so we need add 1 to cmnd.rw.length */
	io_data->iopattern_value_pre[index] =
	    cmnd.rw.slba + (cmnd.rw.length + 1);
	spin_unlock(&iopatt_dev->dev_data.lock);

	return stride_diff;
}

static void calc_io_arrival_interval(struct iopattern_dev *iopatt_dev, int op)
{
	struct timeval tv;
	struct iopattern_data *io_data;

	io_data = &iopatt_dev->dev_data;
	/* get time */
	do_gettimeofday(&tv);
	if (op == IOPATTERN_WRITE) {
		if (test_bit
		    (IOPATTERN_FIRST_WRITE, &iopatt_dev->iopattern_flags)) {
			spin_lock(&iopatt_dev->dev_data.lock);

			io_data->
			    iopattern_value_pre
			    [IOPATTERN_WRITE_IO_ARRIVAL_INTERVAL] =
			    timeval_to_ns(&tv) / 1000;
			spin_unlock(&iopatt_dev->dev_data.lock);
			spin_lock(&iopatt_dev->lock);
			clear_bit(IOPATTERN_FIRST_WRITE,
				  &iopatt_dev->iopattern_flags);
			spin_unlock(&iopatt_dev->lock);
		} else {
			spin_lock(&iopatt_dev->dev_data.lock);
			io_data->
			    iopattern_value[IOPATTERN_WRITE_IO_ARRIVAL_INTERVAL]
			    +=
			    timeval_to_ns(&tv) / 1000 -
			    io_data->
			    iopattern_value_pre
			    [IOPATTERN_WRITE_IO_ARRIVAL_INTERVAL];
			io_data->
			    iopattern_value_pre
			    [IOPATTERN_WRITE_IO_ARRIVAL_INTERVAL] =
			    timeval_to_ns(&tv) / 1000;
			spin_unlock(&iopatt_dev->dev_data.lock);
		}
	} else {
		if (test_bit
		    (IOPATTERN_FIRST_READ, &iopatt_dev->iopattern_flags)) {
			spin_lock(&iopatt_dev->dev_data.lock);
			io_data->
			    iopattern_value_pre
			    [IOPATTERN_READ_IO_ARRIVAL_INTERVAL] =
			    timeval_to_ns(&tv) / 1000;
			spin_unlock(&iopatt_dev->dev_data.lock);
			spin_lock(&iopatt_dev->lock);
			clear_bit(IOPATTERN_FIRST_READ,
				  &iopatt_dev->iopattern_flags);
			spin_unlock(&iopatt_dev->lock);
		} else {
			spin_lock(&iopatt_dev->dev_data.lock);
			io_data->
			    iopattern_value[IOPATTERN_READ_IO_ARRIVAL_INTERVAL]
			    +=
			    timeval_to_ns(&tv) / 1000 -
			    io_data->
			    iopattern_value_pre
			    [IOPATTERN_READ_IO_ARRIVAL_INTERVAL];
			io_data->
			    iopattern_value_pre
			    [IOPATTERN_READ_IO_ARRIVAL_INTERVAL] =
			    timeval_to_ns(&tv) / 1000;
			spin_unlock(&iopatt_dev->dev_data.lock);
		}
	}
}

static unsigned long histlog2_upper_limit(int index, struct histlog2 *h)
{
	return h->first + (index ? h->delta << (index - 1) : 0);
}

static int histlog2_index(unsigned long val, struct histlog2 *h)
{
	int i;

	for (i = 0; i < (h->num - 1) && val > histlog2_upper_limit(i, h); i++);
	return i;
}

static int calc_bucket_account(unsigned long *bucket, struct bio *bio,
			       sector_t bucket_size)
{
	int index = bio->bi_sector / bucket_size;
	bucket[index]++;
	return 0;
}

static void calc_write_chunk_interval(struct bio *bio,
				      struct iopattern_dev *iopatt_dev)
{
	unsigned long interval;
	int index;
	struct timeval tv;
	struct iopattern_data *io_data;

	io_data = &iopatt_dev->dev_data;
	/* get time */
	do_gettimeofday(&tv);
	interval = 0;

	if (iopatt_dev->previous_time)
		interval =
		    timeval_to_ns(&tv) / 1000 - iopatt_dev->previous_time;

	index = histlog2_index(interval, &dev_mgmt->interval_hist);
	iopatt_dev->previous_time = timeval_to_ns(&tv) / 1000;

	spin_lock(&iopatt_dev->dev_data.lock);
	calc_bucket_account(io_data->interval_in_chunks[index], bio,
			    io_data->write_bucket_size);
	spin_unlock(&iopatt_dev->dev_data.lock);
	return;
}

static unsigned int calc_chunk_index(unsigned int index)
{
	if (index >= IOPATTERN_NR_IOSIZE * 2) {
		index = IOPATTERN_NR_IOSIZE + 1;
	} else if (index > IOPATTERN_NR_IOSIZE
		   && index < IOPATTERN_NR_IOSIZE * 2) {
		index = IOPATTERN_NR_IOSIZE;
	}
	return index;
}

static void calc_write_iopattern(struct bio *bio, struct nvme_command cmnd,
				 struct iopattern_dev *iopatt_dev)
{
	unsigned long stride_diff;
	unsigned int index;
	unsigned int rq_bytes = bio->bi_size;
	struct iopattern_data *io_data;

	io_data = &iopatt_dev->dev_data;

	stride_diff = calc_rw_stride(IOPATTERN_WRITE_STRIDE, cmnd, iopatt_dev);
	spin_lock(&iopatt_dev->dev_data.lock);
	if (stride_diff != 0) {
		io_data->iopattern_value[IOPATTERN_RANDOM_WRITE_IO_COUNT]++;
	} else {
		io_data->iopattern_value[IOPATTERN_WRITE_SEQ_SECTOR_BYTES] +=
		    rq_bytes;
		calc_bucket_account(io_data->write_seq_count_per_chunk, bio,
				    io_data->write_bucket_size);
		io_data->write_seq_bytes_per_chunk[bio->bi_sector /
						   io_data->
						   write_bucket_size] +=
		    rq_bytes;
	}

	io_data->iopattern_value[IOPATTERN_WRITE_IO_COUNT]++;
	io_data->iopattern_value[IOPATTERN_TOTAL_WRITE_BYTES] += rq_bytes;
	calc_bucket_account(io_data->write_count_per_chunk, bio,
			    io_data->write_bucket_size);
	spin_unlock(&iopatt_dev->dev_data.lock);

	calc_io_arrival_interval(iopatt_dev, IOPATTERN_WRITE);

	index = calc_chunk_index(rq_bytes / IOPATTERN_4KB_CHUNK);
	spin_lock(&iopatt_dev->dev_data.lock);
	calc_bucket_account(io_data->write_skewness[index], bio,
			    io_data->write_bucket_size);
	spin_unlock(&iopatt_dev->dev_data.lock);
}

static void calc_read_iopattern(struct bio *bio, struct nvme_command cmnd,
				struct iopattern_dev *iopatt_dev)
{
	unsigned long stride_diff;
	unsigned int index;
	unsigned int rq_bytes = bio->bi_size;
	struct iopattern_data *io_data;
	io_data = &iopatt_dev->dev_data;

	/* Read stride */
	stride_diff = calc_rw_stride(IOPATTERN_READ_STRIDE, cmnd, iopatt_dev);

	/*
	 * Read random
	 start address of last io + last io length equal
	 to current start address
	 */
	spin_lock(&iopatt_dev->dev_data.lock);
	if (stride_diff != 0) {
		io_data->iopattern_value[IOPATTERN_RANDOM_READ_IO_COUNT]++;
	} else {
		io_data->iopattern_value[IOPATTERN_READ_SEQ_SECTOR_BYTES] +=
		    rq_bytes;
		calc_bucket_account(io_data->read_seq_count_per_chunk, bio,
				    io_data->read_bucket_size);
		io_data->read_seq_bytes_per_chunk[bio->bi_sector /
						  io_data->read_bucket_size] +=
		    rq_bytes;
	}

	io_data->iopattern_value[IOPATTERN_READ_IO_COUNT]++;
	io_data->iopattern_value[IOPATTERN_TOTAL_READ_BYTES] += rq_bytes;
	calc_bucket_account(io_data->read_count_per_chunk, bio,
			    io_data->read_bucket_size);

	spin_unlock(&iopatt_dev->dev_data.lock);

	calc_io_arrival_interval(iopatt_dev, IOPATTERN_READ);

	index = calc_chunk_index(rq_bytes / IOPATTERN_4KB_CHUNK);
	spin_lock(&iopatt_dev->dev_data.lock);
	calc_bucket_account(io_data->read_skewness[index], bio,
			    io_data->read_bucket_size);
	spin_unlock(&iopatt_dev->dev_data.lock);
}

void calc_iopattern(char *deviceName, struct nvme_command cmnd, struct bio *bio)
{
	struct iopattern_dev *iopatt_dev;
	struct iopattern_data *io_data;

	iopatt_dev = find_iopattern_dev(deviceName);

	if (iopatt_dev
	    && test_bit(IOPATTERN_MONITOR_STATUS,
			&iopatt_dev->iopattern_flags)) {
		io_data = &iopatt_dev->dev_data;
		if (nvme_cmd_write == cmnd.rw.opcode) {
			calc_write_iopattern(bio, cmnd, iopatt_dev);
			calc_write_chunk_interval(bio, iopatt_dev);
		} else if (nvme_cmd_read == cmnd.rw.opcode) {
			calc_read_iopattern(bio, cmnd, iopatt_dev);
		}
	}
}

static int create_iopattern_proc(const char *dev_name)
{
	if (!dev_mgmt->iopattern_root)
		dev_mgmt->iopattern_root = proc_mkdir(user_root_dir, NULL);

	proc_create(dev_name, S_IRUGO, dev_mgmt->iopattern_root,
		    &proc_iopattern_file_operations);
	return 0;
}

static int alloc_dev_data(struct iopattern_data *io_data, sector_t dev_capacity)
{
	int i, j, k;
	spin_lock_init(&io_data->lock);
	if (dev_capacity % IOPATTERN_READ_SKEWNESS != 0)
		io_data->read_bucket_size =
		    dev_capacity / IOPATTERN_READ_SKEWNESS + 1;
	else
		io_data->read_bucket_size =
		    dev_capacity / IOPATTERN_READ_SKEWNESS;
	if (dev_capacity % dev_mgmt->iopattern_chunk_size != 0)
		io_data->write_bucket_size =
		    dev_capacity / dev_mgmt->iopattern_chunk_size + 1;
	else
		io_data->write_bucket_size =
		    dev_capacity / dev_mgmt->iopattern_chunk_size;

	io_data->read_skewness =
	    kzalloc(sizeof(unsigned long *) * IOPATTERN_TOTAL_IOSIZE,
		    GFP_ATOMIC);
	if (!io_data->read_skewness)
		return -ENOMEM;
	for (i = 0; i < IOPATTERN_TOTAL_IOSIZE; i++) {
		io_data->read_skewness[i] =
		    kzalloc(sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS,
			    GFP_ATOMIC);
		if (!io_data->read_skewness[i])
			goto free_read_skewness;
	}

	io_data->write_skewness =
	    kzalloc(sizeof(unsigned long *) * IOPATTERN_TOTAL_IOSIZE,
		    GFP_ATOMIC);
	if (!io_data->write_skewness)
		goto free_read_skewness;

	for (j = 0; j < IOPATTERN_TOTAL_IOSIZE; j++) {
		io_data->write_skewness[j] =
		    kzalloc(sizeof(unsigned long) *
			    dev_mgmt->iopattern_chunk_size, GFP_ATOMIC);
		if (!io_data->write_skewness[j])
			goto free_write_skewness;
	}

	io_data->interval_in_chunks =
	    kzalloc(sizeof(unsigned long *) * IOPATTERN_INTERVAL_BUCKET_NUM,
		    GFP_ATOMIC);
	if (!io_data->interval_in_chunks)
		goto free_write_skewness;

	for (k = 0; k < IOPATTERN_INTERVAL_BUCKET_NUM; k++) {
		io_data->interval_in_chunks[k] =
		    kzalloc(sizeof(unsigned long) *
			    dev_mgmt->iopattern_chunk_size, GFP_ATOMIC);
		if (!io_data->interval_in_chunks[k])
			goto free_interval_chunks;
	}

	io_data->read_seq_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS,
		    GFP_ATOMIC);
	if (!io_data->read_seq_count_per_chunk)
		goto free_interval_chunks;
	io_data->write_seq_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->write_seq_count_per_chunk)
		goto free_read_seq_count_per_chunk;
	io_data->read_seq_bytes_per_chunk =
	    kzalloc(sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS,
		    GFP_ATOMIC);
	if (!io_data->read_seq_bytes_per_chunk)
		goto free_write_seq_count_per_chunk;
	io_data->write_seq_bytes_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->write_seq_bytes_per_chunk)
		goto free_read_seq_bytes_per_chunk;
	io_data->read_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * IOPATTERN_READ_SKEWNESS,
		    GFP_ATOMIC);
	if (!io_data->read_count_per_chunk)
		goto free_write_seq_bytes_per_chunk;
	io_data->write_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->iopattern_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->write_count_per_chunk)
		goto free_read_count_per_chunk;
	return 0;

free_read_count_per_chunk:
	kfree(io_data->read_count_per_chunk);
free_write_seq_bytes_per_chunk:
	kfree(io_data->write_seq_bytes_per_chunk);
free_read_seq_bytes_per_chunk:
	kfree(io_data->read_seq_bytes_per_chunk);
free_write_seq_count_per_chunk:
	kfree(io_data->write_seq_count_per_chunk);
free_read_seq_count_per_chunk:
	kfree(io_data->read_seq_count_per_chunk);
free_interval_chunks:
	for (--k; k >= 0; k--) {
		if (io_data->interval_in_chunks[k])
			kfree(io_data->interval_in_chunks[k]);
	}
	kfree(io_data->interval_in_chunks);
free_write_skewness:
	for (--j; j >= 0; j--) {
		if (io_data->write_skewness[j])
			kfree(io_data->write_skewness[j]);
	}
	kfree(io_data->write_skewness);
free_read_skewness:
	for (--i; i >= 0; i--) {
		if (io_data->read_skewness[i])
			kfree(io_data->read_skewness[i]);
	}
	kfree(io_data->read_skewness);

	return -ENOMEM;
}

int setup_iopattern(struct gendisk *disk, const char *dev_name)
{
	struct iopattern_dev *iopatt_dev;
	sector_t dev_capacity;
	int flag;

	pr_info("setup_iopattern start: %s\n", dev_name);
	if (IOPATTERN_SUPPORT_MAX_DEV == dev_mgmt->iopattern_dev_num) {
		pr_err
		    ("iopattern support dev max : %d, and can't add more nvme ssd!! \n",
		     dev_mgmt->iopattern_dev_num);
		return 1;
	}

	iopatt_dev = kzalloc(sizeof(struct iopattern_dev), GFP_ATOMIC);
	if (!iopatt_dev)
		return -ENOMEM;

	clear_bit(IOPATTERN_MONITOR_STATUS, &iopatt_dev->iopattern_flags);
	set_bit(IOPATTERN_FIRST_READ, &iopatt_dev->iopattern_flags);
	set_bit(IOPATTERN_FIRST_WRITE, &iopatt_dev->iopattern_flags);
	iopatt_dev->previous_time = 0;

	snprintf(iopatt_dev->dev_name, DEV_NAME_LEN, "%s", dev_name);
	spin_lock_init(&iopatt_dev->lock);

	if (disk) {
		dev_capacity = get_capacity(disk);
		flag = alloc_dev_data(&iopatt_dev->dev_data, dev_capacity);
		if (flag != 0) {
			pr_err
			    ("StoneNeedle: creat device %s proc file failed, terminated due to memory allocation error.\n",
			     iopatt_dev->dev_name);
			kfree(iopatt_dev);
			return flag;
		}
	}
	list_add_tail(&iopatt_dev->list, &dev_mgmt->iopattern_dev_list);
	dev_mgmt->iopattern_dev_num++;
	create_iopattern_proc(dev_name);
	pr_info("StoneNeedle: creat device %s proc file successfully.\n",
		iopatt_dev->dev_name);

	return 0;
}

int release_iopattern(const char *dev_name)
{
	struct iopattern_dev *iopatt_dev;
	struct iopattern_data *io_data;
	int i;

	pr_info("remove proc: %s \n", dev_name);
	remove_proc_entry(dev_name, dev_mgmt->iopattern_root);

	iopatt_dev = find_iopattern_dev(dev_name);
	if (!iopatt_dev)
		goto out;

	io_data = &iopatt_dev->dev_data;
	if (io_data->interval_in_chunks) {
		for (i = 0; i < IOPATTERN_INTERVAL_BUCKET_NUM; i++) {
			if (io_data->interval_in_chunks[i])
				kfree(io_data->interval_in_chunks[i]);
		}
		kfree(io_data->interval_in_chunks);
	}

	if (io_data->write_skewness) {
		for (i = 0; i < IOPATTERN_TOTAL_IOSIZE; i++) {
			if (io_data->write_skewness[i])
				kfree(io_data->write_skewness[i]);
		}
		kfree(io_data->write_skewness);
	}

	if (io_data->read_skewness) {
		for (i = 0; i < IOPATTERN_TOTAL_IOSIZE; i++) {
			if (io_data->read_skewness[i])
				kfree(io_data->read_skewness[i]);
		}
		kfree(io_data->read_skewness);
	}

	if (io_data->read_seq_count_per_chunk)
		kfree(io_data->read_seq_count_per_chunk);

	if (io_data->write_seq_count_per_chunk)
		kfree(io_data->write_seq_count_per_chunk);

	if (io_data->read_seq_bytes_per_chunk)
		kfree(io_data->read_seq_bytes_per_chunk);

	if (io_data->write_seq_bytes_per_chunk)
		kfree(io_data->write_seq_bytes_per_chunk);

	if (io_data->read_count_per_chunk)
		kfree(io_data->read_count_per_chunk);

	if (io_data->write_count_per_chunk)
		kfree(io_data->write_count_per_chunk);

	list_del(&iopatt_dev->list);
	kfree(iopatt_dev);
	dev_mgmt->iopattern_dev_num--;

out:
	return 0;
}

int register_stoneneedle(int iopattern_chunk_size,
			 struct stoneneedle_ops *sn_ops)
{
	if (100 == iopattern_chunk_size || 10000 == iopattern_chunk_size) {
		sn_ops->calc_iopattern = calc_iopattern;
		sn_ops->setup_iopattern = setup_iopattern;
		sn_ops->release_iopattern = release_iopattern;
	} else {
		sn_ops->calc_iopattern = NULL;
		sn_ops->setup_iopattern = NULL;
		sn_ops->release_iopattern = NULL;
		pr_err
		    ("StoneNeedle: initialization parameter error. Only 100(default) or 10000 is avialable.\n");
		return 0;
	}

	dev_mgmt = kzalloc(sizeof(struct iopattern_dev_mgmt), GFP_ATOMIC);
	if (!dev_mgmt) {
		pr_err
		    ("StoneNeedle: terminated due to memory allocation error.\n");
		return -ENOMEM;
	}

	dev_mgmt->cur_open_dev = NULL;
	dev_mgmt->iopattern_root = NULL;
	dev_mgmt->iopattern_dev_num = 0;
	dev_mgmt->interval_hist.first = 0;
	dev_mgmt->interval_hist.delta = 1;
	dev_mgmt->interval_hist.num = IOPATTERN_INTERVAL_BUCKET_NUM;
	dev_mgmt->iopattern_chunk_size = iopattern_chunk_size;
	INIT_LIST_HEAD(&dev_mgmt->iopattern_dev_list);
	setup_iopattern(NULL, controller_file);

	return 0;
}

void unregister_stoneneedle(void)
{
	struct iopattern_dev *iopatt_dev, *tmp;

	if (!dev_mgmt)
		return;

	list_for_each_entry_safe(iopatt_dev, tmp, &dev_mgmt->iopattern_dev_list,
				 list) {
		pr_info("unregister_stoneneedle, remove proc file: %s \n",
			iopatt_dev->dev_name);
		remove_proc_entry(iopatt_dev->dev_name,
				  dev_mgmt->iopattern_root);
		list_del(&iopatt_dev->list);
		kfree(iopatt_dev);
		dev_mgmt->iopattern_dev_num--;
	}
	remove_proc_entry(user_root_dir, NULL);
	kfree(dev_mgmt);
	return;
}
