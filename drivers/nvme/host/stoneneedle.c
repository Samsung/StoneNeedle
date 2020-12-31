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
#include <linux/module.h>

#include "stoneneedle.h"

unsigned int stoneneedle_chunk_size = 100;
module_param(stoneneedle_chunk_size, int, 0644);
MODULE_PARM_DESC(stoneneedle_chunk_size, " 100, 1000, 10000(default 100)");
EXPORT_SYMBOL_GPL(stoneneedle_chunk_size);

#define    ERROR_DEVICE_COUNT_OVERFLOW			(1)

#define    BUFFER_LEN                       (32)
#define    STONENEEDLE_4KB_CHUNK              (4*1024)
#define    STONENEEDLE_NR_IOSIZE              (128/4)
#define    STONENEEDLE_TOTAL_IOSIZE           (STONENEEDLE_NR_IOSIZE+2)    /* 32 common + 2 special */
#define    STONENEEDLE_READ                   (0)
#define    STONENEEDLE_WRITE                  (1)
#define    STONENEEDLE_INTERVAL_BUCKET_NUM    (25)
#define    STONENEEDLE_SUPPORT_MAX_DEV        (24+1)    /* controller + 24*ssd */

/* StoneNeedle flags */
#define    STONENEEDLE_MONITOR_STATUS         (0)    /* device monitor status bit:bit0 */
#define    STONENEEDLE_FIRST_READ             (1)    /* first read flag on device: bit1 */
#define    STONENEEDLE_FIRST_WRITE            (2)    /* first write flag on device: bit2 */

enum stoneneedle_index_item {
	STONENEEDLE_WRITE_IO_COUNT = 0,
	STONENEEDLE_READ_IO_COUNT,
	STONENEEDLE_TOTAL_WRITE_BYTES,
	STONENEEDLE_TOTAL_READ_BYTES,
	STONENEEDLE_WRITE_STRIDE,
	STONENEEDLE_READ_STRIDE,
	STONENEEDLE_WRITE_IO_ARRIVAL_INTERVAL,
	STONENEEDLE_READ_IO_ARRIVAL_INTERVAL,
	STONENEEDLE_WRITE_SEQ_SECTOR_BYTES,
	STONENEEDLE_READ_SEQ_SECTOR_BYTES,
	STONENEEDLE_RANDOM_WRITE_IO_COUNT,
	STONENEEDLE_RANDOM_READ_IO_COUNT,
	STONENEEDLE_WRITE_CHUNKS,
	STONENEEDLE_WRITE_128TO256_CHUNKS,
	STONENEEDLE_WRITE_EX256KB_CHUNKS,
	STONENEEDLE_READ_CHUNKS,
	STONENEEDLE_READ_128TO256_CHUNKS,
	STONENEEDLE_READ_EX256KB_CHUNKS,
	STONENEEDLE_WRITE_INTERVAL_PER_CHUNK,
	STONENEEDLE_READ_INTERVAL_PER_CHUNK,
	STONENEEDLE_WRITE_SEQ_COUNT_PER_CHUNK,
	STONENEEDLE_READ_SEQ_COUNT_PER_CHUNK,
	STONENEEDLE_WRITE_SEQ_BYTES_PER_CHUNK,
	STONENEEDLE_READ_SEQ_BYTES_PER_CHUNK,
	STONENEEDLE_WRITE_COUNT_PER_CHUNK,
	STONENEEDLE_READ_COUNT_PER_CHUNK,
	/* add other parameter here */
	STONENEEDLE_MAX,
};

const char *const device_status[] = { "stop", "start" };
const char *const user_root_dir = "stoneneedle";

enum proc_dev_index {
	IOSTAT = 0,
	CONTROLLER,
};

enum stoneneedle_op_type {
	STONENEEDLE_RANDOM,
	STONENEEDLE_SEQ,
};

const char *proc_dev_text[] = {
	"iodata",
	"controller",
};

const char *const stoneneedle_text[] = {
	/* Enum StoneNeedle */
	"WRITE_IO_COUNT:",
	"READ_IO_COUNT:",
	"WRITE_TOTAL_BYTES:",
	"READ_TOTAL_BYTES:",
	"WRITE_STRIDE_SECTORS:",
	"READ_STRIDE_SECTORS:",
	"WRITE_IO_ARRIVAL_INTERVAL:",
	"READ_IO_ARRIVAL_INTERVAL:",
	"WRITE_SEQUENTIAL_BYTES:",
	"READ_SEQUENTIAL_BYTES:",
	"WRITE_RANDOM_IO_COUNT:",
	"READ_RANDOM_IO_COUNT:",
	"KB_WRITE_SKEWNESS:",
	"128KB_WRITE_SKEWNESS:",
	"256KB_WRITE_SKEWNESS:",
	"KB_READ_SKEWNESS:",
	"128KB_READ_SKEWNESS:",
	"256KB_READ_SKEWNESS:",
	"WRITE_INTERVAL_",
	"READ_INTERVAL_",
	"WRITE_SEQUENTIAL_COUNT_PER_CHUNK:",
	"READ_SEQUENTIAL_COUNT_PER_CHUNK:",
	"WRITE_SEQUENTIAL_BYTES_PER_CHUNK:",
	"READ_SEQUENTIAL_BYTES_PER_CHUNK:",
	"WRITE_COUNT_PER_CHUNK:",
	"READ_COUNT_PER_CHUNK:",
};

struct histlog2 {
	int first;
	int delta;
	int num;
};

struct stoneneedle_data {
	unsigned long stoneneedle_value[STONENEEDLE_MAX];
	unsigned long stoneneedle_value_pre[STONENEEDLE_MAX];
	unsigned long **write_skewness;
	unsigned long **read_skewness;
	unsigned long **write_interval_per_chunk;
	unsigned long **read_interval_per_chunk;
	unsigned long *write_seq_count_per_chunk;
	unsigned long *read_seq_count_per_chunk;
	unsigned long *write_seq_bytes_per_chunk;
	unsigned long *read_seq_bytes_per_chunk;
	unsigned long *write_count_per_chunk;
	unsigned long *read_count_per_chunk;
	sector_t bucket_size;
	spinlock_t lock;
	spinlock_t irqlock;
};

struct stoneneedle_dev {
	/* StoneNeedle data */
	struct proc_dir_entry *dev_proc_dir;
	struct stoneneedle_data dev_data;
	char dev_name[DEV_NAME_LEN];
	unsigned long stoneneedle_flags;
	unsigned long previous_write_time;
	unsigned long previous_read_time;
	unsigned long previous_append_addr;
	unsigned long next_real_append_addr;
	spinlock_t lock;
	struct list_head list;
};

struct stoneneedle_dev_mgmt {
	struct proc_dir_entry *stoneneedle_root;
	struct list_head stoneneedle_dev_list;
	struct histlog2 interval_hist;
	int stoneneedle_dev_num;
	int stoneneedle_chunk_size;
};

static struct stoneneedle_dev_mgmt *dev_mgmt;

static struct stoneneedle_dev *find_stoneneedle_dev(const char *dev_name)
{
	struct stoneneedle_dev *sn_dev;
	list_for_each_entry(sn_dev, &dev_mgmt->stoneneedle_dev_list, list) {
		if (strstr (dev_name, sn_dev->dev_name) != NULL)
			return sn_dev;
	}
	return NULL;
}

static void *stoneneedle_start(struct seq_file *m, loff_t *pos)
{
	unsigned long *base;
	int i;

	struct stoneneedle_dev *cur_stoneneedle_dev;
	struct stoneneedle_data *io_data;

	cur_stoneneedle_dev = find_stoneneedle_dev(m->file->f_path.dentry->d_parent->d_iname);
	io_data = &cur_stoneneedle_dev->dev_data;

	if (strstr(m->file->f_path.dentry->d_iname, proc_dev_text[CONTROLLER]) != NULL) {
		if (*pos >= 1)
			return NULL;
	}
	if (*pos >= ARRAY_SIZE(stoneneedle_text))
		return NULL;

	base = kzalloc(sizeof(unsigned long) * STONENEEDLE_MAX, GFP_ATOMIC);
	m->private = base;

	if (!base)
		return ERR_PTR(-ENOMEM);

	if (cur_stoneneedle_dev) {
		for (i = 0; i < STONENEEDLE_MAX; i++)
			base[i] = io_data->stoneneedle_value[i];

		base += STONENEEDLE_MAX;
	}

	return (unsigned long *)m->private + *pos;
}

static void *stoneneedle_next(struct seq_file *m, void *arg, loff_t *pos)
{
	(*pos)++;
	if (strstr(m->file->f_path.dentry->d_iname, proc_dev_text[CONTROLLER]) != NULL) {
		if (*pos >= 1)
			return NULL;
	} else {
		if (*pos >= ARRAY_SIZE(stoneneedle_text))
			return NULL;
	}
	return (unsigned long *)m->private + *pos;
}

static int stoneneedle_row_show(struct seq_file *m, unsigned long *row_data,
			      int chunk_size)
{
	int i;
	for (i = 0; i < chunk_size - 1; i++)
		seq_printf(m, "%lu,", row_data[i]);

	seq_printf(m, "%lu\n", row_data[chunk_size - 1]);
	return 0;
}

static int stoneneedle_skeness_normal_show(struct seq_file *m, int offset,
					 struct stoneneedle_data *io_data)
{
	int j, chunk_size;
	unsigned long **skewness;

	chunk_size = dev_mgmt->stoneneedle_chunk_size;
	if (STONENEEDLE_WRITE_CHUNKS == offset) {
		skewness = io_data->write_skewness;
	} else {
		skewness = io_data->read_skewness;
	}

	for (j = 0; j < STONENEEDLE_NR_IOSIZE; j++) {
		seq_printf(m, "%d%s ", j * 4, stoneneedle_text[offset]);
		stoneneedle_row_show(m, skewness[j], chunk_size);
	}
	return 0;
}

static int stoneneedle_128to256kb_show(struct seq_file *m, int offset,
				     struct stoneneedle_data *io_data)
{
	int chunk_size;
	unsigned long **skewness;

	chunk_size = dev_mgmt->stoneneedle_chunk_size;
	if (STONENEEDLE_WRITE_128TO256_CHUNKS == offset) {
		skewness = io_data->write_skewness;
	} else {
		skewness = io_data->read_skewness;
	}

	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, skewness[STONENEEDLE_NR_IOSIZE], chunk_size);
	return 0;
}

static int stoneneedle_write_interval_bucket_show(struct seq_file *m, int offset,
					  struct stoneneedle_data *io_data)
{
	int j;
	int chunk_size;
	unsigned long **skewness;

	skewness = io_data->write_interval_per_chunk;
	chunk_size = dev_mgmt->stoneneedle_chunk_size;

	for (j = 0; j < STONENEEDLE_INTERVAL_BUCKET_NUM; j++) {
		seq_printf(m, "%sBUCKET%d: ", stoneneedle_text[offset], j);
		stoneneedle_row_show(m, skewness[j], chunk_size);
	}
	return 0;
}

static int stoneneedle_read_interval_bucket_show(struct seq_file *m, int offset,
					  struct stoneneedle_data *io_data)
{
	int j;
	int chunk_size;
	unsigned long **skewness;

	skewness = io_data->read_interval_per_chunk;
	chunk_size = dev_mgmt->stoneneedle_chunk_size;

	for (j = 0; j < STONENEEDLE_INTERVAL_BUCKET_NUM; j++) {
		seq_printf(m, "%sBUCKET%d: ", stoneneedle_text[offset], j);
		stoneneedle_row_show(m, skewness[j], chunk_size);
	}
	return 0;
}

static int stoneneedle_ex256kb_show(struct seq_file *m, int offset,
				  struct stoneneedle_data *io_data)
{
	int chunk_size;
	unsigned long **skewness;

	chunk_size = dev_mgmt->stoneneedle_chunk_size;
	if (STONENEEDLE_WRITE_EX256KB_CHUNKS == offset) {
		skewness = io_data->write_skewness;
	} else {
		skewness = io_data->read_skewness;
	}

	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, skewness[STONENEEDLE_NR_IOSIZE + 1], chunk_size);
	return 0;
}

static int stoneneedle_read_seq_count_chunk_show(struct seq_file *m, int offset,
					       struct stoneneedle_data *io_data)
{
	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, io_data->read_seq_count_per_chunk,
			   dev_mgmt->stoneneedle_chunk_size);

	return 0;
}

static int stoneneedle_write_seq_count_chunk_show(struct seq_file *m, int offset,
						struct stoneneedle_data *io_data)
{
	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, io_data->write_seq_count_per_chunk,
			   dev_mgmt->stoneneedle_chunk_size);

	return 0;
}

static int stoneneedle_read_seq_bytes_chunk_show(struct seq_file *m, int offset,
					       struct stoneneedle_data *io_data)
{
	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, io_data->read_seq_bytes_per_chunk,
			   dev_mgmt->stoneneedle_chunk_size);

	return 0;
}

static int stoneneedle_write_seq_bytes_chunk_show(struct seq_file *m, int offset,
						struct stoneneedle_data *io_data)
{
	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, io_data->write_seq_bytes_per_chunk,
			   dev_mgmt->stoneneedle_chunk_size);

	return 0;
}

static int stoneneedle_read_count_chunk_show(struct seq_file *m, int offset,
					   struct stoneneedle_data *io_data)
{
	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, io_data->read_count_per_chunk,
			   dev_mgmt->stoneneedle_chunk_size);

	return 0;
}

static int stoneneedle_write_count_chunk_show(struct seq_file *m, int offset,
					    struct stoneneedle_data *io_data)
{
	seq_printf(m, "%s ", stoneneedle_text[offset]);
	stoneneedle_row_show(m, io_data->write_count_per_chunk,
			   dev_mgmt->stoneneedle_chunk_size);

	return 0;
}

static int stoneneedle_common_show(struct seq_file *m, int offset,
				 struct stoneneedle_data *io_data)
{
	seq_printf(m, "%s %lu\n", stoneneedle_text[offset],
		   io_data->stoneneedle_value[offset]);
	return 0;
}

typedef int (*oper_func[STONENEEDLE_MAX]) (struct seq_file *, int,
					 struct stoneneedle_data *);
oper_func stoneneedle_show_list = {
	stoneneedle_common_show,	/*"WRITE_IO_COUNT:", */
	stoneneedle_common_show,	/*"READ_IO_COUNT:", */
	stoneneedle_common_show,	/*"WRITE_TOTAL_BYTES:", */
	stoneneedle_common_show,	/*"READ_TOTAL_BYTES:", */
	stoneneedle_common_show,	/*"WRITE_STRIDE_SECTORS:", */
	stoneneedle_common_show,	/*"READ_STRIDE_SECTORS:", */
	stoneneedle_common_show,	/*"WRITE_IO_ARRIVAL_INTERVAL:", */
	stoneneedle_common_show,	/*"READ_IO_ARRIVAL_INTERVAL:", */
	stoneneedle_common_show,	/*"WRITE_SEQUETIAL_SECTOR_COUNT:", */
	stoneneedle_common_show,	/*"READ_SEQUETIAL_SECTOR_COUNT:", */
	stoneneedle_common_show,	/*"WRITE_RANDOM_IOS:",    */
	stoneneedle_common_show,	/*"READ_RANDOM_IOS:",     */
	stoneneedle_skeness_normal_show,	/*"KB_WRITE_SKEWNESS:",   */
	stoneneedle_128to256kb_show,	/*"128KB_WRITE_SKEWNESS:", */
	stoneneedle_ex256kb_show,	/*"256KB_WRITE_SKEWNESS:", */
	stoneneedle_skeness_normal_show,	/*"KB_READ_SKEWNESS:",    */
	stoneneedle_128to256kb_show,	/*"128KB_READ_SKEWNESS:", */
	stoneneedle_ex256kb_show,	/*"256KB_READ_SKEWNESS:", */
	stoneneedle_write_interval_bucket_show,	/*"WRITE_INTERVAL",  */
	stoneneedle_read_interval_bucket_show,	/*"READ_INTERVAL",  */
	stoneneedle_write_seq_count_chunk_show,	/*"WRITE_SEQUETIAL_COUNT_PER_CHUNK:", */
	stoneneedle_read_seq_count_chunk_show,	/*"READ_SEQUETIAL_COUNT_PER_CHUNK:", */
	stoneneedle_write_seq_bytes_chunk_show,	/*"WRITE_SEQUETIAL_BYTES_PER_CHUNK:", */
	stoneneedle_read_seq_bytes_chunk_show,	/*"READ_SEQUETIAL_BYTES_PER_CHUNK:", */
	stoneneedle_write_count_chunk_show,	/*"WRITE_COUNT_PER_CHUNK:", */
	stoneneedle_read_count_chunk_show,	/*"READ_COUNT_PER_CHUNK:", */
};

static int stoneneedle_show(struct seq_file *m, void *arg)
{
	unsigned long *input = arg;
	unsigned long offset = input - (unsigned long *)m->private;
	struct stoneneedle_dev *cur_stoneneedle_dev;
	struct stoneneedle_data *io_data;
	cur_stoneneedle_dev = find_stoneneedle_dev(m->file->f_path.dentry->d_parent->d_iname);
	io_data = &cur_stoneneedle_dev->dev_data;

	if (strstr(m->file->f_path.dentry->d_iname, proc_dev_text[CONTROLLER])) {
		seq_printf(m, "%s\t%s\n", cur_stoneneedle_dev->dev_name,
			   device_status[test_bit
					 (STONENEEDLE_MONITOR_STATUS,
					  &cur_stoneneedle_dev->
					  stoneneedle_flags)]);
	} else {
		stoneneedle_show_list[offset] (m, offset, io_data);
	}
	return 0;
}

static void stoneneedle_stop(struct seq_file *m, void *arg)
{
	kfree(m->private);
	m->private = NULL;
}

static const struct seq_operations stoneneedle_op = {
	.start = stoneneedle_start,
	.next = stoneneedle_next,
	.stop = stoneneedle_stop,
	.show = stoneneedle_show,
};

static void stoneneedle_dev_data_clear(struct stoneneedle_dev *sn_dev)
{
	int i;
	memset(sn_dev->dev_data.stoneneedle_value, 0,
	       sizeof(unsigned long) * STONENEEDLE_MAX);
	memset(sn_dev->dev_data.stoneneedle_value_pre, 0,
	       sizeof(unsigned long) * STONENEEDLE_MAX);
	for (i = 0; i < STONENEEDLE_TOTAL_IOSIZE; i++) {
		memset(sn_dev->dev_data.write_skewness[i], 0,
		       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
		memset(sn_dev->dev_data.read_skewness[i], 0,
		       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	}
	for (i = 0; i < STONENEEDLE_INTERVAL_BUCKET_NUM; i++) {
		memset(sn_dev->dev_data.write_interval_per_chunk[i], 0,
		       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
		memset(sn_dev->dev_data.read_interval_per_chunk[i], 0,
		       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	}
	memset(sn_dev->dev_data.read_seq_count_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	memset(sn_dev->dev_data.write_seq_count_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	memset(sn_dev->dev_data.read_seq_bytes_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	memset(sn_dev->dev_data.write_seq_bytes_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	memset(sn_dev->dev_data.read_count_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	memset(sn_dev->dev_data.write_count_per_chunk, 0,
	       sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size);
	return;
}

static ssize_t stoneneedle_controller(struct file *filp, const char __user *buf,
				    size_t count, loff_t *f_ops)
{
	char buffer[BUFFER_LEN] = { 0 };
	struct stoneneedle_dev *sn_dev;

	if (copy_from_user
	    (buffer, buf, (count > BUFFER_LEN ? BUFFER_LEN : count)))
		return count;

	sn_dev = find_stoneneedle_dev(filp->f_path.dentry->d_parent->d_iname);
	if (!sn_dev)
		return count;

	spin_lock(&sn_dev->lock);
	if (strstr(buffer, device_status[0])) {
		clear_bit(STONENEEDLE_MONITOR_STATUS,
		&sn_dev->stoneneedle_flags);
    }
	else if (strstr(buffer, device_status[1])
	    && !test_bit(STONENEEDLE_MONITOR_STATUS,
			 &sn_dev->stoneneedle_flags)) {
		set_bit(STONENEEDLE_MONITOR_STATUS, &sn_dev->stoneneedle_flags);
		set_bit(STONENEEDLE_FIRST_READ, &sn_dev->stoneneedle_flags);
		set_bit(STONENEEDLE_FIRST_WRITE, &sn_dev->stoneneedle_flags);
		sn_dev->previous_write_time = 0;
		sn_dev->previous_read_time = 0;
		sn_dev->previous_append_addr = 0;
		sn_dev->next_real_append_addr = 0;

		/* clear stoneneedle dev data */
		spin_lock(&sn_dev->dev_data.lock);
		stoneneedle_dev_data_clear(sn_dev);
		spin_unlock(&sn_dev->dev_data.lock);
	} else if (strstr(buffer, device_status[1]) && test_bit(STONENEEDLE_MONITOR_STATUS, &sn_dev->stoneneedle_flags)) {
		pr_info("StoneNeedle: %s controller has already started.\n", sn_dev->dev_name);
	} else {
		pr_err("StoneNeedle: error input to %s controller.\n",
		sn_dev->dev_name);
	}
	spin_unlock(&sn_dev->lock);

	return count;
}

static int stoneneedle_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &stoneneedle_op);
}

static const struct proc_ops proc_stoneneedle_file_operations = {
	.proc_open = stoneneedle_open,
	.proc_read = seq_read,
	.proc_write = stoneneedle_controller,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};

static long __calc_rw_stride(enum stoneneedle_index_item index,
				   __le64 slba, __le16 length,
				   struct stoneneedle_dev *sn_dev, int op)
{
	unsigned long stride_value = 0;
	unsigned long addr_max;
    long stride = 0;
	struct stoneneedle_data *io_data;

	io_data = &sn_dev->dev_data;

	if(op == REQ_OP_ZONE_APPEND) {
		if(slba != sn_dev->previous_append_addr) {
			addr_max = sn_dev->next_real_append_addr < io_data->stoneneedle_value_pre[index] ? io_data->stoneneedle_value_pre[index] : sn_dev->next_real_append_addr;
			stride = slba - addr_max;
			spin_lock(&sn_dev->dev_data.lock);
			sn_dev->previous_append_addr = slba;
			io_data->stoneneedle_value_pre[index] = slba;
			spin_unlock(&sn_dev->dev_data.lock);
		}
	} else {
		stride = slba - io_data->stoneneedle_value_pre[index];
	}

	if (stride < 0)
		stride_value = -stride;
	else
		stride_value = stride;

	/* if the bs=512, length=0, so we need add 1 to length */

	spin_lock(&sn_dev->dev_data.lock);
	io_data->stoneneedle_value[index] += stride_value;
	io_data->stoneneedle_value_pre[index] =
			(op == REQ_OP_ZONE_APPEND ? io_data->stoneneedle_value_pre[index] : slba) + (length + 1);
	spin_unlock(&sn_dev->dev_data.lock);

	return stride;
}

static void calc_rw_stride(enum stoneneedle_index_item index, __le64 slba, __le16 length, struct stoneneedle_dev *sn_dev, struct request *req) {
	long stride_diff = __calc_rw_stride(index, slba, length, sn_dev, req_op(req));
	nvme_req(req)->seq = stride_diff == 0 ? STONENEEDLE_SEQ : STONENEEDLE_RANDOM;
}

static void calc_io_arrival_interval(struct stoneneedle_dev *sn_dev, int op)
{
	struct stoneneedle_data *io_data;
	io_data = &sn_dev->dev_data;
	/* get time */
	u64 milliseconds = ktime_to_ns(ktime_get());
	if (op == STONENEEDLE_WRITE) {
		if (test_bit
		    (STONENEEDLE_FIRST_WRITE, &sn_dev->stoneneedle_flags)) {
			spin_lock(&sn_dev->dev_data.lock);

			io_data->
			    stoneneedle_value_pre
			    [STONENEEDLE_WRITE_IO_ARRIVAL_INTERVAL] =
			    milliseconds / 1000;
			spin_unlock(&sn_dev->dev_data.lock);
			spin_lock(&sn_dev->lock);
			clear_bit(STONENEEDLE_FIRST_WRITE,
				  &sn_dev->stoneneedle_flags);
			spin_unlock(&sn_dev->lock);
		} else {
			spin_lock(&sn_dev->dev_data.lock);
			io_data->
			    stoneneedle_value[STONENEEDLE_WRITE_IO_ARRIVAL_INTERVAL]
			    +=
			    milliseconds / 1000 -
			    io_data->
			    stoneneedle_value_pre
			    [STONENEEDLE_WRITE_IO_ARRIVAL_INTERVAL];
			io_data->
			    stoneneedle_value_pre
			    [STONENEEDLE_WRITE_IO_ARRIVAL_INTERVAL] =
			    milliseconds / 1000;
			spin_unlock(&sn_dev->dev_data.lock);
		}
	} else {
		if (test_bit
		    (STONENEEDLE_FIRST_READ, &sn_dev->stoneneedle_flags)) {
			spin_lock(&sn_dev->dev_data.lock);
			io_data->
			    stoneneedle_value_pre
			    [STONENEEDLE_READ_IO_ARRIVAL_INTERVAL] =
			    milliseconds / 1000;
			spin_unlock(&sn_dev->dev_data.lock);
			spin_lock(&sn_dev->lock);
			clear_bit(STONENEEDLE_FIRST_READ,
				  &sn_dev->stoneneedle_flags);
			spin_unlock(&sn_dev->lock);
		} else {
			spin_lock(&sn_dev->dev_data.lock);
			io_data->
			    stoneneedle_value[STONENEEDLE_READ_IO_ARRIVAL_INTERVAL]
			    +=
			    milliseconds / 1000 -
			    io_data->
			    stoneneedle_value_pre
			    [STONENEEDLE_READ_IO_ARRIVAL_INTERVAL];
			io_data->
			    stoneneedle_value_pre
			    [STONENEEDLE_READ_IO_ARRIVAL_INTERVAL] =
			    milliseconds / 1000;
			spin_unlock(&sn_dev->dev_data.lock);
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

static int calc_bucket_account(unsigned long *bucket, struct request *req,
			       sector_t bucket_size)
{
	int index = blk_rq_pos(req) / bucket_size;
	bucket[index]++;
	return 0;
}

static void calc_write_chunk_interval_upper(struct request *req,
				      struct stoneneedle_dev *sn_dev)
{
	unsigned long interval;
	/* get time */
	u64 milliseconds = ktime_to_ns(ktime_get());
	interval = 0;

	if (sn_dev->previous_write_time)
		interval =
		    milliseconds / 1000 - sn_dev->previous_write_time;

	nvme_req(req)->interval = interval;

	spin_lock(&sn_dev->dev_data.lock);
	sn_dev->previous_write_time = milliseconds / 1000;
	spin_unlock(&sn_dev->dev_data.lock);
	return;
}

static void calc_write_chunk_interval_down(struct request *req,
				      struct stoneneedle_dev *sn_dev)
{
	unsigned long interval;
	int index;
	struct stoneneedle_data *io_data;

	io_data = &sn_dev->dev_data;
	
	interval = nvme_req(req)->interval;

	index = histlog2_index(interval, &dev_mgmt->interval_hist);

	spin_lock(&sn_dev->dev_data.irqlock);
	calc_bucket_account(io_data->write_interval_per_chunk[index], req,
			    io_data->bucket_size);
	spin_unlock(&sn_dev->dev_data.irqlock);
	return;
}

static void calc_read_chunk_interval(struct request *req,
				      struct stoneneedle_dev *sn_dev)
{
	unsigned long interval;
	int index;
	struct stoneneedle_data *io_data;

	io_data = &sn_dev->dev_data;
	/* get time */
	u64 milliseconds = ktime_to_ns(ktime_get());
	interval = 0;

	if (sn_dev->previous_read_time)
		interval =
		    milliseconds / 1000 - sn_dev->previous_read_time;

	index = histlog2_index(interval, &dev_mgmt->interval_hist);

	spin_lock(&sn_dev->dev_data.lock);
	sn_dev->previous_read_time = milliseconds / 1000;
	calc_bucket_account(io_data->read_interval_per_chunk[index], req,
			    io_data->bucket_size);
	spin_unlock(&sn_dev->dev_data.lock);
	return;
}

static unsigned int calc_chunk_index(unsigned int index)
{
	if (index >= STONENEEDLE_NR_IOSIZE * 2) {
		index = STONENEEDLE_NR_IOSIZE + 1;
	} else if (index > STONENEEDLE_NR_IOSIZE
		   && index < STONENEEDLE_NR_IOSIZE * 2) {
		index = STONENEEDLE_NR_IOSIZE;
	}
	return index;
}

static void calc_write_stoneneedle_upper(struct request *req, __le64 slba, __le16 length,
				 struct stoneneedle_dev *sn_dev)
{
	calc_rw_stride(STONENEEDLE_WRITE_STRIDE, slba, length, sn_dev, req);
	calc_io_arrival_interval(sn_dev, STONENEEDLE_WRITE);
}

static void calc_write_stoneneedle_down(struct request *req, __le64 slba, __le16 length,
				 struct stoneneedle_dev *sn_dev)
{
	unsigned int index;
	unsigned int rq_bytes = blk_rq_bytes(req);
	struct stoneneedle_data *io_data;

	io_data = &sn_dev->dev_data;
	
	spin_lock(&sn_dev->dev_data.irqlock);
	sn_dev->next_real_append_addr = slba + (length + 1);
	if (nvme_req(req)->seq != STONENEEDLE_SEQ) {
		io_data->stoneneedle_value[STONENEEDLE_RANDOM_WRITE_IO_COUNT]++;
	} else {
		io_data->stoneneedle_value[STONENEEDLE_WRITE_SEQ_SECTOR_BYTES] +=
		    rq_bytes;
		calc_bucket_account(io_data->write_seq_count_per_chunk, req,
				    io_data->bucket_size);
		io_data->write_seq_bytes_per_chunk[blk_rq_pos(req) /
						   io_data->bucket_size] += rq_bytes;
	}

	io_data->stoneneedle_value[STONENEEDLE_WRITE_IO_COUNT]++;
	io_data->stoneneedle_value[STONENEEDLE_TOTAL_WRITE_BYTES] += rq_bytes;
	calc_bucket_account(io_data->write_count_per_chunk, req,
			    io_data->bucket_size);
	spin_unlock(&sn_dev->dev_data.irqlock);

	index = calc_chunk_index(rq_bytes / STONENEEDLE_4KB_CHUNK);
	spin_lock(&sn_dev->dev_data.irqlock);
	calc_bucket_account(io_data->write_skewness[index], req,
			    io_data->bucket_size);
	spin_unlock(&sn_dev->dev_data.irqlock);
}

static void calc_read_stoneneedle(struct request *req, __le64 slba, __le16 length,
				struct stoneneedle_dev *sn_dev)
{
	unsigned int index;
	unsigned int rq_bytes = blk_rq_bytes(req);
	struct stoneneedle_data *io_data;
	io_data = &sn_dev->dev_data;
	
	calc_rw_stride(STONENEEDLE_READ_STRIDE, slba, length, sn_dev, req);

	/*
	 * Read random
	 start address of last io + last io length equal
	 to current start address
	 */
	spin_lock(&sn_dev->dev_data.lock);
	if (nvme_req(req)->seq != STONENEEDLE_SEQ) {
		io_data->stoneneedle_value[STONENEEDLE_RANDOM_READ_IO_COUNT]++;
	} else {
		io_data->stoneneedle_value[STONENEEDLE_READ_SEQ_SECTOR_BYTES] +=
		    rq_bytes;
		calc_bucket_account(io_data->read_seq_count_per_chunk, req,
				    io_data->bucket_size);
		io_data->read_seq_bytes_per_chunk[blk_rq_pos(req) /
						  io_data->bucket_size] +=
		    rq_bytes;
	}

	io_data->stoneneedle_value[STONENEEDLE_READ_IO_COUNT]++;
	io_data->stoneneedle_value[STONENEEDLE_TOTAL_READ_BYTES] += rq_bytes;
	calc_bucket_account(io_data->read_count_per_chunk, req,
			    io_data->bucket_size);

	spin_unlock(&sn_dev->dev_data.lock);

	calc_io_arrival_interval(sn_dev, STONENEEDLE_READ);

	index = calc_chunk_index(rq_bytes / STONENEEDLE_4KB_CHUNK);
	spin_lock(&sn_dev->dev_data.lock);
	calc_bucket_account(io_data->read_skewness[index], req,
			    io_data->bucket_size);
	spin_unlock(&sn_dev->dev_data.lock);
}

static void calc_stoneneedle(struct nvme_ns *ns, struct request *req, int complete)
{
	struct stoneneedle_dev *sn_dev;
	struct stoneneedle_data *io_data;
	__le64 slba;
	__le16 length;
	
	if(unlikely(!ns || !ns->disk))
		return;

	sn_dev = find_stoneneedle_dev(ns->disk->disk_name);

	if (sn_dev
	    && test_bit(STONENEEDLE_MONITOR_STATUS,
			&sn_dev->stoneneedle_flags)) {
		io_data = &sn_dev->dev_data;
		slba = cpu_to_le64(nvme_sect_to_lba(ns, blk_rq_pos(req)));
		//slba = nvme_req(req)->result.u64;
		length = cpu_to_le16((blk_rq_bytes(req) >> ns->lba_shift) - 1);
		switch(req_op(req)) {
			case REQ_OP_READ:
				if(!complete) {
					calc_read_stoneneedle(req, slba, length, sn_dev);
					calc_read_chunk_interval(req, sn_dev);
				}
				break;
			case REQ_OP_ZONE_APPEND:
			case REQ_OP_WRITE:
				if(complete) {
					calc_write_stoneneedle_down(req, slba, length, sn_dev);
					calc_write_chunk_interval_down(req, sn_dev);
				} else {
					calc_write_stoneneedle_upper(req, slba, length, sn_dev);
					calc_write_chunk_interval_upper(req, sn_dev);
				}
				break;
			default:
				break;
		}
	}
}

static void create_stoneneedle_proc(struct stoneneedle_dev *sn_dev)
{
	int i;
	if (!dev_mgmt->stoneneedle_root)
		dev_mgmt->stoneneedle_root = proc_mkdir(user_root_dir, NULL);
	
	sn_dev->dev_proc_dir = proc_mkdir(sn_dev->dev_name, dev_mgmt->stoneneedle_root);
	
	if(NULL != sn_dev->dev_proc_dir) {
		for(i=0; i<ARRAY_SIZE(proc_dev_text); i++)
			proc_create(proc_dev_text[i], S_IRUGO, sn_dev->dev_proc_dir,
					&proc_stoneneedle_file_operations);
	}
}

static int alloc_dev_data(struct stoneneedle_data *io_data, sector_t dev_capacity)
{
	int i, j, k, m;
	spin_lock_init(&io_data->lock);
	spin_lock_init(&io_data->irqlock);
	if (dev_capacity % dev_mgmt->stoneneedle_chunk_size != 0)
		io_data->bucket_size =
		    dev_capacity / dev_mgmt->stoneneedle_chunk_size + 1;
	else
		io_data->bucket_size =
		    dev_capacity / dev_mgmt->stoneneedle_chunk_size;
		    
	io_data->write_skewness =
	    kzalloc(sizeof(unsigned long *) * STONENEEDLE_TOTAL_IOSIZE,
		    GFP_ATOMIC);
	if (!io_data->write_skewness)
		return -ENOMEM;

	for (i = 0; i < STONENEEDLE_TOTAL_IOSIZE; i++) {
		io_data->write_skewness[i] =
		    kzalloc(sizeof(unsigned long) *
			    dev_mgmt->stoneneedle_chunk_size, GFP_ATOMIC);
		if (!io_data->write_skewness[i])
			goto free_write_skewness;
	}

	io_data->read_skewness =
	    kzalloc(sizeof(unsigned long *) * STONENEEDLE_TOTAL_IOSIZE,
		    GFP_ATOMIC);
	if (!io_data->read_skewness)
		goto free_write_skewness;
	for (j = 0; j < STONENEEDLE_TOTAL_IOSIZE; j++) {
		io_data->read_skewness[j] =
		    kzalloc(sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size,
			    GFP_ATOMIC);
		if (!io_data->read_skewness[j])
			goto free_read_skewness;
	}

	io_data->write_interval_per_chunk =
	    kzalloc(sizeof(unsigned long *) * STONENEEDLE_INTERVAL_BUCKET_NUM,
		    GFP_ATOMIC);
	if (!io_data->write_interval_per_chunk)
		goto free_read_skewness;

	for (k = 0; k < STONENEEDLE_INTERVAL_BUCKET_NUM; k++) {
		io_data->write_interval_per_chunk[k] =
		    kzalloc(sizeof(unsigned long) *
			    dev_mgmt->stoneneedle_chunk_size, GFP_ATOMIC);
		if (!io_data->write_interval_per_chunk[k])
			goto free_write_interval;
	}

	io_data->read_interval_per_chunk =
	    kzalloc(sizeof(unsigned long *) * STONENEEDLE_INTERVAL_BUCKET_NUM,
		    GFP_ATOMIC);
	if (!io_data->read_interval_per_chunk)
		goto free_write_interval;

	for (m = 0; m < STONENEEDLE_INTERVAL_BUCKET_NUM; m++) {
		io_data->read_interval_per_chunk[m] =
		    kzalloc(sizeof(unsigned long) *
			    dev_mgmt->stoneneedle_chunk_size, GFP_ATOMIC);
		if (!io_data->read_interval_per_chunk[m])
			goto free_read_interval;
	}

	io_data->write_seq_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->write_seq_count_per_chunk)
		goto free_read_interval;
	io_data->read_seq_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->read_seq_count_per_chunk)
		goto free_write_seq_count_per_chunk;

	io_data->write_seq_bytes_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->write_seq_bytes_per_chunk)
		goto free_read_seq_count_per_chunk;
	io_data->read_seq_bytes_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->read_seq_bytes_per_chunk)
		goto free_write_seq_bytes_per_chunk;

	io_data->write_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->write_count_per_chunk)
		goto free_read_seq_bytes_per_chunk;
	io_data->read_count_per_chunk =
	    kzalloc(sizeof(unsigned long) * dev_mgmt->stoneneedle_chunk_size,
		    GFP_ATOMIC);
	if (!io_data->read_count_per_chunk)
		goto free_write_count_per_chunk;
	return 0;

free_write_count_per_chunk:
	kfree(io_data->write_count_per_chunk);
free_read_seq_bytes_per_chunk:
	kfree(io_data->read_seq_bytes_per_chunk);
free_write_seq_bytes_per_chunk:
	kfree(io_data->write_seq_bytes_per_chunk);
free_read_seq_count_per_chunk:
	kfree(io_data->read_seq_count_per_chunk);
free_write_seq_count_per_chunk:
	kfree(io_data->write_seq_count_per_chunk);
free_read_interval:
	for (--m; m >= 0; m--) {
		if (io_data->read_interval_per_chunk[m])
			kfree(io_data->read_interval_per_chunk[m]);
	}
	kfree(io_data->read_interval_per_chunk);
free_write_interval:
	for (--k; k >= 0; k--) {
		if (io_data->write_interval_per_chunk[k])
			kfree(io_data->write_interval_per_chunk[k]);
	}
	kfree(io_data->write_interval_per_chunk);
free_read_skewness:
	for (--j; j >= 0; j--) {
		if (io_data->read_skewness[j])
			kfree(io_data->read_skewness[j]);
	}
	kfree(io_data->read_skewness);
free_write_skewness:
	for (--i; i >= 0; i--) {
		if (io_data->write_skewness[i])
			kfree(io_data->write_skewness[i]);
	}
	kfree(io_data->write_skewness);
	return -ENOMEM;
}

static int setup_stoneneedle(struct gendisk *disk, const char *dev_name)
{
	struct stoneneedle_dev *sn_dev;
	sector_t dev_capacity;
	int flag;

	if (STONENEEDLE_SUPPORT_MAX_DEV == dev_mgmt->stoneneedle_dev_num) {
		pr_err
		    ("StoneNeedle support dev max : %d, and can't add more nvme ssd!! \n",
		     dev_mgmt->stoneneedle_dev_num);
		return -ERROR_DEVICE_COUNT_OVERFLOW;
	}

	sn_dev = kzalloc(sizeof(struct stoneneedle_dev), GFP_ATOMIC);
	if (!sn_dev)
		return -ENOMEM;

	clear_bit(STONENEEDLE_MONITOR_STATUS, &sn_dev->stoneneedle_flags);
	set_bit(STONENEEDLE_FIRST_READ, &sn_dev->stoneneedle_flags);
	set_bit(STONENEEDLE_FIRST_WRITE, &sn_dev->stoneneedle_flags);
	sn_dev->previous_write_time = 0;
	sn_dev->previous_read_time = 0;
	sn_dev->previous_append_addr = 0;
	sn_dev->next_real_append_addr = 0;
	sn_dev->dev_proc_dir = NULL;

	snprintf(sn_dev->dev_name, DEV_NAME_LEN, "%s", dev_name);
	spin_lock_init(&sn_dev->lock);

	if (disk) {
		dev_capacity = get_capacity(disk);
		flag = alloc_dev_data(&sn_dev->dev_data, dev_capacity);
		if (flag != 0) {
			pr_err("StoneNeedle: out of memory in alloc_dev_data[%s].\n", sn_dev->dev_name);
			kfree(sn_dev);
			return flag;
		}
	}
	create_stoneneedle_proc(sn_dev);
	list_add_tail(&sn_dev->list, &dev_mgmt->stoneneedle_dev_list);
	dev_mgmt->stoneneedle_dev_num++;
	pr_info("StoneNeedle: create device %s proc file successfully.\n",
	sn_dev->dev_name);

	return 0;
}

void remove_dev_proc(struct stoneneedle_dev *sn_dev) {
	int i;
	if(NULL != sn_dev->dev_proc_dir) {
		for(i=0; i<ARRAY_SIZE(proc_dev_text); i++)
			remove_proc_entry(proc_dev_text[i], sn_dev->dev_proc_dir);
		remove_proc_entry(sn_dev->dev_name, dev_mgmt->stoneneedle_root);
	}
}

static void release_stoneneedle(const char *dev_name)
{
	struct stoneneedle_dev *sn_dev;
	struct stoneneedle_data *io_data;
	int i;

	pr_info("remove proc: %s \n", dev_name);
	
	sn_dev = find_stoneneedle_dev(dev_name);
	if (!sn_dev)
		return;
	remove_dev_proc(sn_dev);
	
	io_data = &sn_dev->dev_data;

	if (io_data->write_interval_per_chunk) {
		for (i = 0; i < STONENEEDLE_INTERVAL_BUCKET_NUM; i++) {
			if (io_data->write_interval_per_chunk[i])
				kfree(io_data->write_interval_per_chunk[i]);
		}
		kfree(io_data->write_interval_per_chunk);
	}

	if (io_data->read_interval_per_chunk) {
		for (i = 0; i < STONENEEDLE_INTERVAL_BUCKET_NUM; i++) {
			if (io_data->read_interval_per_chunk[i])
				kfree(io_data->read_interval_per_chunk[i]);
		}
		kfree(io_data->read_interval_per_chunk);
	}

	if (io_data->write_skewness) {
		for (i = 0; i < STONENEEDLE_TOTAL_IOSIZE; i++) {
			if (io_data->write_skewness[i])
				kfree(io_data->write_skewness[i]);
		}
		kfree(io_data->write_skewness);
	}

	if (io_data->read_skewness) {
		for (i = 0; i < STONENEEDLE_TOTAL_IOSIZE; i++) {
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

	list_del(&sn_dev->list);
	sn_dev->dev_proc_dir = NULL;
	kfree(sn_dev);
	dev_mgmt->stoneneedle_dev_num--;
}

static struct stoneneedle_ops ops = {
	.setup_stoneneedle	        = setup_stoneneedle,
	.calc_stoneneedle		    = calc_stoneneedle,
	.release_stoneneedle		= release_stoneneedle,
};

static int __init stoneneedle_init(void)
{
	dev_mgmt = kzalloc(sizeof(struct stoneneedle_dev_mgmt), GFP_ATOMIC);
	if (!dev_mgmt) {
		pr_err("StoneNeedle: terminated due to memory allocation error.\n");
		return -ENOMEM;
	}

	dev_mgmt->stoneneedle_root = NULL;
	dev_mgmt->stoneneedle_dev_num = 0;
	dev_mgmt->interval_hist.first = 0;
	dev_mgmt->interval_hist.delta = 1;
	dev_mgmt->interval_hist.num = STONENEEDLE_INTERVAL_BUCKET_NUM;
	dev_mgmt->stoneneedle_chunk_size = stoneneedle_chunk_size;
	INIT_LIST_HEAD(&dev_mgmt->stoneneedle_dev_list);

	pr_info("Register StoneNeedle successfully! Chunk Size: %d\n", dev_mgmt->stoneneedle_chunk_size);
	
	nvme_register_stoneneedle(&ops);
	return 0;
}

static void __exit stoneneedle_exit(void)
{
	struct stoneneedle_dev *sn_dev, *tmp;
	
	if (!dev_mgmt)
		return;
	
	nvme_unregister_stoneneedle();

	list_for_each_entry_safe(sn_dev, tmp, &dev_mgmt->stoneneedle_dev_list,
				 list) {
		pr_info("unregister_stoneneedle, remove proc file: %s \n",
			sn_dev->dev_name);
		ops.release_stoneneedle(sn_dev->dev_name);
	}
	remove_proc_entry(user_root_dir, NULL);
	kfree(dev_mgmt);
}

MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
module_init(stoneneedle_init);
module_exit(stoneneedle_exit);
