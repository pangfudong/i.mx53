/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_asrc.c
 *
 * @brief MXC Asynchronous Sample Rate Converter
 *
 * @ingroup SOUND
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <linux/mxc_asrc.h>
#include <asm/irq.h>
#include <asm/memory.h>
#include <asm/arch/dma.h>

static int asrc_major;
static struct class *asrc_class;
#define ASRC_PROC_PATH        "driver/asrc"

#define ASRC_RATIO_DECIMAL_DEPTH 26

DEFINE_SPINLOCK(data_lock);
DEFINE_SPINLOCK(input_int_lock);
DEFINE_SPINLOCK(output_int_lock);

enum asrc_status {
	ASRC_ASRSTR_AIDEA = 0x01,
	ASRC_ASRSTR_AIDEB = 0x02,
	ASRC_ASRSTR_AIDEC = 0x04,
	ASRC_ASRSTR_AODFA = 0x08,
	ASRC_ASRSTR_AODFB = 0x10,
	ASRC_ASRSTR_AODFC = 0x20,
	ASRC_ASRSTR_AOLE = 0x40,
	ASRC_ASRSTR_FPWT = 0x80,
	ASRC_ASRSTR_AIDUA = 0x100,
	ASRC_ASRSTR_AIDUB = 0x200,
	ASRC_ASRSTR_AIDUC = 0x400,
	ASRC_ASRSTR_AODOA = 0x800,
	ASRC_ASRSTR_AODOB = 0x1000,
	ASRC_ASRSTR_AODOC = 0x2000,
	ASRC_ASRSTR_AIOLA = 0x4000,
	ASRC_ASRSTR_AIOLB = 0x8000,
	ASRC_ASRSTR_AIOLC = 0x10000,
	ASRC_ASRSTR_AOOLA = 0x20000,
	ASRC_ASRSTR_AOOLB = 0x40000,
	ASRC_ASRSTR_AOOLC = 0x80000,
	ASRC_ASRSTR_ATQOA = 0x100000,
	ASRC_ASRSTR_DSLCNT = 0x200000,
};

static const unsigned char asrc_process_table[][8][2] = {
	/* 32kHz 44.1kHz 48kHz   64kHz   88.2kHz 96kHz  128kHz   192kHz */
/*8kHz*/
	{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*12kHz*/
	{{0, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*16kHz*/
	{{0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*24kHz*/
	{{0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*32kHz*/
	{{0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0},},
/*44.1kHz*/
	{{0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0},},
/*48kHz*/
	{{0, 2}, {0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0},},
/*64kHz*/
	{{0, 2}, {0, 2}, {0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0},},
/*88.2kHz*/
	{{1, 2}, {1, 2}, {1, 2}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1},},
/*96kHz*/
	{{1, 2}, {1, 2}, {1, 2}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1},},
/*128kHz*/
	{{1, 2}, {1, 2}, {1, 2}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1},},
/*192kHz*/
	{{2, 2}, {2, 2}, {2, 2}, {2, 1}, {2, 1}, {2, 1}, {2, 1}, {2, 1},},
};

static const unsigned char asrc_divider_table[] = {
/*8kHz 12kHz 16kHz 24kHz 32kHz 44.1kHz 48kHz 64kHz 88.2kHz 96kHz 128kHz 192kHz*/
	0x15, 0x06, 0x14, 0x05, 0x13, 0x04, 0x04, 0x12, 0x03, 0x03, 0x11, 0x02,
};

static struct asrc_data *g_asrc_data;
static struct proc_dir_entry *proc_asrc;
static unsigned long asrc_vrt_base_addr;
static struct mxc_asrc_platform_data *mxc_asrc_data;

static int asrc_set_clock_ratio(enum asrc_pair_index index,
				int input_sample_rate, int output_sample_rate)
{
	int i;
	int integ = 0;
	unsigned long reg_val = 0;

	if (output_sample_rate == 0)
		return -1;

	while (input_sample_rate >= output_sample_rate) {
		input_sample_rate -= output_sample_rate;
		integ++;
	}
	reg_val |= (integ << 26);

	for (i = 1; i <= ASRC_RATIO_DECIMAL_DEPTH; i++) {
		if ((input_sample_rate * 2) >= output_sample_rate) {
			reg_val |= (1 << (ASRC_RATIO_DECIMAL_DEPTH - i));
			input_sample_rate =
			    input_sample_rate * 2 - output_sample_rate;
		} else
			input_sample_rate = input_sample_rate << 1;

		if (input_sample_rate == 0)
			break;
	}

	__raw_writel(reg_val,
		     (asrc_vrt_base_addr + ASRC_ASRIDRLA_REG + (index << 3)));
	__raw_writel((reg_val >> 24),
		     (asrc_vrt_base_addr + ASRC_ASRIDRHA_REG + (index << 3)));

	return 0;
}

static int asrc_set_process_configuration(enum asrc_pair_index index,
					  int input_sample_rate,
					  int output_sample_rate)
{
	int i = 0, j = 0;
	unsigned long reg;

	if (input_sample_rate == 8000)
		i = 0;
	else if (input_sample_rate == 12000)
		i = 1;
	else if (input_sample_rate == 16000)
		i = 2;
	else if (input_sample_rate == 24000)
		i = 3;
	else if (input_sample_rate == 32000)
		i = 4;
	else if (input_sample_rate == 44100)
		i = 5;
	else if (input_sample_rate == 48000)
		i = 6;
	else if (input_sample_rate == 64000)
		i = 7;
	else if (input_sample_rate == 88200)
		i = 8;
	else if (input_sample_rate == 96000)
		i = 9;
	else if (input_sample_rate == 128000)
		i = 10;
	else if (input_sample_rate == 192000)
		i = 11;
	else
		return -1;

	if (output_sample_rate == 32000)
		j = 0;
	else if (output_sample_rate == 44100)
		j = 1;
	else if (output_sample_rate == 48000)
		j = 2;
	else if (output_sample_rate == 64000)
		j = 3;
	else if (output_sample_rate == 88200)
		j = 4;
	else if (output_sample_rate == 96000)
		j = 5;
	else if (output_sample_rate == 128000)
		j = 6;
	else if (output_sample_rate == 192000)
		j = 7;
	else
		return -1;

	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCFG_REG);
	reg &= ~(0x0f << (6 + (index << 2)));
	reg |=
	    ((asrc_process_table[i][j][0] << (6 + (index << 2))) |
	     (asrc_process_table[i][j][1] << (8 + (index << 2))));
	__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCFG_REG);

	if (index == ASRC_PAIR_A) {
		reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCDR1_REG);
		reg &= 0xfc0fc0;
		reg |=
		    (asrc_divider_table[i] | (asrc_divider_table[j + 4] << 12));
		__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCDR1_REG);
	} else if (index == ASRC_PAIR_B) {
		reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCDR1_REG);
		reg &= 0x3f03f;
		reg |=
		    ((asrc_divider_table[i] << 6) |
		     (asrc_divider_table[j + 4] << 18));
		__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCDR1_REG);
	} else {
		reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCDR2_REG);
		reg =
		    (asrc_divider_table[i] | (asrc_divider_table[j + 4] << 6));
		__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCDR2_REG);
	}

	return 0;
}

int asrc_req_pair(int chn_num, enum asrc_pair_index *index)
{
	int err = 0;
	unsigned long lock_flags;
	spin_lock_irqsave(&data_lock, lock_flags);
	if (chn_num > 2) {
		if (g_asrc_data->asrc_pair[ASRC_PAIR_C].active
		    || (chn_num > g_asrc_data->asrc_pair[ASRC_PAIR_C].chn_max))
			err = -EBUSY;
		else {
			*index = ASRC_PAIR_C;
			g_asrc_data->asrc_pair[ASRC_PAIR_C].chn_num = chn_num;
			g_asrc_data->asrc_pair[ASRC_PAIR_C].active = 1;
		}
	} else {
		if (g_asrc_data->asrc_pair[ASRC_PAIR_A].active ||
		    (g_asrc_data->asrc_pair[ASRC_PAIR_A].chn_max == 0)) {
			if (g_asrc_data->asrc_pair[ASRC_PAIR_B].
			    active
			    || (g_asrc_data->asrc_pair[ASRC_PAIR_B].
				chn_max == 0))
				err = -EBUSY;
			else {
				*index = ASRC_PAIR_B;
				g_asrc_data->asrc_pair[ASRC_PAIR_B].chn_num = 2;
				g_asrc_data->asrc_pair[ASRC_PAIR_B].active = 1;
			}
		} else {
			*index = ASRC_PAIR_A;
			g_asrc_data->asrc_pair[ASRC_PAIR_A].chn_num = 2;
			g_asrc_data->asrc_pair[ASRC_PAIR_A].active = 1;
		}
	}
	spin_unlock_irqrestore(&data_lock, lock_flags);
	return err;
}

EXPORT_SYMBOL(asrc_req_pair);

void asrc_release_pair(enum asrc_pair_index index)
{
	unsigned long reg;
	unsigned long lock_flags;
	spin_lock_irqsave(&data_lock, lock_flags);
	g_asrc_data->asrc_pair[index].active = 0;
	/********Disable PAIR*************/
	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	reg &= ~(1 << (index + 1));
	__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	spin_unlock_irqrestore(&data_lock, lock_flags);
}

EXPORT_SYMBOL(asrc_release_pair);

int asrc_config_pair(struct asrc_config *config)
{
	int err = 0;
	int reg, tmp, channel_num;
	unsigned long lock_flags;
	/* Set the channel number */
	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCNCR_REG);
	spin_lock_irqsave(&data_lock, lock_flags);
	g_asrc_data->asrc_pair[config->pair].chn_num = config->channel_num;
	spin_unlock_irqrestore(&data_lock, lock_flags);
	reg &=
	    ~((0xFFFFFFFF >> (32 - mxc_asrc_data->channel_bits)) <<
	      (mxc_asrc_data->channel_bits * config->pair));
	if (mxc_asrc_data->channel_bits > 3)
		channel_num = config->channel_num;
	else
		channel_num = (config->channel_num + 1) / 2;
	tmp = channel_num << (mxc_asrc_data->channel_bits * config->pair);
	reg |= tmp;
	__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCNCR_REG);

	/* Set the clock source */
	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCSR_REG);
	tmp = ~(0x0f << (config->pair << 2));
	reg &= tmp;
	tmp = ~(0x0f << (12 + (config->pair << 2)));
	reg &= tmp;
	reg |=
	    ((config->inclk << (config->pair << 2)) | (config->
						       outclk << (12 +
								  (config->
								   pair <<
								   2))));

	__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCSR_REG);

	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	if ((config->inclk & 0x0f) != INCLK_NONE) {
		reg |= (1 << (20 + config->pair));
		reg &= ~(1 << (14 + (config->pair << 1)));
		__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	} else {
		reg &= ~(1 << (20 + config->pair));
		reg |= (0x03 << (13 + (config->pair << 1)));
		__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCTR_REG);
		err =
		    asrc_set_clock_ratio(config->pair,
					 config->input_sample_rate,
					 config->output_sample_rate);
		if (err < 0)
			return err;
	}

	err = asrc_set_process_configuration(config->pair,
					     config->input_sample_rate,
					     config->output_sample_rate);
	return err;
}

EXPORT_SYMBOL(asrc_config_pair);

void asrc_start_conv(enum asrc_pair_index index)
{
	int reg;
	unsigned long lock_flags;
	spin_lock_irqsave(&data_lock, lock_flags);
	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	if ((reg & 0x0E) == 0)
		clk_enable(mxc_asrc_data->asrc_audio_clk);
	reg |= (1 << (1 + index));
	__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	spin_unlock_irqrestore(&data_lock, lock_flags);
	return;
}

EXPORT_SYMBOL(asrc_start_conv);

void asrc_stop_conv(enum asrc_pair_index index)
{
	int reg;
	unsigned long lock_flags;
	spin_lock_irqsave(&data_lock, lock_flags);
	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	reg &= ~(1 << (1 + index));
	__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCTR_REG);
	if ((reg & 0x0E) == 0)
		clk_disable(mxc_asrc_data->asrc_audio_clk);
	spin_unlock_irqrestore(&data_lock, lock_flags);
	return;
}

EXPORT_SYMBOL(asrc_stop_conv);

/*!
 * @brief asrc interrupt handler
 */
static irqreturn_t asrc_isr(int irq, void *dev_id)
{
	unsigned long status;
	status = __raw_readl(asrc_vrt_base_addr + ASRC_ASRSTR_REG);

	if (status & ASRC_ASRSTR_AOLE) {
		pr_info("asrc overload error\n");
		/*****How to clear it, write 1 to this bit?*/
	}

	if (status & ASRC_ASRSTR_AIDUA)
		pr_info("input data buffer A has underflowed\n");

	if (status & ASRC_ASRSTR_AIDUB)
		pr_info("input data buffer B has underflowed\n");

	if (status & ASRC_ASRSTR_AIDUC)
		pr_info("input data buffer C has underflowed\n");

	if (status & ASRC_ASRSTR_AODOA)
		pr_info("output data buffer A has overflowed\n");

	if (status & ASRC_ASRSTR_AODOB)
		pr_info("output data buffer B has overflowed\n");

	if (status & ASRC_ASRSTR_AODOC)
		pr_info("output data buffer C has overflowed\n");

	return IRQ_HANDLED;
}

static int mxc_init_asrc(void)
{

	/* Halt ASRC internal FP when input FIFO needs data for pair A, B, C */
	__raw_writel(0x0001, asrc_vrt_base_addr + ASRC_ASRCTR_REG);

	/* Enable overflow interrupt */
	__raw_writel(0x00, asrc_vrt_base_addr + ASRC_ASRIER_REG);

	/* Default 6: 2: 2 channel assignment */
	__raw_writel((0x06 << mxc_asrc_data->channel_bits *
		      2) | (0x02 << mxc_asrc_data->channel_bits) | 0x02,
		     asrc_vrt_base_addr + ASRC_ASRCNCR_REG);

	/* Parameter Registers recommended settings */
	__raw_writel(0x7fffff, asrc_vrt_base_addr + ASRC_ASRPM1_REG);
	__raw_writel(0x255555, asrc_vrt_base_addr + ASRC_ASRPM2_REG);
	__raw_writel(0xff7280, asrc_vrt_base_addr + ASRC_ASRPM3_REG);
	__raw_writel(0xff7280, asrc_vrt_base_addr + ASRC_ASRPM4_REG);
	__raw_writel(0xff7280, asrc_vrt_base_addr + ASRC_ASRPM5_REG);

	/* Set the processing clock for 76KHz, 133M  */
	__raw_writel(0x06D6, asrc_vrt_base_addr + ASRC_ASR76K_REG);

	/* Set the processing clock for 56KHz, 133M */
	__raw_writel(0x0947, asrc_vrt_base_addr + ASRC_ASR56K_REG);

	if (request_irq(MXC_INT_ASRC, asrc_isr, 0, "asrc", NULL))
		return -1;

	return 0;
}

static int asrc_get_output_buffer_size(int input_buffer_size,
				       int input_sample_rate,
				       int output_sample_rate)
{
	int i = 0;
	int outbuffer_size = 0;
	int outsample = output_sample_rate;
	while (outsample >= input_sample_rate) {
		++i;
		outsample -= input_sample_rate;
	}
	outbuffer_size = i * input_buffer_size;
	i = 1;
	while (((input_buffer_size >> i) > 2) && (outsample != 0)) {
		if (((outsample << 1) - input_sample_rate) >= 0) {
			outsample = (outsample << 1) - input_sample_rate;
			outbuffer_size += (input_buffer_size >> i);
		} else {
			outsample = outsample << 1;
		}
		i++;
	}
	outbuffer_size = (outbuffer_size >> 3) << 3;
	return outbuffer_size;
}

static void asrc_input_dma_callback(void *data, int error, unsigned int count)
{
	struct asrc_pair_params *params;
	struct dma_block *block;
	mxc_dma_requestbuf_t dma_request;
	unsigned long lock_flags;

	params = data;
	spin_lock_irqsave(&input_int_lock, lock_flags);
	params->input_queue_empty--;
	if (!list_empty(&params->input_queue)) {
		block =
		    list_entry(params->input_queue.next,
			       struct dma_block, queue);
		dma_request.src_addr = (dma_addr_t) block->dma_paddr;
		dma_request.dst_addr =
		    (ASRC_BASE_ADDR + ASRC_ASRDIA_REG + (params->index << 3));
		dma_request.num_of_bytes = block->length;
		mxc_dma_config(params->input_dma_channel, &dma_request,
			       1, MXC_DMA_MODE_WRITE);
		mxc_dma_enable(params->input_dma_channel);
		list_del(params->input_queue.next);
		list_add_tail(&block->queue, &params->input_done_queue);
		params->input_queue_empty++;
	}
	params->input_counter++;
	wake_up_interruptible(&params->input_wait_queue);
	spin_unlock_irqrestore(&input_int_lock, lock_flags);
	return;
}

static void asrc_output_dma_callback(void *data, int error, unsigned int count)
{
	struct asrc_pair_params *params;
	struct dma_block *block;
	mxc_dma_requestbuf_t dma_request;
	unsigned long lock_flags;

	params = data;
	spin_lock_irqsave(&output_int_lock, lock_flags);
	params->output_queue_empty--;

	if (!list_empty(&params->output_queue)) {
		block =
		    list_entry(params->output_queue.next,
			       struct dma_block, queue);
		dma_request.src_addr =
		    (ASRC_BASE_ADDR + ASRC_ASRDOA_REG + (params->index << 3));
		dma_request.dst_addr = (dma_addr_t) block->dma_paddr;
		dma_request.num_of_bytes = block->length;
		mxc_dma_config(params->output_dma_channel, &dma_request,
			       1, MXC_DMA_MODE_READ);
		mxc_dma_enable(params->output_dma_channel);
		list_del(params->output_queue.next);
		list_add_tail(&block->queue, &params->output_done_queue);
		params->output_queue_empty++;
	}

	params->output_counter++;
	wake_up_interruptible(&params->output_wait_queue);
	spin_unlock_irqrestore(&output_int_lock, lock_flags);
	return;
}

static void mxc_free_dma_buf(struct asrc_pair_params *params)
{
	int i;
	for (i = 0; i < ASRC_DMA_BUFFER_NUM; i++) {
		if (params->input_dma[i].dma_vaddr != NULL) {
			dma_free_coherent(0,
					  params->input_buffer_size,
					  params->input_dma[i].
					  dma_vaddr,
					  params->input_dma[i].dma_paddr);
			params->input_dma[i].dma_vaddr = NULL;
		}
		if (params->output_dma[i].dma_vaddr != NULL) {
			dma_free_coherent(0,
					  params->output_buffer_size,
					  params->output_dma[i].
					  dma_vaddr,
					  params->output_dma[i].dma_paddr);
			params->output_dma[i].dma_vaddr = NULL;
		}
	}

	return;
}

static int mxc_allocate_dma_buf(struct asrc_pair_params *params)
{
	int i;

	for (i = 0; i < ASRC_DMA_BUFFER_NUM; i++) {
		params->input_dma[i].dma_vaddr =
		    dma_alloc_coherent(0, params->input_buffer_size,
				       &params->input_dma[i].dma_paddr,
				       GFP_DMA | GFP_KERNEL);
		if (params->input_dma[i].dma_vaddr == NULL) {
			mxc_free_dma_buf(params);
			pr_info("can't allocate buff\n");
			return -ENOBUFS;
		}
	}
	for (i = 0; i < ASRC_DMA_BUFFER_NUM; i++) {
		params->output_dma[i].dma_vaddr =
		    dma_alloc_coherent(0,
				       params->output_buffer_size,
				       &params->output_dma[i].dma_paddr,
				       GFP_DMA | GFP_KERNEL);
		if (params->output_dma[i].dma_vaddr == NULL) {
			mxc_free_dma_buf(params);
			return -ENOBUFS;
		}
	}

	return 0;
}

/*!
 * asrc interface - ioctl function
 *
 * @param inode      struct inode *
 *
 * @param file       struct file *
 *
 * @param cmd    unsigned int
 *
 * @param arg        unsigned long
 *
 * @return           0 success, ENODEV for invalid device instance,
 *                   -1 for other errors.
 */
static int asrc_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct asrc_pair_params *params;
	params = file->private_data;

	if (down_interruptible(&params->busy_lock))
		return -EBUSY;

	switch (cmd) {
	case ASRC_REQ_PAIR:
		{
			struct asrc_req req;
			if (copy_from_user(&req, (void __user *)arg,
					   sizeof(struct asrc_req))) {
				err = -EFAULT;
				break;
			}
			err = asrc_req_pair(req.chn_num, &req.index);
			if (err < 0)
				break;
			params->pair_hold = 1;
			if (copy_to_user
			    ((void __user *)arg, &req, sizeof(struct asrc_req)))
				err = -EFAULT;

			break;
		}
	case ASRC_CONIFG_PAIR:
		{
			struct asrc_config config;
			mxc_dma_device_t rx_id, tx_id;
			char *rx_name, *tx_name;
			int channel = -1;
			if (copy_from_user
			    (&config, (void __user *)arg,
			     sizeof(struct asrc_config))) {
				err = -EFAULT;
				break;
			}
			config.inclk = INCLK_NONE;
			config.outclk = OUTCLK_ASRCK1_CLK;
			err = asrc_config_pair(&config);
			if (err < 0)
				break;
			params->output_buffer_size =
			    asrc_get_output_buffer_size(config.
							dma_buffer_size,
							config.
							input_sample_rate,
							config.
							output_sample_rate);
			params->input_buffer_size = config.dma_buffer_size;
			if (config.buffer_num > ASRC_DMA_BUFFER_NUM)
				params->buffer_num = ASRC_DMA_BUFFER_NUM;
			else
				params->buffer_num = config.buffer_num;
			err = mxc_allocate_dma_buf(params);
			if (err < 0)
				break;

			/* TBD - need to update when new SDMA interface ready */
			if (config.pair == ASRC_PAIR_A) {
				rx_id = MXC_DMA_ASRC_A_RX;
				tx_id = MXC_DMA_ASRC_A_TX;
				rx_name = asrc_pair_id[0];
				tx_name = asrc_pair_id[1];
			} else if (config.pair == ASRC_PAIR_B) {
				rx_id = MXC_DMA_ASRC_B_RX;
				tx_id = MXC_DMA_ASRC_B_TX;
				rx_name = asrc_pair_id[2];
				tx_name = asrc_pair_id[3];
			} else {
				rx_id = MXC_DMA_ASRC_C_RX;
				tx_id = MXC_DMA_ASRC_C_TX;
				rx_name = asrc_pair_id[4];
				tx_name = asrc_pair_id[5];
			}
			channel = mxc_dma_request(rx_id, rx_name);
			params->input_dma_channel = channel;
			err = mxc_dma_callback_set(channel, (mxc_dma_callback_t)
						   asrc_input_dma_callback,
						   (void *)params);
			channel = mxc_dma_request(tx_id, tx_name);
			params->output_dma_channel = channel;
			err = mxc_dma_callback_set(channel, (mxc_dma_callback_t)
						   asrc_output_dma_callback,
						   (void *)params);
			/* TBD - need to update when new SDMA interface ready */
			params->input_queue_empty = 0;
			params->output_queue_empty = 0;
			INIT_LIST_HEAD(&params->input_queue);
			INIT_LIST_HEAD(&params->input_done_queue);
			INIT_LIST_HEAD(&params->output_queue);
			INIT_LIST_HEAD(&params->output_done_queue);
			init_waitqueue_head(&params->input_wait_queue);
			init_waitqueue_head(&params->output_wait_queue);

			if (copy_to_user
			    ((void __user *)arg, &config,
			     sizeof(struct asrc_config)))
				err = -EFAULT;
			break;
		}
	case ASRC_QUERYBUF:
		{
			struct asrc_querybuf buffer;
			if (copy_from_user
			    (&buffer, (void __user *)arg,
			     sizeof(struct asrc_querybuf))) {
				err = -EFAULT;
				break;
			}
			buffer.input_offset =
			    (unsigned long)params->input_dma[buffer.
							     buffer_index].
			    dma_paddr;
			buffer.input_length = params->input_buffer_size;
			buffer.output_offset =
			    (unsigned long)params->output_dma[buffer.
							      buffer_index].
			    dma_paddr;
			buffer.output_length = params->output_buffer_size;
			if (copy_to_user
			    ((void __user *)arg, &buffer,
			     sizeof(struct asrc_querybuf)))
				err = -EFAULT;
			break;
		}
	case ASRC_RELEASE_PAIR:
		{
			enum asrc_pair_index index;
			if (copy_from_user
			    (&index, (void __user *)arg,
			     sizeof(enum asrc_pair_index))) {
				err = -EFAULT;
				break;
			}

			mxc_dma_free(params->input_dma_channel);
			mxc_dma_free(params->output_dma_channel);
			mxc_free_dma_buf(params);
			asrc_release_pair(index);
			params->pair_hold = 0;
			break;
		}
	case ASRC_Q_INBUF:
		{
			struct asrc_buffer buf;
			struct dma_block *block;
			mxc_dma_requestbuf_t dma_request;
			unsigned long lock_flags;
			if (copy_from_user
			    (&buf, (void __user *)arg,
			     sizeof(struct asrc_buffer))) {
				err = -EFAULT;
				break;
			}
			spin_lock_irqsave(&input_int_lock, lock_flags);
			params->input_dma[buf.index].index = buf.index;
			params->input_dma[buf.index].length = buf.length;
			list_add_tail(&params->input_dma[buf.index].
				      queue, &params->input_queue);
			if (params->input_queue_empty == 0) {
				block =
				    list_entry(params->input_queue.next,
					       struct dma_block, queue);
				dma_request.src_addr =
				    (dma_addr_t) block->dma_paddr;
				dma_request.dst_addr =
				    (ASRC_BASE_ADDR + ASRC_ASRDIA_REG +
				     (params->index << 3));
				dma_request.num_of_bytes = block->length;
				mxc_dma_config(params->
					       input_dma_channel,
					       &dma_request, 1,
					       MXC_DMA_MODE_WRITE);
				mxc_dma_enable(params->input_dma_channel);
				params->input_queue_empty++;
				list_del(params->input_queue.next);
				list_add_tail(&block->queue,
					      &params->input_done_queue);
			}
			spin_unlock_irqrestore(&input_int_lock, lock_flags);
			break;
		}
	case ASRC_DQ_INBUF:{
			struct asrc_buffer buf;
			struct dma_block *block;
			unsigned long lock_flags;
			if (copy_from_user
			    (&buf, (void __user *)arg,
			     sizeof(struct asrc_buffer))) {
				err = -EFAULT;
				break;
			}
			if (!wait_event_interruptible_timeout
			    (params->input_wait_queue,
			     params->input_counter != 0, 10 * HZ)) {
				pr_info
				    ("ASRC_DQ_INBUF timeout counter %x\n",
				     params->input_counter);
				err = -ETIME;
				break;
			} else if (signal_pending(current)) {
				pr_info("ASRC_DQ_INBUF interrupt received\n");
				err = -ERESTARTSYS;
				break;
			}
			spin_lock_irqsave(&input_int_lock, lock_flags);
			params->input_counter--;
			block =
			    list_entry(params->input_done_queue.next,
				       struct dma_block, queue);
			list_del(params->input_done_queue.next);
			spin_unlock_irqrestore(&input_int_lock, lock_flags);
			buf.index = block->index;
			buf.length = block->length;
			if (copy_to_user
			    ((void __user *)arg, &buf,
			     sizeof(struct asrc_buffer)))
				err = -EFAULT;

			break;
		}
	case ASRC_Q_OUTBUF:{
			struct asrc_buffer buf;
			struct dma_block *block;
			mxc_dma_requestbuf_t dma_request;
			unsigned long lock_flags;
			if (copy_from_user
			    (&buf, (void __user *)arg,
			     sizeof(struct asrc_buffer))) {
				err = -EFAULT;
				break;
			}
			spin_lock_irqsave(&output_int_lock, lock_flags);
			params->output_dma[buf.index].index = buf.index;
			params->output_dma[buf.index].length = buf.length;
			list_add_tail(&params->output_dma[buf.index].
				      queue, &params->output_queue);
			if (params->output_queue_empty == 0) {
				block =
				    list_entry(params->output_queue.
					       next, struct dma_block, queue);
				dma_request.src_addr =
				    (ASRC_BASE_ADDR + ASRC_ASRDOA_REG +
				     (params->index << 3));
				dma_request.dst_addr =
				    (dma_addr_t) block->dma_paddr;
				dma_request.num_of_bytes = block->length;
				mxc_dma_config(params->
					       output_dma_channel,
					       &dma_request, 1,
					       MXC_DMA_MODE_READ);
				mxc_dma_enable(params->output_dma_channel);
				list_del(params->output_queue.next);
				list_add_tail(&block->queue,
					      &params->output_done_queue);
				params->output_queue_empty++;
			}
			spin_unlock_irqrestore(&output_int_lock, lock_flags);
			break;
		}
	case ASRC_DQ_OUTBUF:{
			struct asrc_buffer buf;
			struct dma_block *block;
			unsigned long lock_flags;
			if (copy_from_user
			    (&buf, (void __user *)arg,
			     sizeof(struct asrc_buffer))) {
				err = -EFAULT;
				break;
			}

			if (!wait_event_interruptible_timeout
			    (params->output_wait_queue,
			     params->output_counter != 0, 10 * HZ)) {
				pr_info
				    ("ASRC_DQ_OUTBUF timeout counter %x\n",
				     params->output_counter);
				err = -ETIME;
				break;
			} else if (signal_pending(current)) {
				pr_info("ASRC_DQ_INBUF interrupt received\n");
				err = -ERESTARTSYS;
				break;
			}
			spin_lock_irqsave(&output_int_lock, lock_flags);
			params->output_counter--;
			block =
			    list_entry(params->output_done_queue.next,
				       struct dma_block, queue);
			list_del(params->output_done_queue.next);
			spin_unlock_irqrestore(&output_int_lock, lock_flags);
			buf.index = block->index;
			buf.length = block->length;
			if (copy_to_user
			    ((void __user *)arg, &buf,
			     sizeof(struct asrc_buffer)))
				err = -EFAULT;

			break;
		}
	case ASRC_START_CONV:{
			enum asrc_pair_index index;
			struct dma_block *block;
			mxc_dma_requestbuf_t dma_request;
			if (copy_from_user
			    (&index, (void __user *)arg,
			     sizeof(enum asrc_pair_index))) {
				err = -EFAULT;
				break;
			}

			if (list_empty(&params->input_queue)) {
				err = -EFAULT;
				pr_info
				    ("ASRC_START_CONV - no block available\n");
				break;
			}
			while (!list_empty(&params->input_queue)) {
				block =
				    list_entry(params->input_queue.next,
					       struct dma_block, queue);

				dma_request.num_of_bytes = block->length;
				dma_request.src_addr =
				    (dma_addr_t) block->dma_paddr;
				dma_request.dst_addr =
				    (ASRC_BASE_ADDR + ASRC_ASRDIA_REG +
				     (index << 3));
				mxc_dma_config(params->input_dma_channel,
					       &dma_request, 1,
					       MXC_DMA_MODE_WRITE);
				params->input_queue_empty++;
				list_del(params->input_queue.next);
				list_add_tail(&block->queue,
					      &params->input_done_queue);
			}

			while (!list_empty(&params->output_queue)) {
				block =
				    list_entry(params->output_queue.next,
					       struct dma_block, queue);
				dma_request.num_of_bytes = block->length;
				dma_request.src_addr =
				    (ASRC_BASE_ADDR + ASRC_ASRDOA_REG +
				     (index << 3));
				dma_request.dst_addr =
				    (dma_addr_t) block->dma_paddr;
				mxc_dma_config(params->output_dma_channel,
					       &dma_request, 1,
					       MXC_DMA_MODE_READ);
				params->output_queue_empty++;
				list_del(params->output_queue.next);
				list_add_tail(&block->queue,
					      &params->output_done_queue);
			}
			params->asrc_active = 1;

			mxc_dma_enable(params->input_dma_channel);
			mxc_dma_enable(params->output_dma_channel);
			asrc_start_conv(index);
			break;
		}
	case ASRC_STOP_CONV:{
			enum asrc_pair_index index;
			if (copy_from_user
			    (&index, (void __user *)arg,
			     sizeof(enum asrc_pair_index))) {
				err = -EFAULT;
				break;
			}

			mxc_dma_disable(params->input_dma_channel);
			mxc_dma_disable(params->output_dma_channel);
			asrc_stop_conv(index);
			params->asrc_active = 0;
			break;
		}
	default:
		break;
	}

	up(&params->busy_lock);
	return err;
}

/*!
 * asrc interface - open function
 *
 * @param inode        structure inode *
 *
 * @param file         structure file *
 *
 * @return  status    0 success, ENODEV invalid device instance,
 *      ENOBUFS failed to allocate buffer, ERESTARTSYS interrupted by user
 */
static int mxc_asrc_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct asrc_pair_params *pair_params;

	if (signal_pending(current))
		return -EINTR;

	pair_params = kzalloc(sizeof(struct asrc_pair_params), GFP_KERNEL);
	if (pair_params == NULL) {
		pr_debug("Failed to allocate pair_params\n");
		err = -ENOBUFS;
	}

	init_MUTEX(&pair_params->busy_lock);
	file->private_data = pair_params;
	return err;
}

/*!
 * asrc interface - close function
 *
 * @param inode    struct inode *
 * @param file        structure file *
 *
 * @return status     0 Success, EINTR busy lock error, ENOBUFS remap_page error
 */
static int mxc_asrc_close(struct inode *inode, struct file *file)
{
	struct asrc_pair_params *pair_params;

	pair_params = file->private_data;
	if (pair_params->asrc_active == 1) {
		mxc_dma_disable(pair_params->input_dma_channel);
		mxc_dma_disable(pair_params->output_dma_channel);
		asrc_stop_conv(pair_params->index);
		wake_up_interruptible(&pair_params->input_wait_queue);
		wake_up_interruptible(&pair_params->output_wait_queue);
	}
	if (pair_params->pair_hold == 1) {
		mxc_dma_free(pair_params->input_dma_channel);
		mxc_dma_free(pair_params->output_dma_channel);
		mxc_free_dma_buf(pair_params);
		asrc_release_pair(pair_params->index);
	}
	kfree(pair_params);
	file->private_data = NULL;
	return 0;
}

/*!
 * asrc interface - mmap function
 *
 * @param file        structure file *
 *
 * @param vma         structure vm_area_struct *
 *
 * @return status     0 Success, EINTR busy lock error, ENOBUFS remap_page error
 */
static int mxc_asrc_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size;
	int res = 0;
	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start,
			    vma->vm_pgoff, size, vma->vm_page_prot))
		return -ENOBUFS;

	vma->vm_flags &= ~VM_IO;
	return res;
}

static struct file_operations asrc_fops = {
	.owner = THIS_MODULE,
	.ioctl = asrc_ioctl,
	.mmap = mxc_asrc_mmap,
	.open = mxc_asrc_open,
	.release = mxc_asrc_close,
};

static int asrc_read_proc_attr(char *page, char **start, off_t off,
			       int count, int *eof, void *data)
{
	unsigned long reg;
	int len = 0;

	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCNCR_REG);

	len += sprintf(page, "ANCA: %d\n",
		       (int)(reg &
			     (0xFFFFFFFF >>
			      (32 - mxc_asrc_data->channel_bits))));
	len +=
	    sprintf(page + len, "ANCB: %d\n",
		    (int)((reg >> mxc_asrc_data->
			   channel_bits) & (0xFFFFFFFF >> (32 -
							   mxc_asrc_data->
							   channel_bits))));
	len +=
	    sprintf(page + len, "ANCC: %d\n",
		    (int)((reg >> (mxc_asrc_data->channel_bits * 2)) &
			  (0xFFFFFFFF >> (32 - mxc_asrc_data->channel_bits))));

	if (off > len)
		return 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return min(count, len - (int)off);
}

static int asrc_write_proc_attr(struct file *file, const char *buffer,
				unsigned long count, void *data)
{
	char buf[50];
	unsigned long reg;
	int na, nb, nc;
	int total;
	if (count > 48)
		return -EINVAL;

	if (copy_from_user(buf, buffer, count)) {
		pr_debug("Attr proc write, Failed to copy buffer from user\n");
		return -EFAULT;
	}

	reg = __raw_readl(asrc_vrt_base_addr + ASRC_ASRCNCR_REG);
	sscanf(buf, "ANCA: %d\nANCB: %d\nANCC: %d", &na, &nb, &nc);
	if (mxc_asrc_data->channel_bits > 3)
		total = 10;
	else
		total = 5;
	if ((na + nb + nc) != total) {
		pr_info("Wrong ASRCNR settings\n");
		return -EFAULT;
	}
	reg = na | (nb << mxc_asrc_data->
		    channel_bits) | (nc << (mxc_asrc_data->channel_bits * 2));

	__raw_writel(reg, asrc_vrt_base_addr + ASRC_ASRCNCR_REG);

	return count;
}

static void asrc_proc_create(void)
{
	struct proc_dir_entry *proc_attr;

	proc_asrc = proc_mkdir(ASRC_PROC_PATH, NULL);
	if (proc_asrc) {
		proc_attr = create_proc_entry("ChSettings",
					      S_IFREG | S_IRUGO |
					      S_IWUSR, proc_asrc);
		if (proc_attr) {
			proc_attr->read_proc = asrc_read_proc_attr;
			proc_attr->write_proc = asrc_write_proc_attr;
			proc_attr->size = 48;
			proc_attr->uid = proc_attr->gid = 0;
			proc_attr->owner = THIS_MODULE;
		} else {
			pr_info("Failed to create proc attribute entry \n");
		}
	} else {
		pr_info("ASRC: Failed to create proc entry %s\n",
			ASRC_PROC_PATH);
	}
}

/*!
 * Entry point for the asrc device
 *
 * @param	pdev Pionter to the registered platform device
 * @return  Error code indicating success or failure
 */
static int mxc_asrc_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	struct device *temp_class;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	g_asrc_data = kzalloc(sizeof(struct asrc_data), GFP_KERNEL);

	if (g_asrc_data == NULL) {
		pr_info("Failed to allocate g_asrc_data\n");
		return -ENOMEM;
	}

	g_asrc_data->asrc_pair[0].chn_max = 2;
	g_asrc_data->asrc_pair[1].chn_max = 2;
	g_asrc_data->asrc_pair[2].chn_max = 6;

	asrc_major = register_chrdev(asrc_major, "mxc_asrc", &asrc_fops);
	if (asrc_major < 0) {
		pr_info("Unable to register asrc device\n");
		err = -EBUSY;
		goto error;
	}

	asrc_class = class_create(THIS_MODULE, "mxc_asrc");
	if (IS_ERR(asrc_class)) {
		err = PTR_ERR(asrc_class);
		goto err_out_chrdev;
	}

	temp_class = device_create(asrc_class, NULL, MKDEV(asrc_major, 0),
				   "mxc_asrc");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	asrc_vrt_base_addr =
	    (unsigned long)ioremap(res->start, res->end - res->start + 1);

	mxc_asrc_data =
	    (struct mxc_asrc_platform_data *)pdev->dev.platform_data;
	clk_enable(mxc_asrc_data->asrc_core_clk);

	asrc_proc_create();
	err = mxc_init_asrc();
	if (err < 0)
		goto err_out_class;

	goto out;

      err_out_class:
	clk_disable(mxc_asrc_data->asrc_core_clk);
	device_destroy(asrc_class, MKDEV(asrc_major, 0));
	class_destroy(asrc_class);
      err_out_chrdev:
	unregister_chrdev(asrc_major, "mxc_asrc");
      error:
	kfree(g_asrc_data);
      out:
	pr_info("mxc_asrc registered\n");
	return err;
}

/*!
 * Exit asrc device
 *
 * @param	pdev Pionter to the registered platform device
 * @return  Error code indicating success or failure
 */
static int mxc_asrc_remove(struct platform_device *pdev)
{
	free_irq(MXC_INT_ASRC, NULL);
	kfree(g_asrc_data);
	clk_disable(mxc_asrc_data->asrc_core_clk);
	mxc_asrc_data = NULL;
	iounmap((unsigned long __iomem *)asrc_vrt_base_addr);
	remove_proc_entry(proc_asrc->name, NULL);
	device_destroy(asrc_class, MKDEV(asrc_major, 0));
	class_destroy(asrc_class);
	unregister_chrdev(asrc_major, "mxc_asrc");
	return 0;
}

/*! mxc asrc driver definition
 *
 */
static struct platform_driver mxc_asrc_driver = {
	.driver = {
		   .name = "mxc_asrc",
		   },
	.probe = mxc_asrc_probe,
	.remove = mxc_asrc_remove,
};

/*!
 * Register asrc driver
 *
 */
static __init int asrc_init(void)
{
	int ret;
	ret = platform_driver_register(&mxc_asrc_driver);
	return ret;
}

/*!
 * Exit and free the asrc data
 *
 */ static void __exit asrc_exit(void)
{
	platform_driver_unregister(&mxc_asrc_driver);
	return;
}

module_init(asrc_init);
module_exit(asrc_exit);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Asynchronous Sample Rate Converter");
MODULE_LICENSE("GPL");
