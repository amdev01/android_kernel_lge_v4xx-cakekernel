/*
 * android vibrator driver
 *
 * Copyright (C) 2009-2012 LGE, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifdef CONFIG_ANDROID_SW_IRRC
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/android_irrc.h>
#include <linux/spinlock.h>
#include "../staging/android/timed_output.h"
#include <linux/types.h>
#include <linux/err.h>
#include <mach/msm_iomap.h>
#include <linux/io.h>
#include <mach/gpiomux.h>
#include <mach/board_lge.h>
#include <linux/i2c.h>
#include <mach/msm_xo.h>
#include <linux/slab.h>

#include <linux/ioctl.h>
#include <asm/ioctls.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/ktime.h>
#include <linux/seq_file.h>
#include <linux/math64.h>
#include <linux/preempt.h>
#include <asm/processor.h> /* cpu_relax() */

/*
    For ADB debugging
    if you want to turn on with pwm clock(33Khz), duty(50%), #echo 1 33 50 > /sys/kernel/debug/sw_irrc/poke
    if you want to turn on with gpio level high.                      #echo 1 0 0 > /sys/kernel/debug/sw_irrc/poke
    if you want to turn off,                                                #echo 0 33 50 > /sys/kernel/debug/sw_irrc/poke
*/

#define REG_WRITEL(value, reg)		writel(value, reg)
#define REG_READL(reg)				readl(reg)

#define MMSS_GP0_CMD_RCGR(x) (void __iomem *)(virt_bases_v + (x))
#define MMSS_CC_PWM_SIZE	SZ_1K

struct timed_irrc_data {
	struct platform_device dev;

	struct regulator *vreg;
	struct regulator *vreg2;

	unsigned int gp_cmd_rcgr;
	int pwm_gpio;
	int pwm_gpio_func;

	struct clk *gp_clk;
	const char *clk_name;
	unsigned int clk_rate;

    struct workqueue_struct *workqueue;
    struct delayed_work gpio_off_work;
};

struct irrc_compr_params {
	int frequency;
	int duty;
	int length;
};

static void __iomem *virt_bases_v = NULL;
static struct timed_irrc_data *irrc_data_ptr;
static struct platform_device *irrc_dev_ptr;
static int gpio_high_flag = 0;
static bool g_pwm_enabled = false;
/* Regulators stay on across mark/space; only PWM/clk is gated per pulse. */
static bool g_irrc_powered = false;
/* Mux + gp_clk stay prepared across a transmit burst; disarm after idle. */
static bool g_irrc_clk_armed = false;
static int g_pwm_clk;
static int g_pwm_duty;
/* Cached RCGR N/D so mark edges only toggle ROOT_EN when carrier unchanged. */
static int g_pwm_n = -1;
static int g_pwm_d = -1;
static bool g_pwm_root_on = false;

/*
 * Hot-path timing capture (no printk). Read after a transmit:
 *   cat /sys/kernel/debug/sw_irrc/timing
 * Write "reset" to clear. NEC expects ~560/1690 us marks and ~560/4500/9000 spaces.
 */
#define IRRC_TIMING_MAX 256
static u64 g_edge_ns[IRRC_TIMING_MAX];
static u8 g_edge_on[IRRC_TIMING_MAX];
static unsigned g_edge_count;
static unsigned g_edge_dropped;
static DEFINE_SPINLOCK(g_timing_lock);

static int android_irrc_set_pwm(int enable, int PWM_CLK, int duty);

static void android_irrc_timing_reset(void)
{
	unsigned long flags;

	spin_lock_irqsave(&g_timing_lock, flags);
	g_edge_count = 0;
	g_edge_dropped = 0;
	spin_unlock_irqrestore(&g_timing_lock, flags);
}

static void android_irrc_timing_edge(int on)
{
	unsigned long flags;
	u64 now = ktime_to_ns(ktime_get());

	spin_lock_irqsave(&g_timing_lock, flags);
	if (g_edge_count < IRRC_TIMING_MAX) {
		g_edge_ns[g_edge_count] = now;
		g_edge_on[g_edge_count] = on ? 1 : 0;
		g_edge_count++;
	} else {
		g_edge_dropped++;
	}
	spin_unlock_irqrestore(&g_timing_lock, flags);
}

static void android_irrc_power_on(struct timed_irrc_data *irrc)
{
	int rc;
	bool ok = true;

	if (g_irrc_powered)
		return;

	if (irrc->vreg != NULL) {
		rc = regulator_enable(irrc->vreg);
		if (rc < 0) {
			ERR_MSG("regulator_enable failed\n");
			ok = false;
		}
	}
	if (irrc->vreg2 != NULL) {
		rc = regulator_enable(irrc->vreg2);
		if (rc < 0) {
			ERR_MSG("regulator_enable failed2\n");
			ok = false;
		}
	}
	if (ok)
		g_irrc_powered = true;
}

static void android_irrc_power_off(struct timed_irrc_data *irrc)
{
	int rc;

	if (!g_irrc_powered)
		return;

	if (irrc->vreg != NULL && regulator_is_enabled(irrc->vreg) > 0) {
		rc = regulator_disable(irrc->vreg);
		if (rc < 0)
			ERR_MSG("regulator_disable failed\n");
	}
	if (irrc->vreg2 != NULL && regulator_is_enabled(irrc->vreg2) > 0) {
		rc = regulator_disable(irrc->vreg2);
		if (rc < 0)
			ERR_MSG("regulator_disable failed2\n");
	}
	g_irrc_powered = false;
}

static void android_irrc_disarm(struct timed_irrc_data *irrc)
{
	if (gpio_high_flag == 1) {
		gpio_set_value(irrc->pwm_gpio, 0);
	} else if (g_irrc_clk_armed) {
		android_irrc_set_pwm(0, g_pwm_clk, g_pwm_duty);
		clk_disable_unprepare(irrc->gp_clk);
		g_irrc_clk_armed = false;
	}
	gpio_high_flag = 0;
	g_pwm_enabled = false;
	g_pwm_n = -1;
	g_pwm_d = -1;
	g_pwm_root_on = false;
}

/*
 * Gate carrier on/off within a burst. Spaces only clear ROOT_EN; mux/clk stay
 * armed until android_irrc_disarm() after idle.
 */
static void android_irrc_pwm_gate(struct timed_irrc_data *irrc, int on,
		int PWM_CLK, int duty)
{
	if (gpio_high_flag == 1) {
		gpio_set_value(irrc->pwm_gpio, on ? 1 : 0);
		return;
	}

	if (on) {
		if (!g_irrc_clk_armed) {
			gpio_tlmm_config(GPIO_CFG(irrc->pwm_gpio,
						irrc->pwm_gpio_func,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
			clk_prepare_enable(irrc->gp_clk);
			g_irrc_clk_armed = true;
			/* Force RCGR N/D program on first mark of a burst. */
			g_pwm_n = -1;
			g_pwm_d = -1;
			g_pwm_root_on = false;
		}
		g_pwm_clk = PWM_CLK;
		g_pwm_duty = duty;
		android_irrc_set_pwm(1, PWM_CLK, duty);
	} else if (g_irrc_clk_armed) {
		android_irrc_set_pwm(0, g_pwm_clk, g_pwm_duty);
	}
}

static struct gpiomux_setting irrc_active = {
	.func = 0, //[WX project] The value will be from device tree. GPIO for GP clock has alternative function.
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting irrc_suspend = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config irrc_config[] = {
	{
		.gpio = 0, //[WX project] The value will be from device tree. GPIO_IRRC_PWM gpio number
		.settings = {
			[GPIOMUX_ACTIVE] =    &irrc_active,
			[GPIOMUX_SUSPENDED] = &irrc_suspend,

		},
	},
};

static int android_irrc_set_pwm(int enable,int PWM_CLK, int duty)
{
	int M_VAL = 1;
	int N_VAL = 1;
	int D_VAL = 1;

	N_VAL = (9600+PWM_CLK)/(PWM_CLK*2); //Formular in case SRC is 19.2Mhz. N_VAL = SRC/(div*PWM_CLK) + 0.5
	D_VAL = (N_VAL*duty+50)/100;
	if (D_VAL == 0)
		D_VAL = 1;

	INFO_MSG("enable:%d, pwm_clk:%d, duty:%d, M:%d,N:%d,D:%d\n", enable,PWM_CLK,duty, M_VAL,N_VAL,D_VAL);

	if (enable) {
		/* N/D stay valid across ROOT_EN clear; only rewrite if carrier changed. */
		if (N_VAL != g_pwm_n || D_VAL != g_pwm_d) {
			REG_WRITEL(
				((~(N_VAL-M_VAL)) & 0xffU),	/* N[7:0] */
				MMSS_GP0_CMD_RCGR(0x0C));
			REG_WRITEL(
				((~(D_VAL << 1)) & 0xffU),	/* D[7:0] */
				MMSS_GP0_CMD_RCGR(0x10));
			g_pwm_n = N_VAL;
			g_pwm_d = D_VAL;
		}
		REG_WRITEL(
			(1 << 1U) + /* ROOT_EN[1] */
			(1),		/* UPDATE[0] */
			MMSS_GP0_CMD_RCGR(0));
		g_pwm_root_on = true;
	} else {
		REG_WRITEL(
			(0 << 1U) + /* ROOT_EN[1] */
			(0),		/* UPDATE[0] */
			MMSS_GP0_CMD_RCGR(0));
		g_pwm_root_on = false;
	}
	return 0;
}

static void android_irrc_enable_pwm(struct timed_irrc_data *irrc, int PWM_CLK, int duty)
{
	/*
	 * Non-sync cancel: never sleep on the mark/space hot path. Pending idle
	 * work is dropped; an already-running disarm finishes and the next
	 * enable re-arms clk/rails as needed.
	 */
	cancel_delayed_work(&irrc->gpio_off_work);

	/* New burst after idle — reset timing capture for debugfs. */
	if (!g_pwm_enabled && !g_irrc_clk_armed)
		android_irrc_timing_reset();

	android_irrc_power_on(irrc);

	if ((PWM_CLK == 0) || (duty == 100)) {
		INFO_MSG("gpio set to high!!!\n");

		if (gpio_high_flag != 1) {
			if (g_irrc_clk_armed)
				android_irrc_disarm(irrc);
			gpio_tlmm_config(GPIO_CFG(irrc->pwm_gpio, 0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
			gpio_high_flag = 1;
		}
		gpio_set_value(irrc->pwm_gpio, 1);

	} else if ((PWM_CLK < 23) || (PWM_CLK > 1200) ||
			(duty > 60) || (duty < 20)) {
		ERR_MSG("Out of range: pwm_clk=%d duty=%d\n", PWM_CLK, duty);
		return;

	} else {
		INFO_MSG("gpio set to gp!!!\n");

		if (gpio_high_flag == 1) {
			gpio_set_value(irrc->pwm_gpio, 0);
			gpio_high_flag = 0;
		}
		android_irrc_pwm_gate(irrc, 1, PWM_CLK, duty);
	}
	g_pwm_enabled = true;
	android_irrc_timing_edge(1);
}

static void android_irrc_gate_carrier_off(struct timed_irrc_data *irrc)
{
	if (!g_pwm_enabled)
		return;

	android_irrc_pwm_gate(irrc, 0, g_pwm_clk, g_pwm_duty);
	g_pwm_enabled = false;
	android_irrc_timing_edge(0);
}

/*
 * Busy-wait for pattern marks/spaces. usleep_range / timer slack is ~tens of
 * ms on this platform and cannot reproduce NEC (~560 us) edges.
 *
 * Use only addition on ktime (CONFIG_KTIME_SCALAR) — never u64 / u64, which
 * pulls in __aeabi_uldivmod on ARM EABI and is not linked into the kernel.
 * Hold preempt only for short NEC marks/spaces; longer gaps (>3 ms) may
 * schedule so we do not soft-lock the CPU on inter-frame delays.
 */
static void irrc_busy_wait_us(unsigned int us)
{
	ktime_t end = ktime_add_ns(ktime_get(), (u64)us * 1000ULL);
	int tight = (us <= 3000);

	if (tight)
		preempt_disable();
	while (ktime_compare(ktime_get(), end) < 0)
		cpu_relax();
	if (tight)
		preempt_enable();
}

static int android_irrc_transmit(struct timed_irrc_data *irrc,
		struct irrc_transmit_params *params, const int *pattern)
{
	int freq_khz = params->frequency / 1000;
	int duty = params->duty;
	int gpio_mode;
	int i;

	gpio_mode = (freq_khz == 0) || (duty == 100);

	if (!gpio_mode && ((freq_khz < 23) || (freq_khz > 1200) ||
			(duty > 60) || (duty < 20))) {
		ERR_MSG("Out of range: pwm_clk=%d duty=%d\n", freq_khz, duty);
		return -EINVAL;
	}

	/*
	 * Must sync-cancel: a running idle disarm mid-pattern would clear
	 * g_irrc_clk_armed and force clk_prepare_enable() from pwm_gate while
	 * preempt is disabled in the busy-wait.
	 */
	cancel_delayed_work_sync(&irrc->gpio_off_work);
	android_irrc_timing_reset();
	android_irrc_power_on(irrc);

	/* Arm mux/clk (or GPIO) once before the pattern loop (may sleep). */
	if (gpio_mode) {
		if (gpio_high_flag != 1) {
			if (g_irrc_clk_armed)
				android_irrc_disarm(irrc);
			gpio_tlmm_config(GPIO_CFG(irrc->pwm_gpio, 0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
			gpio_high_flag = 1;
		}
	} else {
		if (gpio_high_flag == 1) {
			gpio_set_value(irrc->pwm_gpio, 0);
			gpio_high_flag = 0;
		}
		if (!g_irrc_clk_armed) {
			gpio_tlmm_config(GPIO_CFG(irrc->pwm_gpio,
						irrc->pwm_gpio_func,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
			clk_prepare_enable(irrc->gp_clk);
			g_irrc_clk_armed = true;
			g_pwm_n = -1;
			g_pwm_d = -1;
			g_pwm_root_on = false;
		}
		g_pwm_clk = freq_khz;
		g_pwm_duty = duty;
	}

	/* Even index = mark (carrier on), odd = space — ConsumerIr contract. */
	for (i = 0; i < params->count; i++) {
		int on = ((i & 1) == 0);

		if (on) {
			android_irrc_pwm_gate(irrc, 1, freq_khz, duty);
			g_pwm_enabled = true;
			android_irrc_timing_edge(1);
		} else {
			android_irrc_pwm_gate(irrc, 0, freq_khz, duty);
			g_pwm_enabled = false;
			android_irrc_timing_edge(0);
		}

		if (pattern[i] > 0)
			irrc_busy_wait_us((unsigned int)pattern[i]);
	}

	/* Ensure carrier off and schedule idle disarm. */
	android_irrc_gate_carrier_off(irrc);
	queue_delayed_work(irrc->workqueue, &irrc->gpio_off_work,
			msecs_to_jiffies(100));

	return 0;
}

static void android_irrc_disable_pwm(struct work_struct *work)
{
	struct timed_irrc_data *irrc = container_of(work, struct timed_irrc_data,
			gpio_off_work.work);

	/*
	 * Delayed idle work: full disarm (clk/mux) then rails. Carrier is gated
	 * synchronously on IRRC_STOP / poke-off so mark/space stays accurate.
	 */
	if (!g_pwm_enabled) {
		android_irrc_disarm(irrc);
		android_irrc_power_off(irrc);
	}
}

static int android_irrc_open(struct inode *inode, struct file *file)
{
	struct timed_irrc_data *irrc = platform_get_drvdata(irrc_dev_ptr);
	file->private_data = irrc;

	return 0;
}

static int android_irrc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t android_irrc_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	return 0;
}

#ifdef CONFIG_LGE_SW_IRRC_MUTE_SPEAKER
extern void mute_spk_for_swirrc (int enable);
#endif

static long android_irrc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct timed_irrc_data *irrc = file->private_data;
	struct irrc_compr_params test;
	struct irrc_transmit_params xmit;
	int *pattern = NULL;
	unsigned long total_us;
	int i;
	int rc = 0;

	switch (cmd) {
	case IRRC_START:
		rc = copy_from_user(&test, (void __user *)arg, sizeof(test));

		INFO_MSG("IRRC_START: freq:%d, duty:%d\n", test.frequency/1000, test.duty);
		android_irrc_enable_pwm(irrc, test.frequency/1000, test.duty);
#ifdef CONFIG_LGE_SW_IRRC_MUTE_SPEAKER
		mute_spk_for_swirrc (1);
#endif
		break;

	case IRRC_STOP:
		INFO_MSG("IRRC_STOP\n");
		/*
		 * Gate carrier immediately (ROOT_EN off / gpio low) without
		 * clk_disable. Full disarm + rails drop after 100 ms idle.
		 * Use non-sync cancel — never block the space edge.
		 */
		cancel_delayed_work(&irrc->gpio_off_work);
		android_irrc_gate_carrier_off(irrc);
		queue_delayed_work(irrc->workqueue, &irrc->gpio_off_work,
				msecs_to_jiffies(100));
#ifdef CONFIG_LGE_SW_IRRC_MUTE_SPEAKER
		mute_spk_for_swirrc (0);
#endif
		break;

	case IRRC_TRANSMIT:
		if (copy_from_user(&xmit, (void __user *)arg, sizeof(xmit)))
			return -EFAULT;

		if (xmit.count <= 0 || xmit.count > IRRC_TRANSMIT_MAX_COUNT)
			return -EINVAL;
		if (!xmit.pattern)
			return -EINVAL;

		pattern = kmalloc(sizeof(*pattern) * xmit.count, GFP_KERNEL);
		if (!pattern)
			return -ENOMEM;

		if (copy_from_user(pattern, (void __user *)xmit.pattern,
				sizeof(*pattern) * xmit.count)) {
			kfree(pattern);
			return -EFAULT;
		}

		total_us = 0;
		for (i = 0; i < xmit.count; i++) {
			if (pattern[i] < 0) {
				kfree(pattern);
				return -EINVAL;
			}
			total_us += (unsigned int)pattern[i];
			if (total_us > IRRC_TRANSMIT_MAX_DURATION_US) {
				kfree(pattern);
				return -EINVAL;
			}
		}

		INFO_MSG("IRRC_TRANSMIT: freq:%d, duty:%d, count:%d\n",
				xmit.frequency / 1000, xmit.duty, xmit.count);
#ifdef CONFIG_LGE_SW_IRRC_MUTE_SPEAKER
		mute_spk_for_swirrc(1);
#endif
		rc = android_irrc_transmit(irrc, &xmit, pattern);
#ifdef CONFIG_LGE_SW_IRRC_MUTE_SPEAKER
		mute_spk_for_swirrc(0);
#endif
		kfree(pattern);
		break;

	default:
	    INFO_MSG("CMD ERROR: cmd:%d\n", cmd);
		rc = -EINVAL;
	}

	return rc;
}

static int android_irrc_pcm_fsync(struct file *file, loff_t a, loff_t b, int datasync)
{
	return 0;
}

static const struct file_operations IRRC_pcm_fops = {
	.owner		= THIS_MODULE,
	.open		= android_irrc_open,
	.release	= android_irrc_release,
	.write		= android_irrc_write,
	.unlocked_ioctl	= android_irrc_ioctl,
	.fsync = android_irrc_pcm_fsync,
};

struct miscdevice irrc_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_IRRC_pcm_dec",
	.fops	= &IRRC_pcm_fops,
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_wcd9xxx_dent;
static struct dentry *debugfs_poke;
static struct dentry *debugfs_timing;

static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt = 0;
	token = strsep(&buf, " ");

	while (token != NULL) {
		if ((strlen(token) > 2) && ((token[1] == 'x') || (token[1] == 'X')))
			base = 16;
		else
			base = 10;

		if (strict_strtoul(token, base, &param1[cnt]) != 0){
			ERR_MSG("strict_strtoul error!!\n");
			return -EINVAL;
		}
		cnt ++;
		token = strsep(&buf, " ");
	}
	return 0;
}

static ssize_t codec_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *access_str = filp->private_data;
	char lbuf[32];
	int rc;
	long int param[5] = {0,};
	struct timed_irrc_data *irrc = platform_get_drvdata(irrc_dev_ptr);

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';

	//INFO_MSG("access_str:%s lbuf:%s cnt:%d\n", access_str, lbuf, cnt);

	if (!strncmp(access_str, "poke", 6)) {
		rc = get_parameters(lbuf, param, 3);
		if (rc) {
			ERR_MSG("error!!! get_parameters rc = %d\n", rc);
			return rc;
		}

		switch (param[0]) {
		case 1:
			INFO_MSG("IRRC_START\n");
			android_irrc_enable_pwm(irrc, param[1], param[2]);
			break;

		case 0:
			INFO_MSG("IRRC_STOP\n");
			cancel_delayed_work(&irrc->gpio_off_work);
			android_irrc_gate_carrier_off(irrc);
			queue_delayed_work(irrc->workqueue, &irrc->gpio_off_work,
					msecs_to_jiffies(100));
			break;
		default:
			rc = -EINVAL;
		}

	}

	if (rc == 0)
		rc = cnt;
	else
		ERR_MSG("rc = %d\n", rc);

	return rc;
}

static const struct file_operations codec_debug_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
};

static int irrc_timing_show(struct seq_file *s, void *unused)
{
	unsigned long flags;
	unsigned i, n;
	u64 *edges;
	u8 *on;
	unsigned dropped;
	u64 mark_min = ~0ULL, mark_max = 0, mark_sum = 0;
	u64 space_min = ~0ULL, space_max = 0, space_sum = 0;
	unsigned marks = 0, spaces = 0;

	edges = kmalloc(sizeof(*edges) * IRRC_TIMING_MAX, GFP_KERNEL);
	on = kmalloc(sizeof(*on) * IRRC_TIMING_MAX, GFP_KERNEL);
	if (!edges || !on) {
		kfree(edges);
		kfree(on);
		return -ENOMEM;
	}

	spin_lock_irqsave(&g_timing_lock, flags);
	n = g_edge_count;
	dropped = g_edge_dropped;
	for (i = 0; i < n; i++) {
		edges[i] = g_edge_ns[i];
		on[i] = g_edge_on[i];
	}
	spin_unlock_irqrestore(&g_timing_lock, flags);

	seq_printf(s, "edges=%u dropped=%u (IRRC_INFO_PRINT=0; durations in us)\n",
			n, dropped);
	seq_printf(s, "# idx state delta_us abs_us\n");

	for (i = 0; i < n; i++) {
		u64 abs_us = div_u64(edges[i], 1000);
		u64 delta_us = 0;

		if (i > 0)
			delta_us = div_u64(edges[i] - edges[i - 1], 1000);

		seq_printf(s, "%u %s %llu %llu\n", i,
				on[i] ? "mark" : "space",
				(unsigned long long)delta_us,
				(unsigned long long)abs_us);

		/* Duration of completed pulse is delta into the next opposite edge. */
		if (i > 0) {
			if (on[i - 1]) {
				if (delta_us < mark_min)
					mark_min = delta_us;
				if (delta_us > mark_max)
					mark_max = delta_us;
				mark_sum += delta_us;
				marks++;
			} else {
				if (delta_us < space_min)
					space_min = delta_us;
				if (delta_us > space_max)
					space_max = delta_us;
				space_sum += delta_us;
				spaces++;
			}
		}
	}

	if (marks) {
		seq_printf(s, "mark_us: n=%u min=%llu avg=%llu max=%llu (NEC ~560/1690)\n",
				marks,
				(unsigned long long)mark_min,
				(unsigned long long)div_u64(mark_sum, marks),
				(unsigned long long)mark_max);
	}
	if (spaces) {
		seq_printf(s, "space_us: n=%u min=%llu avg=%llu max=%llu (NEC ~560/4500/9000)\n",
				spaces,
				(unsigned long long)space_min,
				(unsigned long long)div_u64(space_sum, spaces),
				(unsigned long long)space_max);
	}

	kfree(edges);
	kfree(on);
	return 0;
}

static int irrc_timing_open(struct inode *inode, struct file *file)
{
	return single_open(file, irrc_timing_show, inode->i_private);
}

static ssize_t irrc_timing_write(struct file *file, const char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	char lbuf[16];

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;
	if (copy_from_user(lbuf, ubuf, cnt))
		return -EFAULT;
	lbuf[cnt] = '\0';
	if (!strncmp(lbuf, "reset", 5))
		android_irrc_timing_reset();
	return cnt;
}

static const struct file_operations irrc_timing_ops = {
	.owner = THIS_MODULE,
	.open = irrc_timing_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = irrc_timing_write,
};
#endif

#ifdef CONFIG_OF
static void android_irrc_parse_dt(struct device *dev, struct timed_irrc_data *data)
{
	struct device_node *np = dev->of_node;
	int len;

	of_property_read_u32(np, "lge,gp-cmd-rcgr", &data->gp_cmd_rcgr);
	data->pwm_gpio = of_get_named_gpio_flags(np, "lge,pwm-gpio", 0, NULL);
	of_property_read_u32(np, "lge,pwm-gpio-func", &data->pwm_gpio_func);
	data->clk_name = of_get_property(np, "lge,clk-name", &len);
	of_property_read_u32(np, "lge,clk-rate", &data->clk_rate);

	PROBE_MSG("rcgr:%x, gpio:%d, gpio-func:%d, clk name: %s, clk rate: %u\n",
			data->gp_cmd_rcgr, data->pwm_gpio, data->pwm_gpio_func, data->clk_name, data->clk_rate);
}

static struct of_device_id irrc_match_table[] = {
    { .compatible = "lge,sw_irrc",},
    { },
};
#endif

static void android_irrc_install(int pwm_gpio, int pwm_gpio_func)
{
	irrc_config[0].gpio = pwm_gpio;
	switch(pwm_gpio_func){
		case 0: irrc_active.func = GPIOMUX_FUNC_GPIO; break;
		case 1: irrc_active.func = GPIOMUX_FUNC_1; break;
		case 2: irrc_active.func = GPIOMUX_FUNC_2; break;
		case 3: irrc_active.func = GPIOMUX_FUNC_3; break;
		case 4: irrc_active.func = GPIOMUX_FUNC_4; break;
		case 5: irrc_active.func = GPIOMUX_FUNC_5; break;
		case 6: irrc_active.func = GPIOMUX_FUNC_6; break;
		case 7: irrc_active.func = GPIOMUX_FUNC_7; break;
		case 8: irrc_active.func = GPIOMUX_FUNC_8; break;
		case 9: irrc_active.func = GPIOMUX_FUNC_9; break;
		default:
			irrc_active.func = GPIOMUX_FUNC_GPIO;
			ERR_MSG("gpiomux function error!\n");
			break;
	}
	msm_gpiomux_install(irrc_config, ARRAY_SIZE(irrc_config));
}

static int android_irrc_probe(struct platform_device *pdev)
{
	int rc;
	struct timed_irrc_data *irrc;

	PROBE_MSG("probe\n");

	irrc = kzalloc(sizeof(struct timed_irrc_data), GFP_KERNEL);
	if (irrc == NULL) {
		ERR_MSG("Can not allocate memory.\n");
		goto err_1;
	}

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		android_irrc_parse_dt(&pdev->dev, irrc);
	}
#endif
	android_irrc_install(irrc->pwm_gpio, irrc->pwm_gpio_func);

	rc = gpio_request(irrc->pwm_gpio, "irrc_pwm");

	if (rc) {
		ERR_MSG("IRRC GPIO set failed.\n");
		goto err_2;
	}

	virt_bases_v = ioremap(irrc->gp_cmd_rcgr, MMSS_CC_PWM_SIZE);

	rc = misc_register(&irrc_misc);
	if (rc) {
		ERR_MSG("misc_register failed.\n");
		goto err_3;
	}

	irrc->workqueue = create_workqueue("irrc_ts_workqueue");
	if (!irrc->workqueue) {
		ERR_MSG("Unable to create workqueue\n");
		goto err_4;
	}

	INIT_DELAYED_WORK(&irrc->gpio_off_work, android_irrc_disable_pwm);

	irrc->dev.name = "irrc";
	pdev->dev.init_name = irrc->dev.name;
	PROBE_MSG("dev->init_name : %s, dev->kobj : %s\n", pdev->dev.init_name, pdev->dev.kobj.name);

	irrc->gp_clk = clk_get(&pdev->dev, irrc->clk_name);
	clk_set_rate(irrc->gp_clk, (unsigned long)irrc->clk_rate);

    // for VREG_L19_2V85 on irrc sensor.
	irrc->vreg = regulator_get(&pdev->dev, "vreg_irrc");
	if (IS_ERR(irrc->vreg)) {
		ERR_MSG("regulator_get failed (%ld)\n", PTR_ERR(irrc->vreg));
		irrc->vreg = NULL;
		goto err_4;
	}

	// for lvs1 on irrc sensor.
	irrc->vreg2 = regulator_get(&pdev->dev, "vreg2_irrc");
	if (IS_ERR(irrc->vreg2)) {
		ERR_MSG("regulator_get failed (%ld)\n", PTR_ERR(irrc->vreg2));
		irrc->vreg2 = NULL;
		//goto err_4;
	}

	irrc_data_ptr = irrc;
	irrc_dev_ptr = pdev;

	platform_set_drvdata(pdev, irrc);

#ifdef CONFIG_DEBUG_FS
	debugfs_wcd9xxx_dent = debugfs_create_dir("sw_irrc", 0);
	if (!IS_ERR(debugfs_wcd9xxx_dent)) {
		debugfs_poke = debugfs_create_file("poke",
				S_IFREG | S_IWUSR | S_IWGRP,
				debugfs_wcd9xxx_dent, (void *) "poke",
				&codec_debug_ops);
		debugfs_timing = debugfs_create_file("timing",
				S_IFREG | S_IRUGO | S_IWUSR,
				debugfs_wcd9xxx_dent, NULL, &irrc_timing_ops);
	}
#endif
	return 0;

err_4:
	misc_deregister(&irrc_misc);
err_3:
	iounmap(virt_bases_v);
	gpio_free(irrc->pwm_gpio);
err_2:
	kfree(irrc);
err_1:
	ERR_MSG("probe error.\n");
	return -ENODEV;
}

static int android_irrc_remove(struct platform_device *pdev)
{
	struct timed_irrc_data *irrc = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	misc_deregister(&irrc_misc);
	irrc_dev_ptr = NULL;
	iounmap(virt_bases_v);
	gpio_free(irrc->pwm_gpio);
	kfree(irrc);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(debugfs_timing);
	debugfs_remove(debugfs_poke);
	debugfs_remove(debugfs_wcd9xxx_dent);
#endif

	return 0;
}

#ifdef CONFIG_PM
static int android_irrc_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int android_irrc_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static void android_irrc_shutdown(struct platform_device *pdev)
{
}

static struct platform_driver android_irrc_driver = {
	.probe = android_irrc_probe,
	.remove = android_irrc_remove,
	.shutdown = android_irrc_shutdown,
#ifdef CONFIG_PM
	.suspend = android_irrc_suspend,
	.resume = android_irrc_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
	.driver = {
		.name = "android-irrc",
#ifdef CONFIG_OF
		.of_match_table = irrc_match_table,
#endif
	},
};

static int __init android_irrc_init(void)
{
	PROBE_MSG("init\n");
	return platform_driver_register(&android_irrc_driver);
}

static void __exit android_irrc_exit(void)
{
	PROBE_MSG("exit\n");
	platform_driver_unregister(&android_irrc_driver);
}

late_initcall_sync(android_irrc_init); /* to let init lately */
module_exit(android_irrc_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("Android IRRC Driver");
MODULE_LICENSE("GPL");
#endif
