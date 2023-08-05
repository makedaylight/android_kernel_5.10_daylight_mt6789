// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Seiko Instruments S-35390A RTC Driver
 *
 * Copyright (c) 2007 Byron Bradley
 */

#define S35390A_I2C_WRITE_MAX_RETRIES	(3)
#define S35390A_I2C_READ_MAX_RETRIES	(3)

#define S35390A_SLAVE_RTC_DETECT_DELAY_JIFFIES	(HZ)
#define S35390A_SLAVE_RTC_DETECT_MAX_COUNT	(5)

#define S35390A_DEBUG
//#define S35390A_VERBOSE_DEBUG

#ifdef S35390A_VERBOSE_DEBUG
#define VERBOSE_DEBUG
#endif

#define RTC_BASE_YEAR 1900

#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/i2c.h>
#include <linux/bitrev.h>
#include <linux/bcd.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ratelimit_types.h>

#ifdef S35390A_DEBUG
#undef dev_dbg
#define dev_dbg(dev, fmt, arg...) _dev_info(dev, fmt, ##arg)
#endif

#ifdef S35390A_VERBOSE_DEBUG
#undef dev_vdbg
#define dev_vdbg(dev, fmt, arg...) _dev_info(dev, fmt, ##arg)
#endif

#define S35390A_CMD_STATUS1	0
#define S35390A_CMD_STATUS2	1
#define S35390A_CMD_TIME1	2
#define S35390A_CMD_TIME2	3
#define S35390A_CMD_INT1_REG1	4
#define S35390A_CMD_INT2_REG1	5

#define S35390A_BYTE_YEAR	0
#define S35390A_BYTE_MONTH	1
#define S35390A_BYTE_DAY	2
#define S35390A_BYTE_WDAY	3
#define S35390A_BYTE_HOURS	4
#define S35390A_BYTE_MINS	5
#define S35390A_BYTE_SECS	6

#define S35390A_ALRM_BYTE_WDAY	0
#define S35390A_ALRM_BYTE_HOURS	1
#define S35390A_ALRM_BYTE_MINS	2

/* flags for STATUS1 */
#define S35390A_FLAG_POC	BIT(0)
#define S35390A_FLAG_BLD	BIT(1)
#define S35390A_FLAG_INT2	BIT(2)
#define S35390A_FLAG_INT1	BIT(3)
#define S35390A_FLAG_24H	BIT(6)
#define S35390A_FLAG_RESET	BIT(7)

/* flag for STATUS2 */
#define S35390A_FLAG_TEST	BIT(0)

/* INT2 pin output mode */
#define S35390A_INT2_MODE_MASK		0x0E
#define S35390A_INT2_MODE_NOINTR	0x00
#define S35390A_INT2_MODE_ALARM		BIT(1) /* INT2AE */
#define S35390A_INT2_MODE_PMIN_EDG	BIT(2) /* INT2ME */
#define S35390A_INT2_MODE_FREQ		BIT(3) /* INT2FE */
#define S35390A_INT2_MODE_PMIN		(BIT(3) | BIT(2)) /* INT2FE | INT2ME */
/* INT1 pin output mode */
#define S35390A_INT1_MODE_MASK		0xF0
#define S35390A_INT1_MODE_FREQ		BIT(7)
#define S35390A_INT1_MODE_PMIN_EDG	BIT(6)
#define S35390A_INT1_MODE_ALARM		BIT(5)
#define S35390A_INT1_MODE_32K       BIT(4)

static const struct i2c_device_id s35390a_id[] = {
	{ "s35390a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s35390a_id);

static const struct of_device_id s35390a_of_match[] = {
	{ .compatible = "s35390a" },
	{ .compatible = "sii,s35390a" },
	{ }
};
MODULE_DEVICE_TABLE(of, s35390a_of_match);

struct s35390a {
	struct device *dev;
	struct i2c_client *client[8];
	struct rtc_device *rtc;
	int twentyfourhour;
	int alarm_reg;
	char alarm_ien;
	bool resume;
#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	struct delayed_work work;
	u32 detect_max_count;
	struct rtc_device *slave_rtcdev;
	struct device *slave_parent;
	struct ratelimit_state slave_set_time_rs;
#endif
};

static int s35390a_set_reg(struct s35390a *s35390a, int reg, char *buf, int len)
{
	struct i2c_client *client = s35390a->client[reg];
	struct i2c_msg msg[1];
	int ret = 0;
	int retry;

	for (retry = S35390A_I2C_WRITE_MAX_RETRIES; retry > 0; retry--) {
		msg[0].addr  = client->addr;
		msg[0].flags = 0;
		msg[0].len   = len;
		msg[0].buf   = buf;

		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret == 1) {
			ret = 0;
			break;
		} else {
			dev_err(&client->dev, "%s(0x%02x): len=%d err %d; retry %d...\n", __func__,
				reg, len, ret, retry);
			if (ret >= 0)
				ret = -EIO;
			if (retry > 1)
				msleep(20); /* wait for a moment */
		}
	}

	return ret;
}

static int s35390a_get_reg(struct s35390a *s35390a, int reg, char *buf, int len)
{
	struct i2c_client *client = s35390a->client[reg];
	struct i2c_msg msg[1];
	int ret = 0;
	int retry;

	for (retry = S35390A_I2C_READ_MAX_RETRIES; retry > 0; retry--) {
		msg[0].addr  = client->addr;
		msg[0].flags = I2C_M_RD;
		msg[0].len   = len;
		msg[0].buf   = buf;

		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret == 1) {
			ret = 0;
			break;
		} else {
			dev_err(&client->dev, "%s(0x%02x): len=%d err %d; retry %d...\n", __func__,
				reg, len, ret, retry);
			if (ret >= 0)
				ret = -EIO;
			if (retry > 1)
				msleep(20); /* wait for a moment */
		}
	}

	return ret;
}

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
/* true: allowed */
static bool s35390a_ratelimit(struct ratelimit_state *rs)
{
	bool ret;

	if (!rs->interval)
		return true;

	if (!rs->begin)
		rs->begin = jiffies;

	if (time_is_before_jiffies(rs->begin + rs->interval)) {
		rs->missed  = 0;
		rs->begin   = jiffies;
		rs->printed = 0;
	}
	if (rs->burst && rs->burst > rs->printed) {
		rs->printed++;
		ret = true;
	} else {
		rs->missed++;
		ret = false;
	}

	return ret;
}

static void s35390a_ratelimit_reset(struct ratelimit_state *rs)
{
	rs->begin   = 0;
	rs->printed = 0;
	rs->missed  = 0;
}
#endif

static int s35390a_init(struct s35390a *s35390a)
{
	u8 buf;
	int ret;
	unsigned initcount = 0;

	/*
	 * At least one of POC and BLD are set, so reinitialise chip. Keeping
	 * this information in the hardware to know later that the time isn't
	 * valid is unfortunately not possible because POC and BLD are cleared
	 * on read. So the reset is best done now.
	 *
	 * The 24H bit is kept over reset, so set it already here.
	 */
initialize:
	buf = S35390A_FLAG_RESET | S35390A_FLAG_24H;
	ret = s35390a_set_reg(s35390a, S35390A_CMD_STATUS1, &buf, 1);

	if (ret)
		return ret;

	ret = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, &buf, 1);
	if (ret)
		return ret;

	if (buf & (S35390A_FLAG_POC | S35390A_FLAG_BLD)) {
		dev_dbg(s35390a->dev, "%s: S35390A_CMD_STATUS1=0x%02x\n", __func__, buf);
		/* Try up to five times to reset the chip */
		if (initcount < 5) {
			++initcount;
			goto initialize;
		} else
			return -EIO;
	}

	return 1;
}

/*
 * Returns <0 on error, 0 if rtc is setup fine and 1 if the chip was reset.
 * To keep the information if an irq is pending, pass the value read from
 * STATUS1 to the caller.
 */
static int s35390a_read_status(struct s35390a *s35390a, char *status1)
{
	int ret;

	ret = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, status1, 1);
	if (ret)
		return ret;

	if (*status1 & S35390A_FLAG_POC) {
#if 0 /* The waiting time is long enough here (Preloader->LK2->Kernel->NOW) */
		/*
		 * Do not communicate for 0.5 seconds since the power-on
		 * detection circuit is in operation.
		 */
		msleep(500);
#endif
		return 1;
	} else if (*status1 & S35390A_FLAG_BLD)
		return 1;
	else if (!(*status1 & S35390A_FLAG_24H))
		/* The 24H bit is not kept */
		return 1;
	/*
	 * If both POC and BLD are unset everything is fine.
	 */
	return 0;
}

#if 0
static int s35390a_disable_test_mode(struct s35390a *s35390a)
{
	char buf = 0;

	if (s35390a_get_reg(s35390a, S35390A_CMD_STATUS2, &buf, 1))
		return -EIO;

	if (!(buf & S35390A_FLAG_TEST))
		return 0;

	buf &= ~S35390A_FLAG_TEST;
	return s35390a_set_reg(s35390a, S35390A_CMD_STATUS2, &buf, 1);
}
#endif

static char s35390a_hr2reg(struct s35390a *s35390a, int hour)
{
	if (s35390a->twentyfourhour)
		return bin2bcd(hour);

	if (hour < 12)
		return bin2bcd(hour);

	return 0x40 | bin2bcd(hour - 12);
}

static int s35390a_reg2hr(struct s35390a *s35390a, char reg)
{
	unsigned hour;

	if (s35390a->twentyfourhour)
		return bcd2bin(reg & 0x3f);

	hour = bcd2bin(reg & 0x3f);
	if (reg & 0x40)
		hour += 12;

	return hour;
}

static int s35390a_alarm_irq_enable(struct s35390a *s35390a, unsigned enabled)
{
	char sts = 0;
	int err;

	if (enabled)
		sts |= s35390a->alarm_ien;

	/* set interupt mode */
	err = s35390a_set_reg(s35390a, S35390A_CMD_STATUS2, &sts, 1);
	if (err)
		dev_err(s35390a->dev, "setup alarm irq err: %d\n", err);
	else
		dev_vdbg(s35390a->dev, "alarm %s\n", enabled ? "enabled" : "disabled");

	return err;
}

static irqreturn_t s35390a_rtc_irq_handler(int irq, void *client)
{
	struct s35390a *s35390a = i2c_get_clientdata(client);
	char sts = 0;
	int err;
	u32 events = RTC_IRQF;

	dev_notice(s35390a->dev, "rtc irq\n");

	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS2, &sts, 1);
	if (err)
		return err;

	dev_vdbg(s35390a->dev, "%s: S35390A_CMD_STATUS2=0x%02x\n", __func__, sts);

	if (sts & s35390a->alarm_ien) {
		s35390a_alarm_irq_enable(s35390a, 0);
		events |= RTC_AF;
	}

	rtc_update_irq(s35390a->rtc, 1, events);

	/* clear interrupt */
	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, &sts, 1);
	if (err)
		return err;

	dev_vdbg(s35390a->dev, "%s: S35390A_CMD_STATUS1=0x%02x\n", __func__, sts);

	return IRQ_HANDLED;
}

static int s35390a_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	int i, err;
	char buf[7];

	buf[S35390A_BYTE_YEAR] = bin2bcd(tm->tm_year - 100);
	buf[S35390A_BYTE_MONTH] = bin2bcd(tm->tm_mon + 1);
	buf[S35390A_BYTE_DAY] = bin2bcd(tm->tm_mday);
	buf[S35390A_BYTE_WDAY] = bin2bcd(tm->tm_wday);
	buf[S35390A_BYTE_HOURS] = s35390a_hr2reg(s35390a, tm->tm_hour);
	buf[S35390A_BYTE_MINS] = bin2bcd(tm->tm_min);
	buf[S35390A_BYTE_SECS] = bin2bcd(tm->tm_sec);

	/* This chip expects the bits of each byte to be in reverse order */
	for (i = 0; i < 7; ++i)
		buf[i] = bitrev8(buf[i]);

	err = s35390a_set_reg(s35390a, S35390A_CMD_TIME1, buf, sizeof(buf));

	if (!err)
		dev_notice(s35390a->dev, "set time: %04d/%02d/%02d (%d) %02d:%02d:%02d\n",
			tm->tm_year + RTC_BASE_YEAR, tm->tm_mon + 1, tm->tm_mday,
			tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	if (s35390a->slave_rtcdev) {
		struct rtc_time time = *tm;
		int ret;

		/* always update time */
		ret =  s35390a->slave_rtcdev->ops->set_time(s35390a->slave_parent, &time);
		if (ret)
			dev_err(s35390a->dev, "set slave-rtc time err %d\n", ret);
	}
#endif

	return err;
}

static int s35390a_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	char buf[7];
	int i, err;

	err = s35390a_get_reg(s35390a, S35390A_CMD_TIME1, buf, sizeof(buf));
	if (err)
		return err;

	/* This chip returns the bits of each byte in reverse order */
	for (i = 0; i < 7; ++i)
		buf[i] = bitrev8(buf[i]);

	tm->tm_sec = bcd2bin(buf[S35390A_BYTE_SECS]);
	tm->tm_min = bcd2bin(buf[S35390A_BYTE_MINS]);
	tm->tm_hour = s35390a_reg2hr(s35390a, buf[S35390A_BYTE_HOURS]);
	tm->tm_wday = bcd2bin(buf[S35390A_BYTE_WDAY]);
	tm->tm_mday = bcd2bin(buf[S35390A_BYTE_DAY]);
	tm->tm_mon = bcd2bin(buf[S35390A_BYTE_MONTH]) - 1;
	tm->tm_year = bcd2bin(buf[S35390A_BYTE_YEAR]) + 100;

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	if (s35390a->slave_rtcdev) {
		struct rtc_time time = *tm;

		/* prevent the burst read */
		if (s35390a_ratelimit(&s35390a->slave_set_time_rs)) {
			err = s35390a->slave_rtcdev->ops->set_time(s35390a->slave_parent, &time);
			if (err)
				dev_err(s35390a->dev, "set slave-rtc time err %d\n", err);
		}
	}
#endif

	if (s35390a->resume) {
		s35390a->resume = false;
		dev_notice(s35390a->dev, "get time: [resume] %04d/%02d/%02d (%d) %02d:%02d:%02d\n",
			tm->tm_year + RTC_BASE_YEAR, tm->tm_mon + 1, tm->tm_mday,
			tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	} else {
		dev_dbg(&client->dev, "get time: %04d/%02d/%02d (%d) %02d:%02d:%02d\n",
			tm->tm_year + RTC_BASE_YEAR, tm->tm_mon + 1, tm->tm_mday,
			tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	}

	return 0;
}

static int s35390a_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	char buf[3], sts = 0;
	int err, i;

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	if (s35390a->slave_rtcdev) {
		struct rtc_wkalrm alarm = *alm;

		/* always set alarm */
		err = s35390a->slave_rtcdev->ops->set_alarm(s35390a->slave_parent, &alarm);
		if (err)
			dev_err(s35390a->dev, "set slave-rtc alarm err %d\n", err);
	}
#endif

#if 0
	if (alm->time.tm_sec != 0)
		dev_warn(&client->dev, "Alarms are only supported on a per minute basis!\n");
#endif

	/* disable interrupt (which deasserts the irq line) */
	err = s35390a_alarm_irq_enable(s35390a, 0);
	if (err)
		return err;

	/* clear pending interrupt (in STATUS1 only), if any */
	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS1, &sts, 1);
	if (err)
		return err;

	if (!alm->enabled) {
		dev_notice(s35390a->dev, "set alarm: [DIS] (%04d/%02d/%02d (%d) %02d:%02d:%02d)\n",
			alm->time.tm_year + RTC_BASE_YEAR, alm->time.tm_mon + 1, alm->time.tm_mday,
			alm->time.tm_wday, alm->time.tm_hour, alm->time.tm_min, alm->time.tm_sec);
		return 0;
	} else {
		/**
		 * Add one more minute (60secs) because it only supports the per-minute alarm.
		 * For stability, reserve extra 4 seconds to make sure the Alarm will not be missed.
		 *   ex. NOW      2023-03-16 (4) 11:27:57
		 *       ALARM    2023-03-16 (4) 11:27:59
		 *       S35390A + 60secs => (4) 11:28:59 => (per-minute alarm) (4) 11:28:00
		 *       There is a possibility that this ALARM will be missed.
		 *
		 *       S35390A + 64secs => (4) 11:29:03 => (per-minute alarm) (4) 11:29:00
		 */
		time64_t target = rtc_tm_to_time64(&alm->time);
		target += 64; /* secs */
		rtc_time64_to_tm(target, &alm->time);
	}

	if (alm->time.tm_wday != -1)
		buf[S35390A_ALRM_BYTE_WDAY] = bin2bcd(alm->time.tm_wday) | 0x80;
	else
		buf[S35390A_ALRM_BYTE_WDAY] = 0;

	buf[S35390A_ALRM_BYTE_HOURS] = s35390a_hr2reg(s35390a,
			alm->time.tm_hour) | 0x80;
	buf[S35390A_ALRM_BYTE_MINS] = bin2bcd(alm->time.tm_min) | 0x80;

	if (alm->time.tm_hour >= 12)
		buf[S35390A_ALRM_BYTE_HOURS] |= 0x40;

	for (i = 0; i < 3; ++i)
		buf[i] = bitrev8(buf[i]);

	/* Must enable alarm irq first then setup the alarm time. Otherwise, the irq won't work! */
	err = s35390a_alarm_irq_enable(s35390a, alm->enabled);
	if (err)
		return err;

	err = s35390a_set_reg(s35390a, s35390a->alarm_reg, buf,
								sizeof(buf));
	if (!err)
		dev_notice(s35390a->dev, "set alarm: [EN] (%04d/%02d/%02d) (%d) %02d:%02d:00\n",
			alm->time.tm_year + RTC_BASE_YEAR, alm->time.tm_mon + 1, alm->time.tm_mday,
			alm->time.tm_wday, alm->time.tm_hour, alm->time.tm_min);

	return err;
}

static int s35390a_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	char buf[3], sts;
	int i, err;

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	if (s35390a->slave_rtcdev) {
		/* always get slave alarm */
		err = s35390a->slave_rtcdev->ops->read_alarm(s35390a->slave_parent, alm);
		if (err)
			dev_err(s35390a->dev, "get slave-rtc alarm err %d\n", err);
		else
			return 0;
	}
#endif

	err = s35390a_get_reg(s35390a, S35390A_CMD_STATUS2, &sts, 1);
	if (err)
		return err;

	if (!(sts & s35390a->alarm_ien)) {
		/*
		 * When the alarm isn't enabled, the register to configure
		 * the alarm time isn't accessible.
		 */
		alm->enabled = 0;
		alm->pending = 0;
		rtc_time64_to_tm(0, &alm->time);
		dev_dbg(&client->dev, "get alarm: disabled\n");
		return 0;
	} else {
		alm->enabled = 1;
	}

	err = s35390a_get_reg(s35390a, s35390a->alarm_reg, buf, sizeof(buf));
	if (err)
		return err;

#if 0
	/* This chip returns the bits of each byte in reverse order */
	for (i = 0; i < 3; ++i)
		buf[i] = bitrev8(buf[i]);

	/*
	 * B0 of the three matching registers is an enable flag. Iff it is set
	 * the configured value is used for matching.
	 */
	if (buf[S35390A_ALRM_BYTE_WDAY] & 0x80)
		alm->time.tm_wday =
			bcd2bin(buf[S35390A_ALRM_BYTE_WDAY] & ~0x80);

	if (buf[S35390A_ALRM_BYTE_HOURS] & 0x80)
		alm->time.tm_hour =
			s35390a_reg2hr(s35390a,
				       buf[S35390A_ALRM_BYTE_HOURS] & ~0x80);

	if (buf[S35390A_ALRM_BYTE_MINS] & 0x80)
		alm->time.tm_min = bcd2bin(buf[S35390A_ALRM_BYTE_MINS] & ~0x80);
#else
	/* This chip returns the bits of each byte in reverse order */
	for (i = 0; i < 3; ++i) {
		buf[i] = bitrev8(buf[i]);
		buf[i] &= ~0x80;
	}

	alm->time.tm_wday = bcd2bin(buf[S35390A_ALRM_BYTE_WDAY]);
	alm->time.tm_hour = s35390a_reg2hr(s35390a,
									   buf[S35390A_ALRM_BYTE_HOURS]);
	alm->time.tm_min = bcd2bin(buf[S35390A_ALRM_BYTE_MINS]);
#endif

	/* alarm triggers always at s=0 */
	alm->time.tm_sec = 0;

	dev_notice(&client->dev, "get alarm: [EN] (%d) %02d:%02d:00\n",
		alm->time.tm_wday, alm->time.tm_hour, alm->time.tm_min);

	return 0;
}

static int s35390a_rtc_alarm_irq_enable(struct device *dev, unsigned enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s35390a *s35390a = i2c_get_clientdata(client);
	int err;

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	if (s35390a->slave_rtcdev && s35390a->slave_rtcdev->ops->alarm_irq_enable) {
		/* always set slave alarm irq enable/disable */
		err = s35390a->slave_rtcdev->ops->alarm_irq_enable(s35390a->slave_parent, enabled);
		if (err)
			dev_err(s35390a->dev, "set slave-rtc alarm irq enable(%d) err %d\n", enabled, err);
	}
#endif

	err = s35390a_alarm_irq_enable(s35390a, enabled);
	if (!err)
		dev_notice(s35390a->dev, "alarm irq %s\n", enabled ? "enabled" : "disabled");

	return err;
}

static const struct rtc_class_ops s35390a_rtc_ops = {
	.read_time	= s35390a_rtc_read_time,
	.set_time	= s35390a_rtc_set_time,
	.set_alarm	= s35390a_rtc_set_alarm,
	.read_alarm	= s35390a_rtc_read_alarm,
	.alarm_irq_enable	= s35390a_rtc_alarm_irq_enable,
};

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
static void s35390a_detect_slave_rtc_work(struct work_struct *work)
{
	struct s35390a *s35390a = container_of(to_delayed_work(work),
			struct s35390a, work);

	if (s35390a->detect_max_count--) {
		struct rtc_time tm;
		struct rtc_device *rtcdev;

		rtcdev = rtc_class_open(CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC_DEVICE);
		if (!rtcdev)
			goto slave_retry;

		if (!rtcdev->dev.parent || !rtcdev->ops || !rtcdev->ops->set_time ||
			!rtcdev->ops->set_alarm || !rtcdev->ops->read_alarm) {
			rtc_class_close(rtcdev);
			dev_err(s35390a->dev, "invalid slave '%s' device\n",
				CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC_DEVICE);
			goto slave_exit;
		}

		/* found */
		s35390a->slave_rtcdev = rtcdev;
		s35390a->slave_parent = rtcdev->dev.parent;
		dev_notice(s35390a->dev, "found slave rtc: %s (%s)\n",
			dev_name(&rtcdev->dev), dev_name(s35390a->slave_parent));

		/* sync RTC time */
		(void)rtc_read_time(s35390a->rtc, &tm);
	}

slave_exit:
	return;

slave_retry:
	if (s35390a->detect_max_count) {
		dev_warn(s35390a->dev, "slave '%s' device not found: retry %u\n",
			CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC_DEVICE,
			s35390a->detect_max_count);
		queue_delayed_work(system_power_efficient_wq, &s35390a->work,
			S35390A_SLAVE_RTC_DETECT_DELAY_JIFFIES);
	} else
		dev_err(s35390a->dev, "slave '%s' device not found\n",
			CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC_DEVICE);
}
#endif


static int s35390a_setup_defaults(struct s35390a *s35390a)
{
	struct device *dev = s35390a->dev;
	int err;
	u32 default_time[6];
	struct rtc_time tm;
	time64_t target;

	err = of_property_read_u32_array(dev->of_node, "default-time",
			default_time, ARRAY_SIZE(default_time));
	if (err) {
		if (err != -EINVAL)
			dev_err(dev, "get 'default-time' err %d\n", err);
		else {
			dev_dbg(dev, "'default-time' not found\n");
			err = 0;
		}
		return err;
	}

	tm.tm_year = default_time[0] - RTC_BASE_YEAR;
	tm.tm_mon  = default_time[1] - 1;
	tm.tm_mday = default_time[2];
	tm.tm_hour = default_time[3];
	tm.tm_min  = default_time[4];
	tm.tm_sec  = default_time[5];

	target = rtc_tm_to_time64(&tm);
	rtc_time64_to_tm(target, &tm);

	err = s35390a_rtc_set_time(dev, &tm);
	if (err)
		dev_err(dev, "setup 'default-time' err %d\n", err);

	return err;
}

static int s35390a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err, err_read;
	unsigned int i;
	struct s35390a *s35390a;
	char status1;
	struct device *dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	s35390a = devm_kzalloc(dev, sizeof(struct s35390a), GFP_KERNEL);
	if (!s35390a)
		return -ENOMEM;

	s35390a->dev = dev;
	s35390a->client[0] = client;
	i2c_set_clientdata(client, s35390a);

	/* This chip uses multiple addresses, use dummy devices for them */
	for (i = 1; i < 8; ++i) {
		s35390a->client[i] = devm_i2c_new_dummy_device(dev,
							       client->adapter,
							       client->addr + i);
		if (IS_ERR(s35390a->client[i])) {
			dev_err(dev, "Address %02x unavailable\n",
				client->addr + i);
			return PTR_ERR(s35390a->client[i]);
		}
	}

	s35390a->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(s35390a->rtc))
		return PTR_ERR(s35390a->rtc);

	status1 = 0;
	err_read = s35390a_read_status(s35390a, &status1);
	if (err_read < 0) {
		dev_err(dev, "resetting chip err %d\n", err_read);
		return err_read;
	}
	dev_vdbg(dev, "%s: S35390A_CMD_STATUS1=0x%02x; err_read=%d\n", __func__, status1, err_read);

	if (of_property_read_bool(dev->of_node, "use-int2")) {
		dev_vdbg(dev, "%s: use INT2\n", __func__);
		s35390a->alarm_reg = S35390A_CMD_INT2_REG1;
		s35390a->alarm_ien = S35390A_INT2_MODE_ALARM;
	} else {
		dev_vdbg(dev, "%s: use INT1\n", __func__);
		s35390a->alarm_reg = S35390A_CMD_INT1_REG1;
		s35390a->alarm_ien = S35390A_INT1_MODE_ALARM;
	}

	if (err_read == 1) {
		/* S35390A_FLAG_POC, or S35390A_FLAG_BLD, or not S35390A_FLAG_24H */
		s35390a_init(s35390a);

		err = s35390a_read_status(s35390a, &status1);
		if (err) {
			dev_err(dev, "resetting chip err %d\n", err);
			return err;
		}
		dev_vdbg(dev, "%s: new S35390A_CMD_STATUS1=0x%02x\n", __func__, status1);
	}

#if 0
	/* S35390A_CMD_STATUS2 will be 0 in s35390a_alarm_irq_enable(0) */
	err = s35390a_disable_test_mode(s35390a);
	if (err) {
		dev_err(dev, "disabling test mode err %d\n", err);
		return err;
	}
#endif

	if (status1 & S35390A_FLAG_24H)
		s35390a->twentyfourhour = 1;
	else
		s35390a->twentyfourhour = 0;

	/* disable interrupt (which deasserts the irq line) */
	err = s35390a_alarm_irq_enable(s35390a, 0);
	if (err) {
		dev_err(dev, "error clear ststus2: %d\n", err);
		return err;
	}

	if (err_read == 1)
		/* S35390A_FLAG_POC, or S35390A_FLAG_BLD, or not S35390A_FLAG_24H */
		s35390a_setup_defaults(s35390a);

	if (client->irq >= 0) {
		unsigned long irqflags = 0;

		if (!irq_get_trigger_type(client->irq))
			irqflags = IRQF_TRIGGER_LOW;

		dev_vdbg(dev, "irq (%d) flags: 0x%x\n", client->irq, irqflags);

		err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				s35390a_rtc_irq_handler, irqflags | IRQF_ONESHOT,
				"rtc-s35390a", client);
		if (err) {
			dev_err(dev, "request irq (%d) err\n", client->irq, err);
			return err;
		}
	}

	device_set_wakeup_capable(dev, 1);

	s35390a->rtc->ops = &s35390a_rtc_ops;
	s35390a->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	s35390a->rtc->range_max = RTC_TIMESTAMP_END_2099;

	/* supports per-minute alarms only, therefore set uie_unsupported */
	s35390a->rtc->uie_unsupported = 1;

	err = rtc_register_device(s35390a->rtc);
	if (err) {
		dev_err(dev, "register rtc err %d\n", err);
		return err;
	}

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	ratelimit_state_init(&s35390a->slave_set_time_rs, 5 * HZ, 1); /* 5secs */
	s35390a->detect_max_count = S35390A_SLAVE_RTC_DETECT_MAX_COUNT;
	INIT_DELAYED_WORK(&s35390a->work, s35390a_detect_slave_rtc_work);
	queue_delayed_work(system_power_efficient_wq, &s35390a->work,
		S35390A_SLAVE_RTC_DETECT_DELAY_JIFFIES);
#endif

	dev_info(dev, "probed (wakeup %d)\n", device_may_wakeup(dev) ? 1 : 0);

	return 0;
}

static int s35390a_remove(struct i2c_client *client)
{
	struct s35390a *s35390a = i2c_get_clientdata(client);

	s35390a_alarm_irq_enable(s35390a, 0);

#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	if (s35390a->slave_rtcdev) {
		struct rtc_device *rtcdev = s35390a->slave_rtcdev;

		s35390a->slave_rtcdev = NULL;
		rtc_class_close(rtcdev);
	}
#endif

	return 0;
}

static void s35390a_shutdown(struct i2c_client *client)
{
	struct s35390a *s35390a = i2c_get_clientdata(client);

	s35390a_alarm_irq_enable(s35390a, 0);
}

#ifdef CONFIG_PM_SLEEP
static int s35390a_suspend_noirq(struct device *dev)
{
	struct s35390a *s35390a = dev_get_drvdata(dev);

	s35390a->resume = true;
#ifdef CONFIG_RTC_DRV_S35390A_SUPPORT_SLAVE_RTC
	s35390a_ratelimit_reset(&s35390a->slave_set_time_rs);
#endif

	dev_vdbg(dev, "suspend_noirq\n");

	return 0;
}

#if 0
static int s35390a_resume_noirq(struct device *dev)
{
	dev_vdbg(dev, "resume_noirq\n");

	return 0;
}
#endif

static const struct dev_pm_ops s35390a_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(s35390a_suspend_noirq, NULL/*s35390a_resume_noirq*/)
};
#define S35390A_PM_OPS (&s35390a_pm_ops)
#else
#define S35390A_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static struct i2c_driver s35390a_driver = {
	.driver		= {
		.name	= "rtc-s35390a",
		.of_match_table = of_match_ptr(s35390a_of_match),
		.pm     = S35390A_PM_OPS,
	},
	.probe		= s35390a_probe,
	.remove		= s35390a_remove,
	.shutdown	= s35390a_shutdown,
	.id_table	= s35390a_id,
};

module_i2c_driver(s35390a_driver);

MODULE_AUTHOR("Byron Bradley <byron.bbradley@gmail.com>");
MODULE_DESCRIPTION("S35390A RTC driver");
MODULE_LICENSE("GPL");
