// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Mediatek Inc.
 */

#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/linear_range.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_MTK_CHARGER)
#include <linux/phy/phy.h>
#include "charger_class.h"
#include "mtk_charger.h"
#endif /* CONFIG_MTK_CHARGER */

#define RT9471_DRV_VERSION	"1.0.18_RTK"

/* Device ID */
#define RT9470_DEVID		0x09
#define RT9470D_DEVID		0x0A
#define RT9471_DEVID		0x0D
#define RT9471D_DEVID		0x0E

/* IRQ definitions */
#define RT9471_IRQ_BC12_DONE	0
#define RT9471_IRQ_DETACH	1
#define RT9471_IRQ_RECHG	2
#define RT9471_IRQ_CHG_DONE	3
#define RT9471_IRQ_BG_CHG	4
#define RT9471_IRQ_IE0C		5
#define RT9471_IRQ_CHG_RDY	6
#define RT9471_IRQ_VBUS_GD	7
#define RT9471_IRQ_CHG_BATOV	9
#define RT9471_IRQ_CHG_SYSOV	10
#define RT9471_IRQ_CHG_TOUT	11
#define RT9471_IRQ_CHG_BUSUV	12
#define RT9471_IRQ_CHG_THREG	13
#define RT9471_IRQ_CHG_AICR	14
#define RT9471_IRQ_CHG_MIVR	15
#define RT9471_IRQ_SYS_SHORT	16
#define RT9471_IRQ_SYS_MIN	17
#define RT9471_IRQ_AICC_DONE	18
#define RT9471_IRQ_PE_DONE	19
#define RT9471_IRQ_JEITA_COLD	20
#define RT9471_IRQ_JEITA_COOL	21
#define RT9471_IRQ_JEITA_WARM	22
#define RT9471_IRQ_JEITA_HOT	23
#define RT9471_IRQ_OTG_FAULT	24
#define RT9471_IRQ_OTG_LBP	25
#define RT9471_IRQ_OTG_CC	26
#define RT9471_IRQ_WDT		29
#define RT9471_IRQ_VAC_OV	30
#define RT9471_IRQ_OTP		31

#define RT9471_IRQIDX_MAX	4

/* Register definition */
#define RT9471_REG_OTGCFG	0x00
#define RT9471_REG_TOP		0x01
#define RT9471_REG_FUNCTION	0x02
#define RT9471_REG_IBUS		0x03
#define RT9471_REG_VBUS		0x04
#define RT9471_REG_PRECHG	0x05
#define RT9471_REG_REGU		0x06
#define RT9471_REG_VCHG		0x07
#define RT9471_REG_ICHG		0x08

#define RT9471_REG_STAT0	0x10
#define RT9471_REG_STAT1	0x11
#define RT9471_REG_STAT2	0x12
#define RT9471_REG_STAT3	0x13

#define RT9471_REG_EOC		0x0A
#define RT9471_REG_INFO		0x0B
#define RT9471_REG_JEITA	0x0C
#define RT9471_REG_PUMPEXP	0x0D

#define	RT9471_REG_DPDMDET	0x0E
#define RT9471_REG_STATUS	0x0F
#define RT9471_REG_CHGTIMER	0x09
#define	RT9471_REG_STAT0	0x10
#define RT9471_REG_STAT1	0x11
#define RT9471_REG_STAT2	0x12

#define RT9471_REG_IRQ0		0x20
#define RT9471_REG_MASK0	0x30
#define RT9471_REG_MASK1	0x31
#define RT9471_REG_MASK2	0x32
#define RT9471_REG_MASK3	0x33
#define RT9471_REG_HIDDEN_0	0x40
#define RT9471_REG_HIDDEN_2	0x42
#define RT9471_REG_TOP_HDEN	0x43
#define RT9471_REG_BUCK_HDEN1	0x45
#define RT9471_REG_BUCK_HDEN2	0x46
#define RT9471_REG_BUCK_HDEN3	0x54
#define RT9471_REG_BUCK_HDEN4	0x55
#define RT9471_REG_OTG_HDEN2	0x58
#define RT9471_REG_PASSCODE1	0xA0
//Laker: EMC improvement
#define RT9471_REG_EMC_HDEN1	0x51

#define RT9471_VAC_OVP_MAX_SEL	GENMASK(1, 0)
#define RT9471_OTGCV_MASK	GENMASK(7, 6)
#define RT9471_OTGCC_MASK	BIT(0)
#define RT9471_OTGCV_MINUV	4850000
#define RT9471_OTGCV_STEPUV	150000
#define RT9471_NUM_VOTG		4
#define RT9471_BATFETDIS_MASK	BIT(7)
#define RT9471_HZ_MASK		BIT(5)
#define RT9471_OTG_EN_MASK	BIT(1)
#define RT9471_CV_MAX		4700000
#define RT9471_ICHG_MIN		0
#define RT9471_ICHG_MAX		3150000
#define RT9471_AICR_MIN		0
#define RT9471_DEVID_SHIFT	3
#define RT9471_DEVID_MASK	0x78
#define RT9471_ST_VBUSGD_SHIFT		7
#define RT9471_ST_VBUSGD_MASK		BIT(7)
#define RT9471_ST_CHGRDY_MASK		BIT(6)
#define RT9471_ST_CHGRDY_SHIFT		6

enum rt9471_fields {
	F_PORT_STAT,
	F_IC_STAT,
	F_CHG_EN,
	F_BATFET_DIS_DLY,
	/* SAFETY TIMER*/
	F_CHG_TMR, F_CHG_TMR_EN,
	F_VAC_OVP,
	F_VMIVR, F_VMIVR_TRACK,
	F_AUTO_AICR_EN, F_AICR,
	F_CV, F_VREC,
	F_IPRE,
	F_CC,
	F_HZ,
	F_FORCE_HZ,
	F_EOC_RST,
	F_TE, F_IEOC,
	F_WDT, F_WDT_RST,
	F_OTG_EN,
	F_OTG_CC,
	/* BC12 EN*/
	F_BC12_EN,
	/* AICC FUNC */
	F_AICC_EN,
	/* PE1.0/PE2.0 */
	F_PE20_CODE, F_PE10_INC, F_PE_SEL, F_PE_EN,
	F_REGRST,
	F_JEITA,
	/* STATO*/
	F_BC12_DONE, F_CHG_DONE, F_BG_CHG, F_ST_IEOC, F_CHG_RDY, F_VBUSGD,
	/* STAT1*/
	F_ST_MIVR,
	/* STAT2*/
	F_SYSMIN,
	/* HIDDEN_O */
	F_CHIP_REV,
	/* HIDDEN_2 */
	F_OTG_RES_COMP,
	/* TOP_HDEN */
	F_FORCE_EN_VBUS_SINK,
	F_MAX_FIELDS
};

static struct reg_field rt9471_reg_fields[] = {
	[F_PORT_STAT]	= REG_FIELD(RT9471_REG_STATUS, 4, 7),
	[F_IC_STAT]	= REG_FIELD(RT9471_REG_STATUS, 0, 3),
	[F_CHG_EN]	= REG_FIELD(RT9471_REG_FUNCTION, 0, 0),
	[F_BATFET_DIS_DLY]	= REG_FIELD(RT9471_REG_FUNCTION, 6, 6),
	[F_CHG_TMR]	= REG_FIELD(RT9471_REG_CHGTIMER, 4, 5),
	[F_CHG_TMR_EN]	= REG_FIELD(RT9471_REG_CHGTIMER, 7, 7),
	[F_VAC_OVP]	= REG_FIELD(RT9471_REG_VBUS, 6, 7),
	[F_VMIVR]	= REG_FIELD(RT9471_REG_VBUS, 0, 3),
	[F_VMIVR_TRACK] = REG_FIELD(RT9471_REG_VBUS, 4, 5),
	[F_AUTO_AICR_EN] = REG_FIELD(RT9471_REG_IBUS, 6, 6),
	[F_AICR]	= REG_FIELD(RT9471_REG_IBUS, 0, 5),
	[F_CV]		= REG_FIELD(RT9471_REG_VCHG, 0, 6),
	[F_VREC]	= REG_FIELD(RT9471_REG_VCHG, 7, 7),
	[F_IPRE]	= REG_FIELD(RT9471_REG_PRECHG, 0, 3),
	[F_CC]		= REG_FIELD(RT9471_REG_ICHG, 0, 5),
	[F_HZ]		= REG_FIELD(RT9471_REG_FUNCTION, 5, 5),
	[F_FORCE_HZ]	= REG_FIELD(RT9471_REG_HIDDEN_2, 2, 2),
	[F_EOC_RST]	= REG_FIELD(RT9471_REG_EOC, 0, 0),
	[F_TE]		= REG_FIELD(RT9471_REG_EOC, 1, 1),
	[F_IEOC]	= REG_FIELD(RT9471_REG_EOC, 4, 7),
	[F_WDT]		= REG_FIELD(RT9471_REG_TOP, 0, 1),
	[F_WDT_RST]	= REG_FIELD(RT9471_REG_TOP, 2, 2),
	[F_OTG_EN]	= REG_FIELD(RT9471_REG_FUNCTION, 1, 1),
	[F_OTG_CC]	= REG_FIELD(RT9471_REG_OTGCFG, 0, 0),
	[F_BC12_EN]	= REG_FIELD(RT9471_REG_DPDMDET, 7, 7),
	[F_AICC_EN]	= REG_FIELD(RT9471_REG_IBUS, 7, 7),
	[F_REGRST]	= REG_FIELD(RT9471_REG_INFO, 7, 7),
	[F_PE20_CODE]	= REG_FIELD(RT9471_REG_PUMPEXP, 0, 4),
	[F_PE10_INC]	= REG_FIELD(RT9471_REG_PUMPEXP, 5, 5),
	[F_PE_SEL]	= REG_FIELD(RT9471_REG_PUMPEXP, 6, 6),
	[F_PE_EN]	= REG_FIELD(RT9471_REG_PUMPEXP, 7, 7),
	[F_JEITA]	= REG_FIELD(RT9471_REG_JEITA, 7, 7),
	[F_BC12_DONE]	= REG_FIELD(RT9471_REG_STAT0, 0, 0),
	[F_CHG_DONE]	= REG_FIELD(RT9471_REG_STAT0, 3, 3),
	[F_BG_CHG]	= REG_FIELD(RT9471_REG_STAT0, 4, 4),
	[F_ST_IEOC]	= REG_FIELD(RT9471_REG_STAT0, 5, 5),
	[F_CHG_RDY]	= REG_FIELD(RT9471_REG_STAT0, 6, 6),
	[F_VBUSGD]	= REG_FIELD(RT9471_REG_STAT0, 7, 7),
	[F_ST_MIVR]	= REG_FIELD(RT9471_REG_STAT1, 7, 7),
	[F_SYSMIN]	= REG_FIELD(RT9471_REG_STAT2, 1, 1),
	[F_CHIP_REV]	= REG_FIELD(RT9471_REG_HIDDEN_0, 5, 7),
	[F_OTG_RES_COMP] = REG_FIELD(RT9471_REG_OTG_HDEN2, 4, 5),
	[F_FORCE_EN_VBUS_SINK] = REG_FIELD(RT9471_REG_TOP_HDEN, 4, 4),
};

enum {
	RT9471_ICSTAT_SLEEP = 0,
	RT9471_ICSTAT_VBUSRDY,
	RT9471_ICSTAT_TRICKLECHG,
	RT9471_ICSTAT_PRECHG,
	RT9471_ICSTAT_FASTCHG,
	RT9471_ICSTAT_IEOC,
	RT9471_ICSTAT_BGCHG,
	RT9471_ICSTAT_CHGDONE,
	RT9471_ICSTAT_CHGFAULT,
	RT9471_ICSTAT_OTG = 15,
	RT9471_ICSTAT_MAX,
};

#if IS_ENABLED(CONFIG_MTK_CHARGER)
#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2
enum rt9471_hz_user {
	RT9471_HZU_PP,
	RT9471_HZU_BC12,
	RT9471_HZU_OTG,
	RT9471_HZU_VBUS_GD,
	RT9471_HZU_MAX,
};

static const char * const rt9471_hz_user_names[RT9471_HZU_MAX] = {
	"PP", "BC12", "OTG", "VBUS_GD",
};
enum rt9471_usbsw_state {
	RT9471_USBSW_CHG = 0,
	RT9471_USBSW_USB,
};
#endif /* CONFIG_MTK_CHARGER */

struct rt9471_chip {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *rm_field[F_MAX_FIELDS];
	struct regmap_irq_chip_data *irq_chip_data;
	struct regulator_dev *otg_rdev;
	struct mutex bc12_lock;
	struct mutex hidden_mode_lock;
	int psy_usb_type;
	struct power_supply *psy;
	struct gpio_desc *ceb_gpio;
	unsigned int chip_rev;
	struct delayed_work leave_bat_supply_dwork;
	bool chg_done_once;
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	int psy_chg_type;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	bool is_primary;
	struct mutex hz_lock;
	bool hz_users[RT9471_HZU_MAX];
	const char *chg_name;
	atomic_t bc12_en;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	atomic_t typec_attach;
#endif /* CONFIG_TCPC_CLASS */
	wait_queue_head_t bc12_en_req;
	struct wakeup_source *bc12_en_ws;
	struct task_struct *bc12_en_kthread;
#endif /* CONFIG_MTK_CHARGER */
};

enum rt9471_range {
	RT9471_RANGE_CHG_TMR,
	RT9471_RANGE_ICHG,
	RT9471_RANGE_CV,
	RT9471_RANGE_AICR,
	RT9471_RANGE_MIVR,
	RT9471_RANGE_IEOC,
	RT9471_RANGE_IPRE,
	RT9471_RANGE_PE20_CODE,
	RT9471_RANGE_MAX,
};

static const struct linear_range rt9471_chg_range[RT9471_RANGE_MAX] = {
	[RT9471_RANGE_CHG_TMR] = {0, 0, 0x30, 5},
	[RT9471_RANGE_ICHG] = {0, 0, 0x3f, 50000},
	[RT9471_RANGE_CV] = {3900000, 0, 0x50, 10000},
	[RT9471_RANGE_AICR] = {50000, 1, 0x3e, 50000},
	[RT9471_RANGE_MIVR] = {3900000, 0, 0x0f, 100000},
	[RT9471_RANGE_IEOC] = {50000, 0, 0x0f, 50000},
	[RT9471_RANGE_IPRE] = {50000, 0, 0x0f, 50000},
	[RT9471_RANGE_PE20_CODE] = {5500000, 0, 0x1f, 500000},
};

//Laker: workaround, RT9471 can support 3.9-13.5V, but its MIVR range is only 3.9-5.4V
static unsigned int RT9471_MIVR_uv = 0;


/* Called function by power_supply_set/power_supply_get */
static int rt9471_enable_hidden_mode(struct rt9471_chip *chip, bool en)
{
	int ret;
	u8 rt9471_hidden_mode[] = {0x69, 0x96};

	mutex_lock(&chip->hidden_mode_lock);
	if (en)
		ret = regmap_bulk_write(chip->regmap, RT9471_REG_PASSCODE1, rt9471_hidden_mode,
					ARRAY_SIZE(rt9471_hidden_mode));
	else
		ret = regmap_write(chip->regmap, RT9471_REG_PASSCODE1, 0x00);
	mutex_unlock(&chip->hidden_mode_lock);
	if (ret)
		dev_notice(chip->dev, "%s: en = %d, ret = %d\n", __func__, en, ret);
	return ret;
}

static int __maybe_unused rt9471_is_vbus_gd(struct rt9471_chip *chip, int *vbus_gd)
{
	int ret;
	u32 reg_val = 0;

	ret = regmap_field_read(chip->rm_field[F_VBUSGD], &reg_val);
	if (!ret)
		*vbus_gd = reg_val;

	return ret;
}

static int rt9471_enable_force_hz(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret)
		return ret;
	ret = regmap_field_write(chip->rm_field[F_FORCE_HZ], en);
	if (ret)
		dev_notice(chip->dev, "%s: en = %d, fail\n", __func__, en);
	rt9471_enable_hidden_mode(chip, false);
	return ret;
}

static int rt9471_enable_wdt(struct rt9471_chip *chip, bool en)
{
	return regmap_field_write(chip->rm_field[F_WDT], en ? 0x01 : 0x00);
}
#if !IS_ENABLED(CONFIG_MTK_CHARGER)
static int rt9471_psy_set_online(struct rt9471_chip *chip, bool en)
{
	int ret;

	ret = rt9471_enable_force_hz(chip, !en);
	if (ret < 0) {
		dev_notice(chip->dev, "%s: set force_hz %d fail(%d)\n",
			__func__, en, ret);
		return ret;
	}
	return rt9471_enable_wdt(chip, en);
}
#else
static int rt9471_enable_chg_type_det(struct charger_device *chg_dev, bool en);
static int rt9471_psy_set_online(struct rt9471_chip *chip, bool en)
{
	return rt9471_enable_chg_type_det(chip->chg_dev, en);
}
#endif /* CONFIG_MTK_CHARGER */

void linear_range_get_selector_within(const struct linear_range *r,
				      unsigned int val, unsigned int *selector)
{
	if (r->min > val) {
		*selector = r->min_sel;
		return;
	}

	if (linear_range_get_max_value(r) < val) {
		*selector = r->max_sel;
		return;
	}

	if (r->step == 0)
		*selector = r->min_sel;
	else
		*selector = (val - r->min) / r->step + r->min_sel;
}

static int __rt9471_set_ichg(struct rt9471_chip *chip, u32 ichg)
{
	unsigned int sel = 0;

	linear_range_get_selector_within(&rt9471_chg_range[RT9471_RANGE_ICHG], ichg, &sel);
	return regmap_field_write(chip->rm_field[F_CC], sel);
}

static int __rt9471_set_cv(struct rt9471_chip *chip, u32 cv)
{
	unsigned int sel = 0;

	linear_range_get_selector_within(&rt9471_chg_range[RT9471_RANGE_CV], cv, &sel);
	return regmap_field_write(chip->rm_field[F_CV], sel);
}

static int __rt9471_set_aicr(struct rt9471_chip *chip, u32 aicr)
{
	const struct linear_range *r_aicr = &rt9471_chg_range[RT9471_RANGE_AICR];
	unsigned int sel = 0, linear_max = linear_range_get_max_value(r_aicr);

	if (linear_max < aicr)
		sel = 0x3f;
	else
		linear_range_get_selector_within(r_aicr, aicr, &sel);
	return regmap_field_write(chip->rm_field[F_AICR], sel);
}

static int __rt9471_set_mivr(struct rt9471_chip *chip, u32 mivr)
{
	unsigned int sel = 0;

	linear_range_get_selector_within(&rt9471_chg_range[RT9471_RANGE_MIVR], mivr, &sel);
	return regmap_field_write(chip->rm_field[F_VMIVR], sel);
}

static int __rt9471_set_ieoc(struct rt9471_chip *chip, u32 ieoc)
{
	unsigned int sel = 0;

	linear_range_get_selector_within(&rt9471_chg_range[RT9471_RANGE_IEOC], ieoc, &sel);
	return regmap_field_write(chip->rm_field[F_IEOC], sel);
}

static int __rt9471_run_aicc(struct rt9471_chip *chip, u32 *aicc)
{
	int ret;
	union power_supply_propval val = {.intval = 0};
	u32 aicc_ongoing = 1, chg_mivr = 0;

	if (chip->chip_rev < 4)
		return -EOPNOTSUPP;
	ret = regmap_field_read(chip->rm_field[F_ST_MIVR], &chg_mivr);
	if (!chg_mivr || ret < 0) {
		dev_info(chip->dev, "%s mivr stat not act, ret = %d\n", __func__, ret);
		return ret;
	}
	ret = regmap_field_write(chip->rm_field[F_AICC_EN], 0x01);
	if (ret < 0) {
		dev_notice(chip->dev, "%s aicc en fail(%d)\n", __func__, ret);
		goto out;
	}
	/* HW checks that the status is in mivr loop per 8ms a time. */
	/* 1sec is set for estimating longest time. */
	/* AICC_EN will be cleared automatically once AICC done */
	ret = regmap_field_read_poll_timeout(chip->rm_field[F_AICC_EN],
					     aicc_ongoing, !aicc_ongoing, 8000, 1000000);
	if (!ret) {
		ret = power_supply_get_property(chip->psy,
						POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
		dev_info(chip->dev, "%s aicc = %d\n", __func__, val.intval);
		*aicc = val.intval;
	}
out:
	regmap_field_write(chip->rm_field[F_AICC_EN], 0x00);
	return ret;
}

static int rt9471_enable_bc12(struct rt9471_chip *chip, bool en)
{
	int ret;

	dev_info(chip->dev, "%s: en = %d\n", __func__, en);
	mutex_lock(&chip->bc12_lock);
	if (en) {
		/* if bc12 en bit is 1, toggle it for doing bc12 again */
		ret = regmap_field_write(chip->rm_field[F_BC12_EN], 0x00);
		ret |= regmap_field_write(chip->rm_field[F_BC12_EN], 0x01);
	} else
		ret = regmap_field_write(chip->rm_field[F_BC12_EN], 0x00);
	mutex_unlock(&chip->bc12_lock);
	return ret;
}

static int __rt9471_enable_te(struct rt9471_chip *chip, bool en)
{
	return regmap_field_write(chip->rm_field[F_TE], en);
}

static int rt9471_psy_get_prop_status(struct rt9471_chip *chip, int *ic_stat)
{
	int ret;
	unsigned int val = POWER_SUPPLY_STATUS_UNKNOWN;

	ret = regmap_field_read(chip->rm_field[F_IC_STAT], &val);
	if (ret < 0)
		return -EINVAL;
	*ic_stat = val;
	switch (val) {
	case RT9471_ICSTAT_SLEEP:
	case RT9471_ICSTAT_VBUSRDY:
		val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case RT9471_ICSTAT_TRICKLECHG:
	case RT9471_ICSTAT_PRECHG:
	case RT9471_ICSTAT_FASTCHG:
	case RT9471_ICSTAT_IEOC:
	case RT9471_ICSTAT_BGCHG:
		val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case RT9471_ICSTAT_CHGDONE:
		val = POWER_SUPPLY_STATUS_FULL;
		break;
	case RT9471_ICSTAT_OTG:
		val = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		break;
	}
	return val;
}

static int rt9471_psy_get_current_max(struct rt9471_chip *chip)
{
	int max_current = 0;

	mutex_lock(&chip->bc12_lock);
	if (chip->psy_usb_type == POWER_SUPPLY_USB_TYPE_UNKNOWN ||
	    chip->psy_usb_type == POWER_SUPPLY_USB_TYPE_SDP)
		max_current = 500000;
	else
		max_current = 1500000;
	mutex_unlock(&chip->bc12_lock);
	return max_current;
}

static int rt9471_linear_get_value(struct rt9471_chip *chip,
				   enum rt9471_fields fd, enum rt9471_range rg)
{
	const struct linear_range *r = &rt9471_chg_range[rg];
	unsigned int regval = 0, val = 0;
	int ret;

	ret = regmap_field_read(chip->rm_field[fd], &regval);
	if (ret < 0)
		return ret;
	ret = linear_range_get_value(r, regval, &val);
	return ret < 0 ? ret : val;
}

static int __rt9471_get_aicr(struct rt9471_chip *chip)
{
	unsigned int regval = 0, aicr = 0;
	int ret;

	ret = regmap_field_read(chip->rm_field[F_AICR], &regval);
	if (ret < 0)
		return ret;

	if (regval == 0)
		aicr = 50000;
	else if (regval == 0x3f)
		aicr = 3200000;
	else	/* linear range is from 1 to 0x3e */
		ret = linear_range_get_value(&rt9471_chg_range[RT9471_RANGE_AICR], regval, &aicr);
	return ret < 0 ? ret : aicr;
}

/* power supply init */
static enum power_supply_property rt9471_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL,
};

static enum power_supply_usb_type rt9471_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
};

static int rt9471_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_USB_TYPE:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		return 1;
	default:
		return 0;
	}
}
static int rt9471_charger_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct rt9471_chip *chip = power_supply_get_drvdata(psy);
	int ret;
	u32 aicc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = rt9471_psy_set_online(chip, !!val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = __rt9471_set_ichg(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = __rt9471_set_cv(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval == -1)
			ret = __rt9471_run_aicc(chip, &aicc);
		else
			ret = __rt9471_set_aicr(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = __rt9471_set_mivr(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = __rt9471_set_ieoc(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = rt9471_enable_bc12(chip, true);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = __rt9471_enable_te(chip, val->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	dev_info(chip->dev, "%s: prop = %d, val = %d, ret = %d\n", __func__, psp, val->intval, ret);
	return ret;
}

static int rt9471_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct rt9471_chip *chip = power_supply_get_drvdata(psy);
	int ret = 0, ic_stat = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
#if IS_ENABLED(CONFIG_TCPC_CLASS)
		val->intval  = atomic_read(&chip->typec_attach);
#else
		ret = rt9471_is_vbus_gd(chip, &val->intval);
#endif /* CONFIG_MTK_CHARGER */
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = chip->psy_chg_type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		mutex_lock(&chip->bc12_lock);
		val->intval = chip->psy_usb_type;
		mutex_unlock(&chip->bc12_lock);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rt9471_psy_get_prop_status(chip, &ic_stat);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = rt9471_psy_get_current_max(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = RT9471_CV_MAX;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = rt9471_linear_get_value(chip, F_CC, RT9471_RANGE_ICHG);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = RT9471_ICHG_MAX;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = rt9471_linear_get_value(chip, F_CV, RT9471_RANGE_CV);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = __rt9471_get_aicr(chip);
		if (ret > 0) {
			val->intval = ret;
			ret = 0;
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		val->intval = rt9471_linear_get_value(chip, F_VMIVR, RT9471_RANGE_MIVR);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		val->intval = rt9471_linear_get_value(chip, F_IPRE, RT9471_RANGE_IPRE);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = rt9471_linear_get_value(chip, F_IEOC, RT9471_RANGE_IEOC);
		break;
	default:
		ret = -ENODATA;
		break;
	}
	dev_dbg(chip->dev, "%s: prop = %d, val = %d, ret = %d\n", __func__, psp, val->intval, ret);
	return ret;
}

static struct power_supply_desc rt9471_charger_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= rt9471_charger_properties,
	.num_properties		= ARRAY_SIZE(rt9471_charger_properties),
	.get_property		= rt9471_charger_get_property,
	.set_property		= rt9471_charger_set_property,
	.property_is_writeable	= rt9471_charger_property_is_writeable,
	.usb_types		= rt9471_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(rt9471_charger_usb_types),
};

static int rt9471_init_psy(struct rt9471_chip *chip)
{
	struct power_supply_config charger_cfg = {};
	struct power_supply_desc *desc = &rt9471_charger_desc;

	chip->psy_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	//desc->name = dev_name(chip->dev);
	desc->name = chip->chg_props.alias_name; //Laker: change power_supply name from 7-055 ==> rt9471_chg

	charger_cfg.drv_data = chip;
	charger_cfg.of_node = chip->dev->of_node;
	chip->psy = devm_power_supply_register(chip->dev, desc, &charger_cfg);

	return PTR_ERR_OR_ZERO(chip->psy);
}

/* regmap init */
bool rt9471_accessible_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x0F:
	case 0x10 ... 0x13:
	case 0x20 ... 0x33:
	case 0x40 ... 0xA1:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rt9471_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xA1,
	.writeable_reg = rt9471_accessible_reg,
	.readable_reg = rt9471_accessible_reg,
};

static int rt9471_init_regmap(struct rt9471_chip *chip)
{
	int ret;

	chip->regmap = devm_regmap_init_i2c(chip->client, &rt9471_regmap_config);
	if (IS_ERR(chip->regmap)) {
		dev_notice(chip->dev, "%s fail(%ld)\n", __func__, PTR_ERR(chip->regmap));
		return -EIO;
	}
	ret = devm_regmap_field_bulk_alloc(chip->dev, chip->regmap,
					   chip->rm_field, rt9471_reg_fields,
					   ARRAY_SIZE(rt9471_reg_fields));
	return ret;
}

static int rt9471_reset_register(struct rt9471_chip *chip)
{
	return regmap_field_write(chip->rm_field[F_REGRST], 0x01);
}

/* Otg load transient improvement workaround*/
static int rt9471_otg_trans_wkaround(struct rt9471_chip *chip)
{
	int ret;

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret)
		return ret;
	ret = regmap_field_read(chip->rm_field[F_CHIP_REV], &chip->chip_rev);
	if (ret)
		goto out;
	dev_info(chip->dev, "%s, chip_rev = %d\n", __func__, chip->chip_rev);
	if (chip->chip_rev <= 3)
		ret = regmap_field_write(chip->rm_field[F_OTG_RES_COMP], 0x10);
out:
	rt9471_enable_hidden_mode(chip, false);
	return ret;
}

//Laker: EMC improvement
static int rt9471_emc_wkaround(struct rt9471_chip *chip)
{
	int ret;

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret)
		return ret;

	//REG0x51 = 0x4C – decrease switching slew rate one step
	ret = regmap_write(chip->regmap, RT9471_REG_EMC_HDEN1, 0x4C);
	if (ret)
		dev_notice(chip->dev, "%s: fail()\n", __func__, ret);

	rt9471_enable_hidden_mode(chip, false);

	return ret;
}

static int rt9471_init_setting(struct rt9471_chip *chip)
{
	int ret;

	ret = rt9471_enable_wdt(chip, false);
	if (ret)
		return ret;
	ret = regmap_field_write(chip->rm_field[F_BATFET_DIS_DLY], 0x00);
	if (ret)
		return ret;
	ret = regmap_field_write(chip->rm_field[F_VAC_OVP], RT9471_VAC_OVP_MAX_SEL);
	if (ret)
		return ret;
	ret = regmap_field_write(chip->rm_field[F_AUTO_AICR_EN], 0x00);
	if (ret)
		return ret;
	ret = regmap_field_write(chip->rm_field[F_JEITA], 0x00);
	if (ret)
		return ret;
	ret = regmap_field_write(chip->rm_field[F_VMIVR_TRACK], 0x00);
	if (ret)
		return ret;
	ret = rt9471_otg_trans_wkaround(chip);
	if (ret)
		return ret;

	//Laker: change CV from 4200mV (default) to master's (4350mV):
	//       a. +100mV=4450mV (0x37: 0011 0111);
	//       b.  +40mV=4390mV (0x31: 0011 0001);
	ret = regmap_field_write(chip->rm_field[F_CV], 0x31); //4390mV (0011 0001) for 2/2.5/3A
	if (ret)
		return ret;
	//Laker: change EOC from 200mA (default) to 600mA (1011)
	ret = regmap_field_write(chip->rm_field[F_IEOC], 0x0B);
	if (ret)
		return ret;
	//Laker: change AICR from 2500mA (default) to 3000mA (0011 1100)
	ret = regmap_field_write(chip->rm_field[F_AICR], 0x3C);
	if (ret)
		return ret;

	//Laker: EMC improvement
	ret = rt9471_emc_wkaround(chip);
	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_MTK_CHARGER)
       ret = rt9471_enable_bc12(chip, false);
#endif /* CONFIG_MTK_CHARGER */
	return ret;
}

static void leave_bat_supply_stop_work(void *data)
{
	struct rt9471_chip *chip = data;

	dev_info(chip->dev, "%s\n", __func__);
	cancel_delayed_work_sync(&chip->leave_bat_supply_dwork);
}

#if IS_ENABLED(CONFIG_MTK_CHARGER)
static int __rt9471_enable_hz(struct rt9471_chip *chip, bool en, u32 user);
#endif /* CONFIG_MTK_CHARGER */
/* init irq */
static irqreturn_t rt9471_bc12_result_irq_handler(int irqno, void *priv)
{
	int ret;
	bool inform_psy = true;
	unsigned int reg_val = 0, chg_rdy = 0, vbus_gd = 0, port = 0;
	struct rt9471_chip *chip = priv;
	enum {
		RT9471_PORTSTAT_APPLE_10W = 8,
		RT9471_PORTSTAT_SAMSUNG_10W,
		RT9471_PORTSTAT_APPLE_5W,
		RT9471_PORTSTAT_APPLE_12W,
		RT9471_PORTSTAT_NSDP,
		RT9471_PORTSTAT_SDP,
		RT9471_PORTSTAT_CDP,
		RT9471_PORTSTAT_DCP,
		RT9471_PORTSTAT_MAX,
	};

	mutex_lock(&chip->bc12_lock);
	ret = regmap_read(chip->regmap, RT9471_REG_STAT0, &reg_val);
	dev_info(chip->dev, "%s: stat0 = 0x%02x, ret = %d\n", __func__, reg_val, ret);
	if (ret < 0) {
		mutex_unlock(&chip->bc12_lock);
		return IRQ_NONE;
	}
	chg_rdy = (reg_val & RT9471_ST_CHGRDY_MASK) >> RT9471_ST_CHGRDY_SHIFT;
	vbus_gd = (reg_val & RT9471_ST_VBUSGD_MASK) >> RT9471_ST_VBUSGD_SHIFT;
	/* Workaround waiting for chg_rdy */
	if (chip->chip_rev <= 3 && !chg_rdy && vbus_gd) {
		inform_psy = false;
		goto out;
	}
	if (!vbus_gd) {
		chip->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		goto out;
	} else if (chip->psy_usb_type != POWER_SUPPLY_USB_TYPE_UNKNOWN) {
		/* prevent report bc12 result again */
		inform_psy = false;
		goto out;
	}

	ret = regmap_field_read(chip->rm_field[F_PORT_STAT], &port);
	dev_info(chip->dev, "%s: port_stat = 0x%02x, ret = %d\n", __func__, port, ret);
	ret = regmap_read(chip->regmap, RT9471_REG_STATUS, &reg_val);
	dev_info(chip->dev, "%s: ic_status = 0x%02x, ret = %d\n", __func__, reg_val, ret);

	switch (port) {
	case RT9471_PORTSTAT_SDP:
		chip->psy_chg_type = POWER_SUPPLY_TYPE_USB;
		chip->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case RT9471_PORTSTAT_CDP:
		chip->psy_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		chip->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case RT9471_PORTSTAT_NSDP:
	case RT9471_PORTSTAT_DCP:
	case RT9471_PORTSTAT_SAMSUNG_10W:
	case RT9471_PORTSTAT_APPLE_12W:
	case RT9471_PORTSTAT_APPLE_10W:
	case RT9471_PORTSTAT_APPLE_5W:
		chip->psy_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		chip->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		inform_psy = false;
		break;
	}
out:
	mutex_unlock(&chip->bc12_lock);
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	__rt9471_enable_hz(chip, true, RT9471_HZU_BC12);
#endif /* CONFIG_MTK_CHARGER */
	dev_info(chip->dev, "%s: usb_type = %d\n", __func__, chip->psy_usb_type);
	if (inform_psy)
		power_supply_changed(chip->psy);
	return IRQ_HANDLED;
}

/* Workaround for battery charging unexpected stuck in battery supply mode */
/* gain charging current to leave that loop */
static void rt9471_leave_battery_supply_wkaround_handler(struct work_struct *work)
{
	int ret, i;
	unsigned int chg_done = 0, sys_min = 0;
	struct rt9471_chip *chip = container_of(work, struct rt9471_chip,
						leave_bat_supply_dwork.work);
	struct {
		u8 reg;
		u8 val;
		int delay;
	} rt9471_gain_cur[] = {
		{ RT9471_REG_BUCK_HDEN4, 0x77, 0 },
		{ RT9471_REG_BUCK_HDEN1, 0x2F, 1000 },
		{ RT9471_REG_BUCK_HDEN2, 0xA2, 0 },
		{ RT9471_REG_BUCK_HDEN4, 0x71, 0 },
		{ RT9471_REG_BUCK_HDEN2, 0x22, 0 },
		{ RT9471_REG_BUCK_HDEN1, 0x2D, 0 },
	};

	if (chip->chip_rev > 4)
		return;
	ret = regmap_field_read(chip->rm_field[F_CHG_DONE], &chg_done);
	if (ret)
		return;
	ret = regmap_field_read(chip->rm_field[F_SYSMIN], &sys_min);
	if (ret < 0)
		return;
	dev_info(chip->dev, "%s sys_min = %d, chg_done = %d\n", __func__, sys_min, chg_done);
	if (sys_min)
		rt9471_gain_cur[1].val = 0x2D;
	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0)
		return;

	for (i = 0; i < ARRAY_SIZE(rt9471_gain_cur); i++) {
		ret = regmap_write(chip->regmap,
				   rt9471_gain_cur[i].reg, rt9471_gain_cur[i].val);
		if (ret < 0)
			dev_notice(chip->dev, "%s reg0x%02X = 0x%02X fail(%d)\n", __func__,
					   rt9471_gain_cur[i].reg, rt9471_gain_cur[i].val, ret);
		if (rt9471_gain_cur[i].delay)
			udelay(rt9471_gain_cur[i].delay);
	}

	rt9471_enable_hidden_mode(chip, false);

	if (chg_done && !chip->chg_done_once) {
		chip->chg_done_once = true;
		mod_delayed_work(system_wq, &chip->leave_bat_supply_dwork, msecs_to_jiffies(100));
	}
}

static irqreturn_t rt9471_chg_rdy_irq_handler(int irqno, void *priv)
{
	struct rt9471_chip *chip = priv;

	mod_delayed_work(system_wq, &chip->leave_bat_supply_dwork, msecs_to_jiffies(100));
	return (chip->chip_rev <= 3) ?  rt9471_bc12_result_irq_handler(0, priv) : IRQ_HANDLED;
}

static irqreturn_t rt9471_chg_done_irq_handler(int irqno, void *priv)
{
	struct rt9471_chip *chip = priv;

	cancel_delayed_work_sync(&chip->leave_bat_supply_dwork);
	chip->chg_done_once = false;
	mod_delayed_work(system_wq, &chip->leave_bat_supply_dwork, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static irqreturn_t rt9471_wdt_irq_handler(int irqno, void *priv)
{
	struct rt9471_chip *chip = priv;
	int ret;

	ret = regmap_field_write(chip->rm_field[F_WDT_RST], 0x01);
	return ret ? IRQ_NONE : IRQ_HANDLED;
}

static irqreturn_t rt9471_otg_fault_irq_handler(int irqno, void *priv)
{
	struct rt9471_chip *chip = priv;

	regulator_notifier_call_chain(chip->otg_rdev, REGULATOR_EVENT_FAIL, NULL);
	return IRQ_HANDLED;
}

static irqreturn_t rt9471_detach_irq_handler(int irqno, void *priv);
static irqreturn_t rt9471_vbus_gd_irq_handler(int irqno, void *priv);
static const struct regmap_irq rt9471_regmap_irqs[] = {
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_BC12_DONE, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_DETACH, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_RECHG, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_DONE, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_BG_CHG, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_IE0C, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_RDY, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_VBUS_GD, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_BATOV, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_SYSOV, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_TOUT, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_BUSUV, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_THREG, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_AICR, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_CHG_MIVR, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_SYS_SHORT, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_SYS_MIN, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_AICC_DONE, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_PE_DONE, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_JEITA_COLD, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_JEITA_COOL, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_JEITA_WARM, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_JEITA_HOT, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_OTG_FAULT, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_OTG_LBP, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_OTG_CC, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_WDT, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_VAC_OV, 8),
	REGMAP_IRQ_REG_LINE(RT9471_IRQ_OTP, 8),
};

static const struct regmap_irq_chip rt9471_regmap_irq_chip = {
	.name = "rt9471-irqs",
	.status_base = RT9471_REG_IRQ0,
	.mask_base = RT9471_REG_MASK0,
	.num_regs = RT9471_IRQIDX_MAX,
	.irqs = rt9471_regmap_irqs,
	.num_irqs = ARRAY_SIZE(rt9471_regmap_irqs),
};

static int rt9471_init_irq(struct rt9471_chip *chip)
{
	struct {
		char *name;
		int hw_irq;
		irq_handler_t handler;
	} rt9471_irqs[] = {
		{ "bc12_done", RT9471_IRQ_BC12_DONE, rt9471_bc12_result_irq_handler },
		{ "detach", RT9471_IRQ_DETACH, rt9471_detach_irq_handler },
		{ "vbus_gd", RT9471_IRQ_VBUS_GD, rt9471_vbus_gd_irq_handler },
		{ "chg_done", RT9471_IRQ_CHG_DONE, rt9471_chg_done_irq_handler },
		{ "chg_rdy", RT9471_IRQ_CHG_RDY, rt9471_chg_rdy_irq_handler },
		{ "wdt", RT9471_IRQ_WDT, rt9471_wdt_irq_handler },
		{ "otg_fault", RT9471_IRQ_OTG_FAULT, rt9471_otg_fault_irq_handler },
	};
	int ret, i, num_chg_irqs = ARRAY_SIZE(rt9471_irqs), irqnum = 0;

	dev_info(chip->dev, "%s: irq = %d\n", __func__, chip->client->irq);

	ret = devm_regmap_add_irq_chip(chip->dev, chip->regmap, chip->client->irq,
					IRQF_ONESHOT, 0, &rt9471_regmap_irq_chip,
					&chip->irq_chip_data);
	if (ret) {
		dev_notice(chip->dev, "%s: Failed to add irq chip (%d)\n", __func__, ret);
		return ret;
	}

	for (i = 0; i < num_chg_irqs; i++) {
		irqnum = regmap_irq_get_virq(chip->irq_chip_data, rt9471_irqs[i].hw_irq);
		if (irqnum < 0)
			return -EINVAL;
		ret = devm_request_threaded_irq(chip->dev, irqnum, NULL, rt9471_irqs[i].handler,
						IRQF_ONESHOT | IRQF_TRIGGER_FALLING, rt9471_irqs[i].name, chip);
		if (ret) {
			dev_notice(chip->dev, "%s: request irq %d fail %d\n",
				  __func__, irqnum, ret);
			return ret;
		}
	}
	return ret;
}

/* init regulator */
static const struct regulator_ops rt9471_otg_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.set_current_limit = regulator_set_current_limit_regmap,
	.get_current_limit = regulator_get_current_limit_regmap,
};

static const u32 rt9471_otg_microamp[] = {
	500000, 1200000,
};

static const struct regulator_desc rt9471_otg_rdesc = {
	.of_match = of_match_ptr("otg-vbus"),
	.name = "rt9471-otg-vbus",
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.enable_val = RT9471_OTG_EN_MASK,
	.enable_mask = RT9471_OTG_EN_MASK,
	.enable_reg = RT9471_REG_FUNCTION,
	.min_uV = RT9471_OTGCV_MINUV,
	.uV_step = RT9471_OTGCV_STEPUV,
	.n_voltages = RT9471_NUM_VOTG,
	.vsel_reg = RT9471_REG_OTGCFG,
	.vsel_mask = RT9471_OTGCV_MASK,
	.csel_reg = RT9471_REG_OTGCFG,
	.csel_mask = RT9471_OTGCC_MASK,
	.curr_table = rt9471_otg_microamp,
	.n_current_limits = ARRAY_SIZE(rt9471_otg_microamp),
	.ops = &rt9471_otg_ops,
};

static int rt9471_init_regulator(struct rt9471_chip *chip)
{
	struct regulator_config config = {
		.dev = chip->dev,
		.regmap = chip->regmap,
		.driver_data = chip,
	};
	chip->otg_rdev = devm_regulator_register(chip->dev, &rt9471_otg_rdesc,
						&config);
	return PTR_ERR_OR_ZERO(chip->otg_rdev);
}

/* init regulator end */
static int rt9471_check_devinfo(struct rt9471_chip *chip)
{
	int ret;
	unsigned int dev_id = 0;

	ret = regmap_read(chip->regmap, RT9471_REG_INFO, &dev_id);
	if (ret)
		return ret;
	dev_info(chip->dev, "%s dev_id = 0x%02X\n", __func__, dev_id);

	dev_id = (dev_id & RT9471_DEVID_MASK) >> RT9471_DEVID_SHIFT;
	switch (dev_id) {
	case RT9470_DEVID:
	case RT9470D_DEVID:
	case RT9471_DEVID:
	case RT9471D_DEVID:
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);
	int ret, tmp = 0;
	u8 mask = RT9471_BATFETDIS_MASK | RT9471_HZ_MASK;

	ret = kstrtoint(buf, 10, &tmp);
	if (ret) {
		dev_notice(dev, "%s parsing number fail(%d)\n", __func__, ret);
		return -EINVAL;
	}
	if (tmp != 5526789)
		return -EINVAL;
	ret = rt9471_reset_register(chip);
	if (ret) {
		dev_notice(dev, "%s: reset register fail(%d)\n", __func__, ret);
		return ret;
	}
	ret = regmap_update_bits(chip->regmap, RT9471_REG_FUNCTION, mask, mask);
	if (ret)
		dev_notice(chip->dev, "%s: (%d) enable shipmode fail\n", __func__, ret);
	return ret < 0 ? ret : count;
}
static const DEVICE_ATTR_WO(shipping_mode);

#if IS_ENABLED(CONFIG_MTK_CHARGER)
static int __rt9471_enable_hz(struct rt9471_chip *chip, bool en, u32 user)
{
	int ret = 0, i = 0;

	if (user >= RT9471_HZU_MAX)
		return -EINVAL;

	dev_info(chip->dev, "%s en = %d, user = %s\n",
			    __func__, en, rt9471_hz_user_names[user]);

	mutex_lock(&chip->hz_lock);
	chip->hz_users[user] = en;
	for (i = 0, en = true; i < RT9471_HZU_MAX; i++)
		en &= chip->hz_users[i];
	if (chip->is_primary)
		ret = regmap_field_write(chip->rm_field[F_HZ], en);
	else
		ret = rt9471_enable_force_hz(chip, en);
	mutex_unlock(&chip->hz_lock);

	return ret;
}

static int rt9471_set_usbsw_state(struct rt9471_chip *chip, int usbsw)
{
	struct phy *phy;
	int ret, mode = (usbsw == RT9471_USBSW_CHG) ? PHY_MODE_BC11_SET :
					       PHY_MODE_BC11_CLR;

	dev_info(chip->dev, "usbsw=%d\n", usbsw);
	phy = phy_get(chip->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_notice(chip->dev, "failed to get usb2-phy\n");
		return -ENODEV;
	}
	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, mode);
	if (ret)
		dev_notice(chip->dev, "failed to set phy ext mode\n");
	phy_put(chip->dev, phy);
	return ret;
}

static bool is_usb_rdy(struct device *dev)
{
	bool ready = true;
	struct device_node *node = NULL;

	node = of_parse_phandle(dev->of_node, "usb", 0);
	if (node) {
		ready = !of_property_read_bool(node, "cdp-block");
		dev_info(dev, "usb ready = %d\n", ready);
	} else
		dev_notice(dev, "usb node missing or invalid\n");
	return ready;
}

static int rt9471_bc12_en_kthread(void *data)
{
	int ret = 0, i = 0, en = 0;
	struct rt9471_chip *chip = data;
	const int max_wait_cnt = 200;

wait:
#if 0
	wait_event(chip->bc12_en_req, atomic_read(&chip->bc12_en) >= 0 ||
				      kthread_should_stop());
#else
	wait_event_interruptible(chip->bc12_en_req, atomic_read(&chip->bc12_en) >= 0 ||
				      kthread_should_stop());
#endif
	if (kthread_should_stop()) {
		dev_info(chip->dev, "%s bye\n", __func__);
		return 0;
	}

	en = atomic_xchg(&chip->bc12_en, -1);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	if (en < 0)
		goto wait;

	__pm_stay_awake(chip->bc12_en_ws);

	if (en) {
		/* Workaround for CDP port */
		for (i = 0; i < max_wait_cnt && !is_usb_rdy(chip->dev); i++) {
			dev_dbg(chip->dev, "%s CDP block\n", __func__);
			if (!atomic_read(&chip->bc12_en)) {
				dev_info(chip->dev, "%s plug out\n", __func__);
				goto relax_and_wait;
			}
			msleep(100);
		}
		if (i >= max_wait_cnt)
			dev_notice(chip->dev, "%s CDP timeout\n", __func__);
		else
			dev_info(chip->dev, "%s CDP free\n", __func__);
	}
	rt9471_set_usbsw_state(chip, en ? RT9471_USBSW_CHG : RT9471_USBSW_USB);
	ret = rt9471_enable_bc12(chip, en);
	if (ret < 0)
		dev_notice(chip->dev, "%s en = %d fail(%d)\n",
				      __func__, en, ret);
relax_and_wait:
	__pm_relax(chip->bc12_en_ws);
	goto wait;

	return 0;
}

static irqreturn_t rt9471_detach_irq_handler(int irqno, void *priv)
{
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	struct rt9471_chip *chip = priv;

	dev_info(chip->dev, "%s\n", __func__);
	__rt9471_enable_hz(chip, true, RT9471_HZU_BC12);
	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->bc12_en, 0);
	chip->psy_chg_type = POWER_SUPPLY_TYPE_USB;
	chip->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	mutex_unlock(&chip->bc12_lock);
	wake_up(&chip->bc12_en_req);
	power_supply_changed(chip->psy);
#endif /* CONFIG_TCPC_CLASS */

	return IRQ_HANDLED;
}

static irqreturn_t rt9471_vbus_gd_irq_handler(int irqno, void *priv)
{
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	struct rt9471_chip *chip = priv;

	dev_info(chip->dev, "%s\n", __func__);
	__rt9471_enable_hz(chip, false, RT9471_HZU_BC12);
	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->bc12_en, 1);
	mutex_unlock(&chip->bc12_lock);
	wake_up(&chip->bc12_en_req);
#endif /* CONFIG_TCPC_CLASS */
	return IRQ_HANDLED;
}

static int __rt9471_enable_chg(struct rt9471_chip *chip, bool en)
{
	int ret = 0;

	dev_info(chip->dev, "%s en = %d, chip_rev = %d\n",
			    __func__, en, chip->chip_rev);

	ret = regmap_field_write(chip->rm_field[F_CHG_EN], en);
	if (ret < 0)
		dev_notice(chip->dev, "%s: set chg_en %d fail (%d)\n",
							__func__, en, ret);

	if (ret >= 0 && chip->chip_rev <= 4)
		mod_delayed_work(system_wq, &chip->leave_bat_supply_dwork,
				 msecs_to_jiffies(100));

	return ret;
}

static int rt9471_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	ret = rt9471_enable_wdt(chip, true);
	if (ret < 0)
		dev_notice(chip->dev, "%s set wdt fail(%d)\n", __func__, ret);

	return ret;
}

static int rt9471_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	ret = __rt9471_enable_chg(chip, true);
	if (ret < 0) {
		dev_notice(chip->dev, "%s en chg fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = rt9471_enable_wdt(chip, false);
	if (ret < 0)
		dev_notice(chip->dev, "%s set wdt fail(%d)\n", __func__, ret);

	return ret;
}

static int rt9471_enable_charging(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	ret = __rt9471_enable_chg(chip, en);
	if (ret < 0)
		dev_notice(chip->dev, "%s en chg fail(%d)\n", __func__, ret);

	return ret;
}

static int rt9471_is_charging_enabled(struct charger_device *chg_dev, bool *en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u32 regval = 0;

	ret = regmap_field_read(chip->rm_field[F_CHG_EN], &regval);
	if (ret < 0) {
		dev_notice(chip->dev, "%s:(%d) read CHG_EN fail\n", __func__, ret);
		return ret;
	}
	*en = regval;
	return ret;
}

static int rt9471_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};

	ret = power_supply_get_property(chip->psy,
					POWER_SUPPLY_PROP_STATUS, &val);
	if (ret < 0)
		return ret;
	*done = (val.intval == POWER_SUPPLY_STATUS_FULL);

	return ret;
}

static int rt9471_get_mivr(struct charger_device *chg_dev, u32 *uV)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	int ret = 0;

	ret = power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT, &val);
	if (ret < 0)
		return ret;
	*uV = val.intval;

	if (RT9471_MIVR_uv >= 3900000 && RT9471_MIVR_uv <= 13500000)
		*uV = RT9471_MIVR_uv;

	return ret;
}

static int rt9471_set_mivr(struct charger_device *chg_dev, u32 uV)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = uV};
	int ret = 0;

	ret = power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT, &val);

	if (ret < 0)
		return ret;

	if (uV >= 3900000 && uV <= 13500000)
		RT9471_MIVR_uv = uV;

	return ret;
}

static int rt9471_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u32 regval = 0;

	ret = regmap_field_read(chip->rm_field[F_ST_MIVR], &regval);
	if (ret < 0) {
		dev_notice(chip->dev, "%s:(%d) read mivr_state fail\n",
							__func__, ret);
		return ret;
	}
	*in_loop = regval;
	return ret;
}

static int rt9471_get_aicr(struct charger_device *chg_dev, u32 *uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	int ret = 0;

	ret = power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (ret < 0)
		return ret;
	*uA = val.intval;
	return ret;
}

static int rt9471_set_aicr(struct charger_device *chg_dev, u32 uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = uA};

	return power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
}

static int rt9471_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
	*uA = RT9471_AICR_MIN;
	return 0;
}

static int rt9471_get_cv(struct charger_device *chg_dev, u32 *uV)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	int ret = 0;

	ret = power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
	if (ret < 0)
		return ret;
	*uV = val.intval;
	return ret;
}

static int rt9471_set_cv(struct charger_device *chg_dev, u32 uV)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = uV};

	return power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
}

static int rt9471_get_ichg(struct charger_device *chg_dev, u32 *uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	int ret = 0;

	ret = power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val);
	if (ret < 0)
		return ret;
	*uA = val.intval;
	return ret;
}

static int rt9471_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = uA};

	return power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val);
}

static int rt9471_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = RT9471_ICHG_MIN;
	return 0;
}

static int rt9471_get_ieoc(struct charger_device *chg_dev, u32 *uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	int ret = 0;

	ret = power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT, &val);
	if (ret < 0)
		return ret;
	*uA = val.intval;
	return ret;
}

static int rt9471_set_ieoc(struct charger_device *chg_dev, u32 uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = uA};

	return power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT, &val);
}

static int rt9471_reset_eoc_state(struct charger_device *chg_dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return regmap_field_write(chip->rm_field[F_EOC_RST], 0x01);
}

static int rt9471_enable_te(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __rt9471_enable_te(chip, en);
}

static int rt9471_kick_wdt(struct charger_device *chg_dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return regmap_field_write(chip->rm_field[F_WDT_RST], 0x01);
}

static int rt9471_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s event = %d\n", __func__, event);

	power_supply_changed(chip->psy);

	return 0;
}

static int rt9471_enable_powerpath(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	return __rt9471_enable_hz(chip, !en, RT9471_HZU_PP);
}

static int rt9471_is_powerpath_enabled(struct charger_device *chg_dev, bool *en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chip->hz_lock);
	*en = !chip->hz_users[RT9471_HZU_PP];
	mutex_unlock(&chip->hz_lock);

	return 0;
}

static int rt9471_enable_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	return regmap_field_write(chip->rm_field[F_CHG_TMR_EN], en);
}

static int rt9471_is_safety_timer_enabled(struct charger_device *chg_dev,
					  bool *en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	u32 regval = 0;
	int ret = 0;

	ret = regmap_field_read(chip->rm_field[F_CHG_TMR_EN], &regval);
	if (ret < 0) {
		dev_notice(chip->dev, "%s: read safety_timer en fail(%d)\n",
						__func__, ret);
		return ret;
	}
	*en = regval;
	return ret;
}

static int rt9471_enable_otg(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	struct regulator *regulator;
	int ret = 0;

	dev_info(chip->dev, "%s: en = %d\n", __func__, en);
	regulator = devm_regulator_get(chip->dev, "usb-otg-vbus");
	if (IS_ERR(regulator)) {
		dev_notice(chip->dev, "%s: failed to get otg regulator\n",
								__func__);
		return PTR_ERR(regulator);
	}
	ret = en ? regulator_enable(regulator) : regulator_disable(regulator);
	devm_regulator_put(regulator);
	return ret;
}

static int rt9471_enable_discharge(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0)
		return ret;

	ret = regmap_field_write(chip->rm_field[F_FORCE_EN_VBUS_SINK], en);
	if (ret < 0)
		dev_notice(chip->dev, "%s en = %d fail(%d)\n",
				      __func__, en, ret);

	rt9471_enable_hidden_mode(chip, false);

	return ret;
}

static int rt9471_set_boost_current_limit(struct charger_device *chg_dev,
					  u32 uA)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	struct regulator *regulator;
	int ret = 0;

	regulator = devm_regulator_get(chip->dev, "usb-otg-vbus");
	if (IS_ERR(regulator)) {
		dev_notice(chip->dev, "%s: failed to get otg regulator\n",
								__func__);
		return PTR_ERR(regulator);
	}
	ret = regulator_set_current_limit(regulator, uA, uA);
	devm_regulator_put(regulator);
	return ret;
}

static int rt9471_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	__rt9471_enable_hz(chip, !en, RT9471_HZU_BC12);
	atomic_set(&chip->typec_attach, en);
	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->bc12_en, en);
	if (!en) {
		chip->psy_chg_type = POWER_SUPPLY_TYPE_USB;
		chip->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		power_supply_changed(chip->psy);
	}
	mutex_unlock(&chip->bc12_lock);
	wake_up(&chip->bc12_en_req);
#endif /* CONFIG_TCPC_CLASS */
	return ret;
}

static int rt9471_dump_registers(struct charger_device *chg_dev)
{
	int ret = 0, i = 0, ic_stat = 0;
	u32 mivr = 0, aicr = 0, cv = 0, ichg = 0, ieoc = 0;
	bool chg_en = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	static const char * const rt9471_ic_stat_names[] = {
		"hz/sleep", "ready", "trickle-charge", "pre-charge",
		"fast-charge", "ieoc-charge", "background-charge",
		"done", "fault", "RESERVED", "RESERVED", "RESERVED",
		"RESERVED", "RESERVED", "RESERVED", "OTG",
	};
	enum rt9471_stat_idx {
		RT9471_STATIDX_STAT0 = 0,
		RT9471_STATIDX_STAT1,
		RT9471_STATIDX_STAT2,
		RT9471_STATIDX_STAT3,
		RT9471_STATIDX_MAX,
	};
	u8 stats[RT9471_STATIDX_MAX] = {0}, regval = 0, hidden_2 = 0;
	static u8 rt9471_reg_addr[] = {
		RT9471_REG_OTGCFG,
		RT9471_REG_TOP,
		RT9471_REG_FUNCTION,
		RT9471_REG_IBUS,
		RT9471_REG_VBUS,
		RT9471_REG_PRECHG,
		RT9471_REG_REGU,
		RT9471_REG_VCHG,
		RT9471_REG_ICHG,
		RT9471_REG_CHGTIMER,
		RT9471_REG_EOC,
		RT9471_REG_INFO,
		RT9471_REG_JEITA,
		RT9471_REG_PUMPEXP,
		RT9471_REG_DPDMDET,
		RT9471_REG_STATUS,
		RT9471_REG_STAT0,
		RT9471_REG_STAT1,
		RT9471_REG_STAT2,
		RT9471_REG_STAT3,
		/* Skip IRQs to prevent reading clear while dumping registers */
		RT9471_REG_MASK0,
		RT9471_REG_MASK1,
		RT9471_REG_MASK2,
		RT9471_REG_MASK3,
	};

	ret = regmap_field_write(chip->rm_field[F_WDT_RST], 0x01);

	ret |= power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT, &val);
	mivr = val.intval;
	ret |= power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	aicr = val.intval;
	ret |= power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
	cv = val.intval;
	ret |= power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val);
	ichg = val.intval;
	ret |= power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT, &val);
	ieoc = val.intval;
	ret |= rt9471_is_charging_enabled(chg_dev, &chg_en);
	ret |= regmap_field_read(chip->rm_field[F_IC_STAT], &ic_stat);
	if (ret)
		dev_notice(chip->dev, " %s: (%d) Failed to get settings\n",
								__func__, ret);

	ret = regmap_bulk_read(chip->regmap,
			RT9471_REG_STAT0, stats, RT9471_STATIDX_MAX);
	ret |= regmap_raw_read(chip->regmap,
			RT9471_REG_HIDDEN_2, &hidden_2, sizeof(hidden_2));
	if (ret)
		dev_notice(chip->dev, " %s: (%d) Failed to read status\n",
								__func__, ret);

	if (ic_stat == RT9471_ICSTAT_CHGFAULT) {
		for (i = 0; i < ARRAY_SIZE(rt9471_reg_addr); i++) {
			ret = regmap_raw_read(chip->regmap, rt9471_reg_addr[i],
						   &regval, sizeof(regval));
			if (ret < 0)
				continue;
			dev_notice(chip->dev, "%s reg0x%02X = 0x%02X\n",
					      __func__, rt9471_reg_addr[i],
					      regval);
		}
	}

	dev_info(chip->dev, "%s MIVR = %dmV, AICR = %dmA\n",
		 __func__, mivr / 1000, aicr / 1000);

	dev_info(chip->dev, "%s CV = %dmV, ICHG = %dmA, IEOC = %dmA\n",
		 __func__, cv / 1000, ichg / 1000, ieoc / 1000);

	dev_info(chip->dev, "%s CHG_EN = %d, IC_STAT = %s\n",
		 __func__, chg_en, rt9471_ic_stat_names[ic_stat]);

	dev_info(chip->dev, "%s STAT0 = 0x%02X, STAT1 = 0x%02X\n", __func__,
		 stats[RT9471_STATIDX_STAT0], stats[RT9471_STATIDX_STAT1]);

	dev_info(chip->dev, "%s STAT2 = 0x%02X, STAT3 = 0x%02X\n", __func__,
		 stats[RT9471_STATIDX_STAT2], stats[RT9471_STATIDX_STAT3]);

	dev_info(chip->dev, "%s HIDDEN_2 = 0x%02X\n", __func__, hidden_2);

	return 0;
}

static int rt9471_run_aicc(struct charger_device *chg_dev, u32 *uA)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	u32 aicr = 0;

	/* Backup the aicr */
	ret = power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (ret < 0)
		return ret;
	aicr = val.intval;
	__rt9471_run_aicc(chip, uA);
	/* Restore the aicr */
	val.intval = aicr;
	return  power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
}

static int rt9471_enable_pump_express(struct rt9471_chip *chip, bool pe20)
{
	int ret = 0;
	unsigned int wait_us = 2800000,
		     check_us = pe20 ? 960000 : 2200000, pe_ongoing = 1,
		     defualt_mivr = 4500000, default_aicr = 500000, pe_setting = 0;
	union power_supply_propval val = {.intval = 0};

	dev_info(chip->dev, "%s pe20 = %d\n", __func__, pe20);

	ret = regmap_field_write(chip->rm_field[F_PE_SEL], pe20 ? 0x01 : 0x00);
	if (ret < 0)
		return ret;
	/* Set MIVR/AICR/CHG_EN */
	val.intval = defualt_mivr;
	ret =  power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT, &val);
	if (ret < 0)
		return ret;
	val.intval = default_aicr;
	ret = power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (ret < 0)
		return ret;
	ret = __rt9471_enable_chg(chip, true);
	if (ret < 0)
		return ret;
	/* Start pump express */
	ret = regmap_field_write(chip->rm_field[F_PE_EN], 0x01);
	if (ret < 0) {
		dev_notice(chip->dev, "%s pe en fail(%d)\n", __func__, ret);
		return ret;
	}
	ret = regmap_read(chip->regmap, RT9471_REG_PUMPEXP, &pe_setting);
	dev_info(chip->dev, "%s: 0x0d = 0x%02x, ret = %d\n", __func__, pe_setting, ret);

	ret = regmap_field_read_poll_timeout(chip->rm_field[F_PE_EN],
					     pe_ongoing, !pe_ongoing, check_us, wait_us);
	if (ret < 0)
		dev_notice(chip->dev, "%s wait pe_done fail(%d)\n", __func__, ret);

	dev_info(chip->dev, "%s: pe_ongoing = %d, ret = %d\n", __func__, pe_ongoing, ret);

	return ret;
}

static int rt9471_send_ta_current_pattern(struct charger_device *chg_dev,
					  bool is_inc)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s is_inc = %d, chip_rev = %d\n",
			    __func__, is_inc, chip->chip_rev);
	if (chip->chip_rev < 4)
		return -EOPNOTSUPP;

	ret = regmap_field_write(chip->rm_field[F_PE10_INC], is_inc?0x01:0x00);
	if (ret < 0)
		return ret;

	return rt9471_enable_pump_express(chip, false);
}

static int rt9471_send_ta20_current_pattern(struct charger_device *chg_dev,
					    u32 uV)
{
	int ret = 0;
	unsigned int sel = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s target = %d, chip_rev = %d\n",
			    __func__, uV, chip->chip_rev);
	if (chip->chip_rev < 4)
		return -EOPNOTSUPP;

	linear_range_get_selector_within(&rt9471_chg_range[RT9471_RANGE_PE20_CODE], uV, &sel);

	dev_info(chip->dev, "%s: sel = %d\n", __func__, sel);
	ret = regmap_field_write(chip->rm_field[F_PE20_CODE], sel);
	if (ret < 0)
		return ret;

	return rt9471_enable_pump_express(chip, true);
}

static int rt9471_reset_ta(struct charger_device *chg_dev)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval val = {.intval = 0};
	u32 aicr = 0;

	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);
	if (chip->chip_rev < 4)
		return -EOPNOTSUPP;

	/* Backup the aicr */
	ret = power_supply_get_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (ret < 0)
		goto out;
	aicr = val.intval;

	/* 50mA */
	val.intval = 50000;
	ret = power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
	if (ret < 0)
		goto out_restore_aicr;
	mdelay(250);
out_restore_aicr:
	/* Restore the aicr */
	val.intval = aicr;
	ret = power_supply_set_property(chip->psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
out:
	return ret;
}

static int rt9471_set_pe20_efficiency_table(struct charger_device *chg_dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);

	return -EOPNOTSUPP;
}

static int rt9471_enable_cable_drop_comp(struct charger_device *chg_dev,
					 bool en)
{
	int ret = 0;
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d, chip_rev = %d\n",
			    __func__, en, chip->chip_rev);
	if (chip->chip_rev < 4)
		return -EOPNOTSUPP;

	if (en)
		return ret;

	ret = regmap_field_write(chip->rm_field[F_PE20_CODE], 0x1F);
	if (ret < 0)
		return ret;

	return rt9471_enable_pump_express(chip, true);
}

static int rt9471_enable_hz(struct charger_device *chg_dev,
					 bool en)
{
	struct rt9471_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	return rt9471_enable_powerpath(chg_dev, !en);
}

static struct charger_ops rt9471_chg_ops = {
	/* cable plug in/out for primary charger */
	.plug_in = rt9471_plug_in,
	.plug_out = rt9471_plug_out,

	/* enable/disable charger */
	.enable = rt9471_enable_charging,
	.is_enabled = rt9471_is_charging_enabled,
	.is_charging_done = rt9471_is_charging_done,

	/* get/set minimun input voltage regulation */
	.get_mivr = rt9471_get_mivr,
	.set_mivr = rt9471_set_mivr,
	.get_mivr_state = rt9471_get_mivr_state,

	/* get/set input current */
	.get_input_current = rt9471_get_aicr,
	.set_input_current = rt9471_set_aicr,
	.get_min_input_current = rt9471_get_min_aicr,

	/* get/set charging voltage */
	.get_constant_voltage = rt9471_get_cv,
	.set_constant_voltage = rt9471_set_cv,

	/* get/set charging current*/
	.get_charging_current = rt9471_get_ichg,
	.set_charging_current = rt9471_set_ichg,
	.get_min_charging_current = rt9471_get_min_ichg,

	/* get/set termination current */
	.get_eoc_current = rt9471_get_ieoc,
	.set_eoc_current = rt9471_set_ieoc,
	.reset_eoc_state = rt9471_reset_eoc_state,

	/* enable te */
	.enable_termination = rt9471_enable_te,

	/* kick wdt */
	.kick_wdt = rt9471_kick_wdt,

	.event = rt9471_event,

	/* enable/disable powerpath for primary charger */
	.enable_powerpath = rt9471_enable_powerpath,
	.is_powerpath_enabled = rt9471_is_powerpath_enabled,

	/* enable/disable chip for secondary charger */
	.enable_chip = rt9471_enable_powerpath,
	.is_chip_enabled = rt9471_is_powerpath_enabled,

	/* enable/disable charging safety timer */
	.enable_safety_timer = rt9471_enable_safety_timer,
	.is_safety_timer_enabled = rt9471_is_safety_timer_enabled,

	/* OTG */
	.enable_otg = rt9471_enable_otg,
	.enable_discharge = rt9471_enable_discharge,
	.set_boost_current_limit = rt9471_set_boost_current_limit,

	/* charger type detection */
	.enable_chg_type_det = rt9471_enable_chg_type_det,

	.dump_registers = rt9471_dump_registers,

	/* new features for chip_rev >= 4, AICC */
	.run_aicl = rt9471_run_aicc,
	/* new features for chip_rev >= 4, PE+/PE+2.0 */
	.send_ta_current_pattern = rt9471_send_ta_current_pattern,
	.send_ta20_current_pattern = rt9471_send_ta20_current_pattern,
	.reset_ta = rt9471_reset_ta,
	.set_pe20_efficiency_table = rt9471_set_pe20_efficiency_table,
	.enable_cable_drop_comp = rt9471_enable_cable_drop_comp,

	/* pe50 */
	.enable_hz = rt9471_enable_hz,
};

static int rt9471_init_chg(struct rt9471_chip *chip)
{
	int ret;

	if (!chip->chg_name)
		chip->chg_name = "dummy_chg";

	chip->chg_dev = charger_device_register(chip->chg_name,
			chip->dev, chip, &rt9471_chg_ops, &chip->chg_props);
	if (!chip->chg_dev->dev.class || IS_ERR_OR_NULL(chip->chg_dev))
		return -EPROBE_DEFER;

	mutex_lock(&chip->hz_lock);
	chip->hz_users[RT9471_HZU_PP] = false;
	chip->hz_users[RT9471_HZU_BC12] = false;
	chip->hz_users[RT9471_HZU_OTG] = true;
	chip->hz_users[RT9471_HZU_VBUS_GD] = true;
	mutex_unlock(&chip->hz_lock);
	/*
	 * Customization for MTK platform
	 * Primary charger: HZ controlled by sink vbus with TCPC enabled,
	 *		    CHG_EN controlled by charging algorithm
	 * Secondary charger: HZ=0 and CHG_EN=1 at needed,
	 *		      e.x.: PE10, PE20, etc...
	 */
	if (!chip->is_primary) {
		ret = __rt9471_enable_hz(chip, true, RT9471_HZU_PP);
		if (ret < 0)
			dev_notice(chip->dev, "%s en hz fail(%d)\n",
					      __func__, ret);
		ret = __rt9471_enable_chg(chip, false);
		if (ret < 0)
			dev_notice(chip->dev, "%s dis chg fail(%d)\n",
					      __func__, ret);
		ret = __rt9471_enable_te(chip, false);
		if(ret < 0)
			dev_notice(chip->dev, "%s dis te fail(%d)\n",
					      __func__, ret);
	}

	return 0;
}

static int rt9471_parse_dt(struct rt9471_chip *chip)
{
	int ret = 0;
	struct device_node *parent_np = chip->dev->of_node, *np = NULL;

	if (!parent_np) {
		dev_notice(chip->dev, "%s no device node\n", __func__);
		return -EINVAL;
	}
	np = of_get_child_by_name(parent_np, "rt9471");
	if (!np) {
		dev_info(chip->dev, "%s no rt9471 device node\n", __func__);
		np = parent_np;
	}
	chip->dev->of_node = np;

	ret = of_property_read_string(np, "chg_name", &chip->chg_name);
	if (ret < 0)
		dev_info(chip->dev, "%s no chg_name(%d)\n", __func__, ret);

	ret = of_property_read_string(np, "chg_alias_name",
				      &chip->chg_props.alias_name);
	if (ret < 0) {
		dev_info(chip->dev, "%s no chg_alias_name(%d)\n",
				    __func__, ret);
		chip->chg_props.alias_name = "rt9471_chg";
	}
	dev_info(chip->dev, "%s name = %s, alias name = %s\n", __func__,
			    chip->chg_name, chip->chg_props.alias_name);

	if (strcmp(chip->chg_name, "primary_chg") == 0)
		chip->is_primary = true;
	return 0;
}
#endif /* CONFIG_MTK_CHARGER */

//Laker: ceb-gpios GPIO_ACTIVE_LOW:1 will invert the normal GPIO high/low.
#define CE_B_DIS 0 //output H
#define CE_B_EN	 1 //output L

static int rt9471_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct rt9471_chip *chip = NULL;

	dev_info(&client->dev, "%s (%s)\n", __func__, RT9471_DRV_VERSION);
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->bc12_lock);
	mutex_init(&chip->hidden_mode_lock);
	i2c_set_clientdata(client, chip);
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	mutex_init(&chip->hz_lock);
	chip->is_primary = false;
	atomic_set(&chip->bc12_en, 0);
	atomic_set(&chip->typec_attach, 0);
	init_waitqueue_head(&chip->bc12_en_req);
	chip->bc12_en_ws =
	wakeup_source_register(chip->dev,
				devm_kasprintf(chip->dev,
				GFP_KERNEL,
				"rt9471_bc12_en_ws.%s",
				dev_name(chip->dev)));
#if 0 //Laker: moved to avoid panic in case of i2c failure. (don't start kthread if i2c error)
	chip->bc12_en_kthread =
		kthread_run(rt9471_bc12_en_kthread, chip, "%s",
			    devm_kasprintf(chip->dev, GFP_KERNEL,
			    "rt9471_bc12_en_kthread.%s",
			    dev_name(chip->dev)));
	if (IS_ERR(chip->bc12_en_kthread)) {
		ret = PTR_ERR(chip->bc12_en_kthread);
		dev_notice(chip->dev, "%s kthread run fail(%d)\n",
				      __func__, ret);
		return ret;
	}
#endif
#endif /* CONFIG_MTK_CHARGER */

	chip->ceb_gpio = devm_gpiod_get_optional(&client->dev, "ceb", GPIOD_OUT_LOW);
	if (IS_ERR(chip->ceb_gpio)) {
		dev_notice(chip->dev, "Config ceb-gpio fail\n");
		return PTR_ERR(chip->ceb_gpio);
	}
	if (chip->ceb_gpio)
		gpiod_set_value(chip->ceb_gpio, CE_B_EN); //Laker: change 0 to CE_B_EN

/*
	ret = devm_delayed_work_autocancel(&client->dev, &chip->leave_bat_supply_dwork,
					   rt9471_leave_battery_supply_wkaround_handler);
*/
	/* there is no devm_delayed_work_autocancel in k510, replace it with other func */
	INIT_DELAYED_WORK(&chip->leave_bat_supply_dwork, rt9471_leave_battery_supply_wkaround_handler);
	ret = devm_add_action(&client->dev, leave_bat_supply_stop_work, chip);
	if (ret)
		return ret;

	ret = device_create_file(chip->dev, &dev_attr_shipping_mode);
	if (ret) {
		dev_notice(chip->dev, "%s create file fail(%d)\n",  __func__, ret);
		return ret;
	}

	ret = rt9471_init_regmap(chip);
	if (ret) {
		dev_notice(chip->dev, "%s register regmap fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = rt9471_check_devinfo(chip);
	if (ret)
		return ret;

	ret = rt9471_reset_register(chip);
	if (ret) {
		dev_notice(chip->dev, "%s reset register fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = rt9471_init_setting(chip);
	if (ret) {
		dev_notice(chip->dev, "%s init setting fail(%d)\n", __func__, ret);
		return ret;
	}

#if IS_ENABLED(CONFIG_MTK_CHARGER)
	ret = rt9471_parse_dt(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s parse dt fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = rt9471_init_chg(chip);
	if (ret)
		dev_notice(chip->dev, "%s register chg dev fail(%d)\n",
				      __func__, ret);
#endif /* CONFIG_MTK_CHARGER */

	ret = rt9471_init_psy(chip);
	if (ret) {
		dev_notice(chip->dev, "%s fail to register power supply (%d)\n", __func__, ret);
		return ret;
	}

	ret = rt9471_init_regulator(chip);
	if (ret) {
		dev_notice(chip->dev, "%s init regulator fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = rt9471_init_irq(chip);
	if (ret)
		dev_notice(chip->dev, "%s init irq fail(%d)\n", __func__, ret);

//Laker: moved to avoid panic in case of i2c failure. (don't start kthread if i2c error)
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	chip->bc12_en_kthread =
		kthread_run(rt9471_bc12_en_kthread, chip, "%s",
			    devm_kasprintf(chip->dev, GFP_KERNEL,
			    "rt9471_bc12_en_kthread.%s",
			    dev_name(chip->dev)));
	if (IS_ERR(chip->bc12_en_kthread)) {
		ret = PTR_ERR(chip->bc12_en_kthread);
		dev_notice(chip->dev, "%s kthread run fail(%d)\n",
				      __func__, ret);
		return ret;
	}
#endif /* CONFIG_MTK_CHARGER */

	return ret;
}

static void rt9471_shutdown(struct i2c_client *i2c)
{
	struct rt9471_chip *chip = i2c_get_clientdata(i2c);
	u8 mask = RT9471_BATFETDIS_MASK | RT9471_HZ_MASK; //Laker: enter shipmode, ref: shipping_mode_store()

	dev_info(chip->dev, "%s\n", __func__);
	/*
	 * There's no external reset pin. Do register reset to guarantee charger
	 * function is normal after shutdown
	 */
	rt9471_reset_register(chip);

	//Laker: enter shipmode, ref: shipping_mode_store()
	if (regmap_update_bits(chip->regmap, RT9471_REG_FUNCTION, mask, mask))
		dev_notice(chip->dev, "%s: enable shipmode fail!\n", __func__);
	else
		dev_info(chip->dev, "%s: enable shipmode successfully!\n", __func__);
}

static const struct of_device_id rt9471_of_device_id[] = {
	{ .compatible = "richtek,rt9471", },
	{},
};
MODULE_DEVICE_TABLE(of, rt9471_of_device_id);

static struct i2c_driver rt9471_i2c_driver = {
	.driver = {
		.name = "rt9471",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt9471_of_device_id),
	},
	.probe = rt9471_probe,
	.shutdown = rt9471_shutdown,
};
module_i2c_driver(rt9471_i2c_driver);

MODULE_DESCRIPTION("Richtek rt9471 Charger driver");
MODULE_AUTHOR("Alina Yu <alina_yu@richtek.com>");
MODULE_LICENSE("GPL v2");

