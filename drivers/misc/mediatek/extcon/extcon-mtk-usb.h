/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

struct mtk_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct usb_role_switch *role_sw;
	unsigned int c_role; /* current data role */
	unsigned int usb_role;
	struct workqueue_struct *extcon_wq;
	struct regulator *vbus;
	struct regulator *pogovbus;
	unsigned int vbus_vol;
	unsigned int vbus_cur;
	bool vbus_on;
	bool pogoid_on;
	bool pogo_usb_on;
	struct power_supply *usb_psy;
	struct notifier_block psy_nb;
	struct delayed_work wq_psy;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct tcpc_device *tcpc_dev;
	struct notifier_block tcpc_nb;
#endif
	bool bypss_typec_sink;
	/* id gpio */
	struct gpio_desc *id_gpiod;
	struct gpio_desc *usb_sel_gpiod;
	struct gpio_desc *pogo_vbus_gpiod;
	struct gpio_desc *pogo_id_gpiod;
	int id_irq;
	int pogo_id_irq;
	struct delayed_work wq_detcable;
	struct delayed_work wq_detpogoid;
};

struct usb_role_info {
	struct mtk_extcon_info *extcon;
	struct delayed_work dwork;
	unsigned int d_role; /* desire data role */
};

enum {
	DUAL_PROP_MODE_UFP = 0,
	DUAL_PROP_MODE_DFP,
	DUAL_PROP_MODE_NONE,
};

enum {
	DUAL_PROP_PR_SRC = 0,
	DUAL_PROP_PR_SNK,
	DUAL_PROP_PR_NONE,
};
