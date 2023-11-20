#include <linux/err.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include <linux/proc_fs.h>

#define	LID_DEV_NAME	"hall_sensor"
#define HALL_INPUT	"/dev/input/hall_dev"

struct hall_data {
	int gpio;	/* device use gpio number */
	int irq;	/* device request irq number */
	int irq_enabled;
	int active_low;	/* gpio active high or low for valid value */
	bool wakeup;	/* device can wakeup system or not */
	struct input_dev *hall_dev;
};

static irqreturn_t hall_interrupt_handler(int irq, void *dev)
{
	int value;
	struct hall_data *data = dev;

	value = (gpio_get_value_cansleep(data->gpio) ? 1 : 0) ^
		data->active_low;
	if (value) {
		input_report_switch(data->hall_dev, SW_LID, 0);
		dev_dbg(&data->hall_dev->dev, "far\n");
	} else {
		input_report_switch(data->hall_dev, SW_LID, 1);
		dev_dbg(&data->hall_dev->dev, "near\n");
	}
	input_sync(data->hall_dev);

	return IRQ_HANDLED;
}

static int hall_input_init(struct platform_device *pdev,
		struct hall_data *data)
{
	int err = -1;

	data->hall_dev = devm_input_allocate_device(&pdev->dev);
	if (!data->hall_dev) {
		dev_err(&data->hall_dev->dev,
				"input device allocation failed\n");
		return -EINVAL;
	}
	data->hall_dev->name = LID_DEV_NAME;
	data->hall_dev->phys = HALL_INPUT;
	__set_bit(EV_SW, data->hall_dev->evbit);
	__set_bit(SW_LID, data->hall_dev->swbit);

	err = input_register_device(data->hall_dev);
	if (err < 0) {
		dev_err(&data->hall_dev->dev,
				"unable to register input device %s\n",
				LID_DEV_NAME);
		return err;
	}

	return 0;
}

#ifdef CONFIG_OF
static int hall_parse_dt(struct device *dev, struct hall_data *data)
{
	unsigned int tmp;
	struct device_node *np = dev->of_node;

	data->gpio = of_get_named_gpio_flags(dev->of_node,
			"linux,gpio-int", 0, &tmp);
	if (!gpio_is_valid(data->gpio)) {
		dev_err(dev, "hall gpio is not valid\n");
		return -EINVAL;
	}
	data->active_low = tmp & OF_GPIO_ACTIVE_LOW ? 0 : 1;
	data->wakeup = of_property_read_bool(np, "linux,wakeup");

	return 0;
}
#else
static int hall_parse_dt(struct device *dev, struct hall_data *data)
{
	return -EINVAL;
}
#endif

#define PROC_FILE_HALL "hallsensor"
#define HALL_IRQ_ENABLED "hallsensor/0enable"

static int proc_hall_irq_enabled_show(struct seq_file *s, void *unused)
{
	struct hall_data *data = s->private;

	seq_printf(s, "%d\n", data->irq_enabled);

	return 0;
}

static int hall_irq_enabled_procfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_hall_irq_enabled_show, PDE_DATA(inode));
}

static ssize_t hall_irq_enabled_procfs_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	int ret, cmd;
	char kbuf[256];
	size_t len = 0;

	struct seq_file *s = file->private_data;
	struct hall_data *data = s->private;

	len = min(count, (sizeof(kbuf) - 1));
	pr_info("count: %d", count);
	if (count == 0)
		return -1;
	if (count > 255)
		count = 255;
	ret = copy_from_user(kbuf, buffer, count);
	if (ret < 0)
		return -1;
	kbuf[count] = '\0';
	ret = kstrtoint(kbuf, 10, &cmd);
	pr_info("hall_irq_enabled_procfs_write ret[%d] cmd[%d] hall_irq_enabled[%d]\n", ret, cmd, data->irq_enabled);
	if (cmd == 0) {
		if (data->irq_enabled != 0) {
			disable_irq_wake(data->irq);
			disable_irq(data->irq);
			data->irq_enabled = 0;
		}
	} else if (cmd == 1) {
		if (data->irq_enabled != 1) {
			enable_irq_wake(data->irq);
			enable_irq(data->irq);
			data->irq_enabled = 1;
		}
	}
	if (ret)
		return ret;
	return count;
}

static const struct  proc_ops hall_irq_enabled_procfs_fops = {
	.proc_open = hall_irq_enabled_procfs_open,
	.proc_read = seq_read,
	.proc_write	= hall_irq_enabled_procfs_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int hall_driver_procfs_init(struct hall_data *data)
{
	struct proc_dir_entry *file;
	struct proc_dir_entry *root;

	root = proc_mkdir(PROC_FILE_HALL, NULL);

	file = proc_create_data(HALL_IRQ_ENABLED, 0664, NULL,
		&hall_irq_enabled_procfs_fops, data);

	return 0;
}

static int hall_driver_probe(struct platform_device *dev)
{
	struct hall_data *data;
	int err = 0;
	int irq_flags;

	dev_dbg(&dev->dev, "hall_driver probe\n");
	data = devm_kzalloc(&dev->dev, sizeof(struct hall_data), GFP_KERNEL);
	if (data == NULL) {
		err = -ENOMEM;
		dev_err(&dev->dev,
				"failed to allocate memory %d\n", err);
		goto exit;
	}
	dev_set_drvdata(&dev->dev, data);
	if (dev->dev.of_node) {
		err = hall_parse_dt(&dev->dev, data);
		if (err < 0) {
			dev_err(&dev->dev, "Failed to parse device tree\n");
			goto exit;
		}
	} else if (dev->dev.platform_data != NULL) {
		memcpy(data, dev->dev.platform_data, sizeof(*data));
	} else {
		dev_err(&dev->dev, "No valid platform data.\n");
		err = -ENODEV;
		goto exit;
	}

	err = hall_input_init(dev, data);
	if (err < 0) {
		dev_err(&dev->dev, "input init failed\n");
		goto exit;
	}

	if (!gpio_is_valid(data->gpio)) {
		dev_err(&dev->dev, "gpio is not valid\n");
		err = -EINVAL;
		goto exit;
	}

	irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
		| IRQF_ONESHOT;
	err = gpio_request_one(data->gpio, GPIOF_DIR_IN, "hall_sensor_irq");
	if (err) {
		dev_err(&dev->dev, "unable to request gpio %d\n", data->gpio);
		goto exit;
	}

	data->irq = gpio_to_irq(data->gpio);
	err = devm_request_threaded_irq(&dev->dev, data->irq, NULL,
			hall_interrupt_handler,
			irq_flags, "hall_sensor", data);
	if (err < 0) {
		dev_err(&dev->dev, "request irq failed : %d\n", data->irq);
		goto free_gpio;
	}

	device_init_wakeup(&dev->dev, data->wakeup);
	enable_irq_wake(data->irq);

	data->irq_enabled = 1;
	hall_driver_procfs_init(data);

	return 0;

#if 0
err_regulator_init:
	hall_config_regulator(dev, false);
free_irq:
	disable_irq_wake(data->irq);
	device_init_wakeup(&dev->dev, 0);
#endif
free_gpio:
	gpio_free(data->gpio);
exit:
	return err;
}

static int hall_driver_remove(struct platform_device *dev)
{
	struct hall_data *data = dev_get_drvdata(&dev->dev);

	disable_irq_wake(data->irq);
	device_init_wakeup(&dev->dev, 0);
	if (data->gpio)
		gpio_free(data->gpio);

	return 0;
}

static struct platform_device_id hall_id[] = {
	{LID_DEV_NAME, 0 },
	{ },
};


#ifdef CONFIG_OF
static struct of_device_id hall_match_table[] = {
	{.compatible = "hall-switch", },
	{ },
};
#endif

static struct platform_driver hall_driver = {
	.driver = {
		.name = LID_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hall_match_table),
	},
	.probe = hall_driver_probe,
	.remove = hall_driver_remove,
	.id_table = hall_id,
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_DESCRIPTION("Hall sensor driver");
MODULE_LICENSE("GPL v2");
