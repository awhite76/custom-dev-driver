// SPDX-License-Identifier: GPL-2.0
// ADS1015 round-robin reader paced by ALERT/RDY interrupts
// Raspberry Pi Zero 2 W – kernel-6.12-compatible version (descriptor GPIOs)

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h> // modern gpiod API

#define DRV_NAME "ads1015_rr"

/* Module parameters still used only for I²C and ADC config */
static int i2c_bus = 1;
module_param(i2c_bus, int, 0444);
MODULE_PARM_DESC(i2c_bus, "I2C bus number");

static int i2c_addr = 0x48;
module_param(i2c_addr, int, 0444);
MODULE_PARM_DESC(i2c_addr, "ADS1015 7-bit address");

static int pga_mv = 4096;
module_param(pga_mv, int, 0444);
MODULE_PARM_DESC(pga_mv, "Full-scale range in mV");

static int sps = 1600;
module_param(sps, int, 0444);
MODULE_PARM_DESC(sps, "Samples per second");

/* ADS1015 registers */
#define REG_CONV 0x00
#define REG_CONFIG 0x01
#define REG_LOTH 0x02
#define REG_HITH 0x03

struct ads1015_rr
{
    struct i2c_client *client;

    /* Descriptor GPIOs from Device Tree */
    struct gpio_desc *alert;
    struct gpio_desc *lswitch;
    struct gpio_desc *rswitch;

    int irq;
    int sw_irq[2];

    struct mutex lock;
    u16 raw[4];
    int ch;
    bool sw_st[2];

    struct dentry *dbg_dir;
    struct dentry *dbg_file;
};

static struct ads1015_rr *gdev;

/* ---------- I²C helpers ---------- */

static int ads_w16(struct i2c_client *c, u8 reg, u16 val_be)
{
    u8 buf[2] = {val_be >> 8, val_be & 0xff};
    return i2c_smbus_write_i2c_block_data(c, reg, 2, buf);
}

static int ads_r16(struct i2c_client *c, u8 reg, u16 *out_be)
{
    u8 buf[2];
    int ret = i2c_smbus_read_i2c_block_data(c, reg, 2, buf);
    if (ret < 0)
        return ret;
    *out_be = (buf[0] << 8) | buf[1];
    return 0;
}

/* ---------- Config helpers ---------- */

static u16 pga_bits_from_mv(int mv)
{
    switch (mv)
    {
    case 6144:
        return 0x0;
    case 4096:
        return 0x1;
    case 2048:
        return 0x2;
    case 1024:
        return 0x3;
    case 512:
        return 0x4;
    case 256:
        return 0x5;
    default:
        return 0x1;
    }
}

static u16 dr_bits_from_sps(int sps_in)
{
    if (sps_in >= 3300)
        return 0x7;
    if (sps_in >= 2400)
        return 0x6;
    if (sps_in >= 1600)
        return 0x5;
    if (sps_in >= 920)
        return 0x4;
    if (sps_in >= 490)
        return 0x3;
    if (sps_in >= 250)
        return 0x2;
    return 0x1;
}

static u16 build_config(int ch)
{
    u16 cfg = 0;
    cfg |= (1u << 15);                  /* OS=1 start */
    cfg |= ((0x4u + (ch & 0x3)) << 12); /* MUX */
    cfg |= (pga_bits_from_mv(pga_mv) & 0x7) << 9;
    cfg |= (1u << 8); /* single-shot */
    cfg |= (dr_bits_from_sps(sps) & 0x7) << 5;
    return cfg;
}

static int start_conv(struct ads1015_rr *dev)
{
    return ads_w16(dev->client, REG_CONFIG, build_config(dev->ch));
}

/* ---------- IRQ handlers ---------- */

static irqreturn_t ads_irq_thread(int irq, void *data)
{
    struct ads1015_rr *dev = data;
    u16 be;
    int ret;

    mutex_lock(&dev->lock);

    ret = ads_r16(dev->client, REG_CONV, &be);
    if (!ret)
    {
        s16 val = (s16)be >> 4;
        if (val & 0x0800)
            val |= 0xF000;
        dev->raw[dev->ch] = (u16)val;
    }
    dev->ch = (dev->ch + 1) & 0x3;
    start_conv(dev);

    mutex_unlock(&dev->lock);
    return IRQ_HANDLED;
}

static irqreturn_t ads_sw_irq(int irq, void *data)
{
    struct ads1015_rr *dev = data;

    if (irq == dev->sw_irq[0])
        dev->sw_st[0] = gpiod_get_value_cansleep(dev->lswitch);
    else if (irq == dev->sw_irq[1])
        dev->sw_st[1] = gpiod_get_value_cansleep(dev->rswitch);

    return IRQ_HANDLED;
}

/* ---------- DebugFS ---------- */

static ssize_t dbg_values_read(struct file *f, char __user *ubuf,
                               size_t cnt, loff_t *ppos)
{
    char buf[256];
    int len, i;
    struct ads1015_rr *dev = gdev;

    mutex_lock(&dev->lock);
    len = scnprintf(buf, sizeof(buf), "ch raw volts(mV)\n");
    for (i = 0; i < 4; i++)
    {
        s16 v = (s16)dev->raw[i];
        int mv = (int)((long long)v * pga_mv / 2047);
        len += scnprintf(buf + len, sizeof(buf) - len,
                         "%d %6d %7d\n", i, v, mv);
    }
    len += scnprintf(buf + len, sizeof(buf) - len,
                     "L_switch=%d  R_switch=%d\n",
                     dev->sw_st[0], dev->sw_st[1]);
    mutex_unlock(&dev->lock);

    return simple_read_from_buffer(ubuf, cnt, ppos, buf, len);
}

static const struct file_operations dbg_fops = {
    .owner = THIS_MODULE,
    .read = dbg_values_read,
    .llseek = noop_llseek,
};

/* ---------- ADS1015 init ---------- */

static int ads1015_init_device(struct ads1015_rr *dev)
{
    int ret;
    ret = ads_w16(dev->client, REG_HITH, 0x8000);
    if (ret)
        return ret;
    ret = ads_w16(dev->client, REG_LOTH, 0x0000);
    if (ret)
        return ret;
    dev->ch = 0;
    return start_conv(dev);
}

/* ---------- Module init / exit ---------- */

static int __init ads1015_rr_init(void)
{
    struct i2c_adapter *adap;
    struct i2c_board_info info = {I2C_BOARD_INFO("ads1015-rr", 0)};
    struct device *dev;
    int ret, i;

    info.addr = i2c_addr & 0x7f;

    gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);
    if (!gdev)
        return -ENOMEM;
    mutex_init(&gdev->lock);

    adap = i2c_get_adapter(i2c_bus);
    if (!adap)
    {
        ret = -ENODEV;
        goto err_free_irqs;
    }

    gdev->client = i2c_new_client_device(adap, &info);
    i2c_put_adapter(adap);
    if (IS_ERR(gdev->client))
    {
        ret = PTR_ERR(gdev->client);
        goto err_free_irqs;
    }

    dev = &gdev->client->dev;

    /* --- GPIO descriptors from Device Tree --- */
    gdev->alert = gpiod_get(dev, "alert", GPIOD_IN);
    if (IS_ERR(gdev->alert))
        return dev_err_probe(dev, PTR_ERR(gdev->alert), "alert GPIO\n");

    gdev->lswitch = gpiod_get(dev, "lswitch", GPIOD_IN);
    if (IS_ERR(gdev->lswitch))
        return dev_err_probe(dev, PTR_ERR(gdev->lswitch), "lswitch GPIO\n");

    gdev->rswitch = gpiod_get(dev, "rswitch", GPIOD_IN);
    if (IS_ERR(gdev->rswitch))
        return dev_err_probe(dev, PTR_ERR(gdev->rswitch), "rswitch GPIO\n");

    /* Convert to IRQs */
    gdev->irq = gpiod_to_irq(gdev->alert);
    gdev->sw_irq[0] = gpiod_to_irq(gdev->lswitch);
    gdev->sw_irq[1] = gpiod_to_irq(gdev->rswitch);

    for (i = 0; i < 2; i++)
        gdev->sw_st[i] =
            gpiod_get_value_cansleep(i == 0 ? gdev->lswitch : gdev->rswitch);

    /* Initialise ADC */
    ret = ads1015_init_device(gdev);
    if (ret)
        goto err_gpios;

    /* Request IRQs */
    ret = request_threaded_irq(gdev->irq, NULL, ads_irq_thread,
                               IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                               DRV_NAME, gdev);
    if (ret)
        goto err_gpios;

    for (i = 0; i < 2; i++)
    {
        ret = request_irq(gdev->sw_irq[i], ads_sw_irq,
                          IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                          DRV_NAME, gdev);
        if (ret)
            goto err_free_irqs;
    }

    /* DebugFS */
    gdev->dbg_dir = debugfs_create_dir("ads1015_rr", NULL);
    gdev->dbg_file = debugfs_create_file("values", 0444,
                                         gdev->dbg_dir, NULL, &dbg_fops);

    pr_info(DRV_NAME ": loaded (bus=%d addr=0x%02x)\n",
            i2c_bus, i2c_addr);
    return 0;

err_free_irqs:
    free_irq(gdev->irq, gdev);
err_gpios:
    gpiod_put(gdev->alert);
    gpiod_put(gdev->lswitch);
    gpiod_put(gdev->rswitch);
    mutex_destroy(&gdev->lock);
    kfree(gdev);
    return ret;
}

static void __exit ads1015_rr_exit(void)
{
    int i;
    if (!gdev)
        return;

    if (gdev->dbg_file)
        debugfs_remove(gdev->dbg_file);
    if (gdev->dbg_dir)
        debugfs_remove(gdev->dbg_dir);

    free_irq(gdev->irq, gdev);
    for (i = 0; i < 2; i++)
        free_irq(gdev->sw_irq[i], gdev);

    gpiod_put(gdev->alert);
    gpiod_put(gdev->lswitch);
    gpiod_put(gdev->rswitch);

    if (gdev->client && !IS_ERR(gdev->client))
        i2c_unregister_device(gdev->client);

    mutex_destroy(&gdev->lock);
    kfree(gdev);
    pr_info(DRV_NAME ": unloaded\n");
}

module_init(ads1015_rr_init);
module_exit(ads1015_rr_exit);

MODULE_AUTHOR("Avery White");
MODULE_DESCRIPTION("ADS1015 round-robin via ALERT/RDY interrupt (gpiod)");
MODULE_LICENSE("GPL");
