#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define DEVICE_NAME "mq2adc"
#define CLASS_NAME "mq2adc_class"

#define ADS1115_ADDR 0x48

#define ADS1115_REG_CONVERT  0x00
#define ADS1115_REG_CONFIG   0x01

#define ADS1115_CONFIG_MSB 0xC2
#define ADS1115_CONFIG_LSB 0x83

#define GPIO1_BASE 0x4804C000
#define GPIO_SIZE  0x1000

#define GPIO_OE        0x134
#define GPIO_DATAIN    0x138
#define GPIO_DATAOUT   0x13C
#define GPIO_SETDATAOUT 0x194
#define GPIO_CLEARDATAOUT 0x190

#define SCL_PIN 17  // P9_23 - GPIO1_17
#define SDA_PIN 16  // P9_15 - GPIO1_16

#define SCL_MASK (1 << SCL_PIN)
#define SDA_MASK (1 << SDA_PIN)

#define I2C_DELAY 5

static dev_t dev_num;
static struct class *mq2adc_class = NULL;
static struct cdev mq2adc_cdev;
static struct device *mq2adc_device = NULL;
static void __iomem *gpio_base;

static bool sensor_enabled = false;

static inline void set_scl(int value) {
    if (value)
        iowrite32(SCL_MASK, gpio_base + GPIO_SETDATAOUT);
    else
        iowrite32(SCL_MASK, gpio_base + GPIO_CLEARDATAOUT);
    udelay(I2C_DELAY);
}

static inline void set_sda(int value) {
    if (value)
        iowrite32(SDA_MASK, gpio_base + GPIO_SETDATAOUT);
    else
        iowrite32(SDA_MASK, gpio_base + GPIO_CLEARDATAOUT);
    udelay(I2C_DELAY);
}

static inline int read_sda(void) {
    u32 val, oe_val;
    oe_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(oe_val | SDA_MASK, gpio_base + GPIO_OE);
    val = ioread32(gpio_base + GPIO_DATAIN);
    iowrite32(oe_val, gpio_base + GPIO_OE);
    return (val & SDA_MASK) ? 1 : 0;
}

static inline void sda_dir_in(void) {
    u32 reg_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(reg_val | SDA_MASK, gpio_base + GPIO_OE);
    udelay(I2C_DELAY);
}

static inline void sda_dir_out(void) {
    u32 reg_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(reg_val & ~SDA_MASK, gpio_base + GPIO_OE);
    udelay(I2C_DELAY);
}

static void i2c_start(void) {
    sda_dir_out();
    set_sda(1);
    set_scl(1);
    set_sda(0);
    set_scl(0);
}

static void i2c_stop(void) {
    sda_dir_out();
    set_sda(0);
    set_scl(1);
    set_sda(1);
}

static int i2c_write_byte(unsigned char byte) {
    int i, ack;
    sda_dir_out();
    for (i = 7; i >= 0; i--) {
        set_sda((byte >> i) & 1);
        set_scl(1);
        set_scl(0);
    }
    sda_dir_in();
    set_scl(1);
    ack = !read_sda();
    set_scl(0);
    return ack;
}

static unsigned char i2c_read_byte(int ack) {
    int i;
    unsigned char byte = 0;
    sda_dir_in();
    for (i = 7; i >= 0; i--) {
        set_scl(1);
        if (read_sda())
            byte |= (1 << i);
        set_scl(0);
    }
    sda_dir_out();
    set_sda(!ack);
    set_scl(1);
    set_scl(0);
    return byte;
}

static int ads1115_read_adc(int16_t *adc_value) {
    unsigned char msb, lsb;

    i2c_start();
    if (!i2c_write_byte(ADS1115_ADDR << 1)) goto err;
    i2c_write_byte(ADS1115_REG_CONFIG);
    i2c_write_byte(ADS1115_CONFIG_MSB);
    i2c_write_byte(ADS1115_CONFIG_LSB);
    i2c_stop();
    msleep(10);

    i2c_start();
    if (!i2c_write_byte(ADS1115_ADDR << 1)) goto err;
    i2c_write_byte(ADS1115_REG_CONVERT);
    i2c_stop();

    i2c_start();
    if (!i2c_write_byte((ADS1115_ADDR << 1) | 1)) goto err;
    msb = i2c_read_byte(1);
    lsb = i2c_read_byte(0);
    i2c_stop();

    *adc_value = (msb << 8) | lsb;
    return 0;

err:
    i2c_stop();
    return -EIO;
}

static ssize_t mq2adc_read(struct file *file, char __user *buf, size_t count, loff_t *offset) {
    int16_t adc;
    char out[32];
    int len;

    if (*offset > 0) return 0;
    if (!sensor_enabled) {
        len = snprintf(out, sizeof(out), "Sensor OFF\n");
    } else {
        if (ads1115_read_adc(&adc) < 0)
            return -EIO;
        len = snprintf(out, sizeof(out), "ADC: %d\n", adc);
    }
    if (copy_to_user(buf, out, len)) return -EFAULT;
    *offset += len;
    return len;
}

static ssize_t mq2adc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
    char cmd[16] = {0};
    if (count > sizeof(cmd) - 1)
        return -EINVAL;
    if (copy_from_user(cmd, buf, count))
        return -EFAULT;
    cmd[count] = '\0';

    if (strncmp(cmd, "ON", 2) == 0) {
        sensor_enabled = true;
        printk(KERN_INFO "MQ2 Sensor ON\n");
    } else if (strncmp(cmd, "OFF", 3) == 0) {
        sensor_enabled = false;
        printk(KERN_INFO "MQ2 Sensor OFF\n");
    } else {
        printk(KERN_INFO "MQ2ADC Unknown command: %s\n", cmd);
    }
    return count;
}

static int mq2adc_open(struct inode *inode, struct file *file) {
    return 0;
}

static int mq2adc_release(struct inode *inode, struct file *file) {
    return 0;
}

static const struct file_operations mq2adc_fops = {
    .owner = THIS_MODULE,
    .read = mq2adc_read,
    .write = mq2adc_write,
    .open = mq2adc_open,
    .release = mq2adc_release,
};

static int setup_gpio(void) {
    u32 reg_val;

    gpio_base = ioremap(GPIO1_BASE, GPIO_SIZE);
    if (!gpio_base) return -ENOMEM;

    reg_val = ioread32(gpio_base + GPIO_OE);
    reg_val &= ~(SCL_MASK | SDA_MASK);
    iowrite32(reg_val, gpio_base + GPIO_OE);

    iowrite32(SCL_MASK | SDA_MASK, gpio_base + GPIO_SETDATAOUT);
    return 0;
}

static int __init mq2adc_init(void) {
    int ret;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;

    cdev_init(&mq2adc_cdev, &mq2adc_fops);
    mq2adc_cdev.owner = THIS_MODULE;
    ret = cdev_add(&mq2adc_cdev, dev_num, 1);
    if (ret < 0) goto fail_cdev;

    mq2adc_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(mq2adc_class)) {
        ret = PTR_ERR(mq2adc_class);
        goto fail_class;
    }

    mq2adc_device = device_create(mq2adc_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(mq2adc_device)) {
        ret = PTR_ERR(mq2adc_device);
        goto fail_device;
    }

    ret = setup_gpio();
    if (ret < 0) goto fail_gpio;

    printk(KERN_INFO "MQ2_ADC: Module loaded successfully.\n");
    return 0;

fail_gpio:
    device_destroy(mq2adc_class, dev_num);
fail_device:
    class_destroy(mq2adc_class);
fail_class:
    cdev_del(&mq2adc_cdev);
fail_cdev:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void __exit mq2adc_exit(void) {
    iounmap(gpio_base);
    device_destroy(mq2adc_class, dev_num);
    class_destroy(mq2adc_class);
    cdev_del(&mq2adc_cdev);
    unregister_chrdev_region(dev_num, 1);
    printk(KERN_INFO "MQ2_ADC: Module unloaded.\n");
}

module_init(mq2adc_init);
module_exit(mq2adc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dam Quoc + ChatGPT");
MODULE_DESCRIPTION("MQ2 Gas Sensor Driver using ADS1115 + Bit-banging I2C");
MODULE_VERSION("1.0");

