#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>

#define DEVICE_NAME "dht11"
#define CLASS_NAME "dht11_class"

#define GPIO1_BASE_ADDR  0x4804C000
#define GPIO_SIZE        0x1000
#define GPIO_OE          0x134
#define GPIO_DATAIN      0x138
#define GPIO_DATAOUT     0x13C

#define DHT11_PIN     15
#define DHT11_MASK    (1 << DHT11_PIN)

static void __iomem *gpio_base;
static dev_t dev_num;
static struct class *dht_class;
static struct cdev dht_cdev;
static int dht_enabled = 0;

static void dht_set_output(void) {
    u32 oe = ioread32(gpio_base + GPIO_OE);
    oe &= ~DHT11_MASK;
    iowrite32(oe, gpio_base + GPIO_OE);
}

static void dht_set_input(void) {
    u32 oe = ioread32(gpio_base + GPIO_OE);
    oe |= DHT11_MASK;
    iowrite32(oe, gpio_base + GPIO_OE);
}

static void dht_write(int val) {
    u32 out = ioread32(gpio_base + GPIO_DATAOUT);
    if (val) out |= DHT11_MASK;
    else     out &= ~DHT11_MASK;
    iowrite32(out, gpio_base + GPIO_DATAOUT);
}

static int dht_read(void) {
    return (ioread32(gpio_base + GPIO_DATAIN) & DHT11_MASK) ? 1 : 0;
}

static int dht11_read_data(uint8_t *humidity, uint8_t *temperature) {
    uint8_t data[5] = {0};
    int i, j, timeout;

    dht_set_output();
    dht_write(0);
    mdelay(20);
    dht_write(1);
    udelay(30);
    dht_set_input();

    // Wait for sensor response: LOW 80us, then HIGH 80us
    timeout = 100;
    while (dht_read() && timeout--) udelay(1);
    if (timeout <= 0) return -1;

    timeout = 100;
    while (!dht_read() && timeout--) udelay(1);
    if (timeout <= 0) return -1;

    timeout = 100;
    while (dht_read() && timeout--) udelay(1);
    if (timeout <= 0) return -1;

    // Read 40 bits = 5 bytes
    for (j = 0; j < 5; j++) {
        for (i = 0; i < 8; i++) {
            while (!dht_read()); // Wait for LOW->HIGH
            udelay(40);
            if (dht_read()) {
                data[j] |= (1 << (7 - i));
                while (dht_read()); // Wait for HIGH->LOW
            }
        }
    }

    // Verify checksum
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4])
        return -2;

    *humidity = data[0];
    *temperature = data[2];
    return 0;
}

static ssize_t dht_read_file(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    char result[64];
    int ret;
    uint8_t hum, temp;

    if (*offset > 0) return 0;

    if (!dht_enabled) {
        snprintf(result, sizeof(result), "OFF\n");
    } else {
        ret = dht11_read_data(&hum, &temp);
        if (ret == -2)
            snprintf(result, sizeof(result), "Loi checksum!\n");
        else if (ret < 0)
            snprintf(result, sizeof(result), "Cam bien khong phan hoi!\n");
        else
            snprintf(result, sizeof(result),
                     "Nhiet do: %d°C\nDo am: %d%%\n", temp, hum);
    }

    if (copy_to_user(buf, result, strlen(result)))
        return -EFAULT;

    *offset += strlen(result);
    return strlen(result);
}


static ssize_t dht_write_file(struct file *file, const char __user *buf, size_t len, loff_t *offset) {
    char command[8] = {0};

    if (copy_from_user(command, buf, min(len, sizeof(command) - 1)))
        return -EFAULT;

    if (strncmp(command, "ON", 2) == 0) {
        dht_enabled = 1;
        pr_info("DHT11 enabled\n");
    } else if (strncmp(command, "OFF", 3) == 0) {
        dht_enabled = 0;
        pr_info("DHT11 disabled\n");
    } else {
        pr_info("Unknown command: %s\n", command);
    }

    return len;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read  = dht_read_file,
    .write = dht_write_file,
};

static int __init dht_init(void) {
    int ret;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;

    cdev_init(&dht_cdev, &fops);
    ret = cdev_add(&dht_cdev, dev_num, 1);
    if (ret < 0) return ret;

    dht_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(dht_class)) return PTR_ERR(dht_class);

    device_create(dht_class, NULL, dev_num, NULL, DEVICE_NAME);

    gpio_base = ioremap(GPIO1_BASE_ADDR, GPIO_SIZE);
    if (!gpio_base) {
        pr_err("Failed to ioremap GPIO1\n");
        return -ENOMEM;
    }

    dht_set_input();
    pr_info("DHT11 driver loaded, major=%d\n", MAJOR(dev_num));
    return 0;
}

static void __exit dht_exit(void) {
    iounmap(gpio_base);
    device_destroy(dht_class, dev_num);
    class_destroy(dht_class);
    cdev_del(&dht_cdev);
    unregister_chrdev_region(dev_num, 1);
    pr_info("DHT11 driver unloaded\n");
}

module_init(dht_init);
module_exit(dht_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("YourName");
MODULE_DESCRIPTION("Kernel driver for DHT11 using ioremap (P8_15 - GPIO1_15)");

