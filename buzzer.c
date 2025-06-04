#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>

#define DEVICE_NAME "buzzer"
#define CLASS_NAME  "buzzer_class"

// P9_12 = GPIO1_28 = 1*32 + 28
#define GPIO1_BASE_ADDR    0x4804C000
#define GPIO_SIZE          0x1000
#define GPIO_OE            0x134
#define GPIO_DATAOUT       0x13C
#define BUZZER_PIN         28
#define BUZZER_MASK        (1 << BUZZER_PIN)

static dev_t dev_num;
static struct class *buzzer_class;
static struct cdev buzzer_cdev;
static void __iomem *gpio_base = NULL;
static int buzzer_state = 0;

static ssize_t buzzer_read(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    char msg[16];
    int msg_len;

    if (*offset > 0)
        return 0;

    msg_len = snprintf(msg, sizeof(msg), "%s\n", buzzer_state ? "ON" : "OFF");
    if (copy_to_user(buf, msg, msg_len))
        return -EFAULT;

    *offset = msg_len;
    return msg_len;
}

static ssize_t buzzer_write(struct file *file, const char __user *buf, size_t len, loff_t *offset) {
    char cmd[8] = {0};
    if (copy_from_user(cmd, buf, min(len, sizeof(cmd) - 1)))
        return -EFAULT;

    u32 val = ioread32(gpio_base + GPIO_DATAOUT);

    if (strncmp(cmd, "ON", 2) == 0) {
        iowrite32(val | BUZZER_MASK, gpio_base + GPIO_DATAOUT);
        buzzer_state = 1;
        pr_info("Buzzer ON\n");
    } else if (strncmp(cmd, "OFF", 3) == 0) {
        iowrite32(val & ~BUZZER_MASK, gpio_base + GPIO_DATAOUT);
        buzzer_state = 0;
        pr_info("Buzzer OFF\n");
    } else {
        pr_warn("Unknown command: %s\n", cmd);
    }

    return len;
}

static struct file_operations buzzer_fops = {
    .owner = THIS_MODULE,
    .read  = buzzer_read,
    .write = buzzer_write,
};

static int __init buzzer_init(void) {
    int ret;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;

    cdev_init(&buzzer_cdev, &buzzer_fops);
    ret = cdev_add(&buzzer_cdev, dev_num, 1);
    if (ret < 0) {
        unregister_chrdev_region(dev_num, 1);
        return ret;
    }

    buzzer_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(buzzer_class)) {
        cdev_del(&buzzer_cdev);
        unregister_chrdev_region(dev_num, 1);
        return PTR_ERR(buzzer_class);
    }

    if (IS_ERR(device_create(buzzer_class, NULL, dev_num, NULL, DEVICE_NAME))) {
        class_destroy(buzzer_class);
        cdev_del(&buzzer_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -1;
    }

    gpio_base = ioremap(GPIO1_BASE_ADDR, GPIO_SIZE);
    if (!gpio_base) {
        pr_err("Failed to ioremap GPIO1\n");
        device_destroy(buzzer_class, dev_num);
        class_destroy(buzzer_class);
        cdev_del(&buzzer_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -ENOMEM;
    }

    // Set GPIO as output
    u32 oe = ioread32(gpio_base + GPIO_OE);
    oe &= ~BUZZER_MASK;
    iowrite32(oe, gpio_base + GPIO_OE);

    // Turn OFF by default
    u32 val = ioread32(gpio_base + GPIO_DATAOUT);
    iowrite32(val & ~BUZZER_MASK, gpio_base + GPIO_DATAOUT);

    pr_info("Buzzer driver loaded (ioremap), GPIO1_28 (P9_12), major=%d\n", MAJOR(dev_num));
    return 0;
}

static void __exit buzzer_exit(void) {
    if (gpio_base) {
        // Turn off buzzer
        u32 val = ioread32(gpio_base + GPIO_DATAOUT);
        iowrite32(val & ~BUZZER_MASK, gpio_base + GPIO_DATAOUT);
        iounmap(gpio_base);
    }

    device_destroy(buzzer_class, dev_num);
    class_destroy(buzzer_class);
    cdev_del(&buzzer_cdev);
    unregister_chrdev_region(dev_num, 1);
    pr_info("Buzzer driver unloaded\n");
}

module_init(buzzer_init);
module_exit(buzzer_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("YourName");
MODULE_DESCRIPTION("Buzzer driver with ioremap (P9_12 GPIO1_28)");

