/**
 * @file lcd1602.c
 * @brief Functions and data related to the lcd1602 i2c lcd driver implementation
 *
 * 
 *
 * @student edit: Li-Huan Lu
 * @date 2025-11-22
 * @reference https://embetronicx.com/tutorials/linux/device-drivers/i2c-linux-device-driver-using-raspberry-pi/
 *            ChatGPT
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of.h>      // Device Tree support
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include "../button/device.h"

#define DRIVER_NAME "lcd1602"
#define LCD_ADDRESS 0x27

#define LCD_RS      0x01    // P0
#define LCD_RW      0x02    // P1 (always 0)
#define LCD_EN      0x04    // P2
#define LCD_BL      0x08    // P4

static struct class *dev_class;

// This function writes the data into the I2C client
static int lcd_i2c_write(struct i2c_client *client, uint8_t data)
{
    return i2c_master_send(client, &data, 1);
}

static void lcd_pulse_enable(struct i2c_client *client, uint8_t data)
{
    lcd_i2c_write(client, data | LCD_EN);
    udelay(1);
    lcd_i2c_write(client, data & ~LCD_EN);
    udelay(50);
}

static void lcd_send_nibble(struct i2c_client *client, uint8_t nibble, bool rs)
{
    uint8_t data = 0;

    // Upper nibble goes to PCF8574 P4 - P7
    data = (nibble & 0x0F) << 4;

    // RS=1 when writing data
    if (rs)
        data |= LCD_RS;
    
	data |= LCD_BL;
	
    // Pulse enable to latch
    lcd_pulse_enable(client, data);
}

static void lcd_send_byte(struct i2c_client *client, uint8_t value, bool rs)
{
    lcd_send_nibble(client, (value >> 4) & 0x0F, rs); // high nibble
    lcd_send_nibble(client, value & 0x0F, rs);        // low nibble
}

static int lcd1602_open(struct inode *inode, struct file *f)
{
    PDEBUG("open");
	
	struct lcd1602_dev *dev;

    dev = container_of(inode->i_cdev, struct lcd1602_dev, cdev);
    f->private_data = dev;

    return 0;
}

static ssize_t lcd1602_write(struct file *filp, const char __user *buf,
                            size_t count, loff_t *off)
{
	struct lcd1602_dev *dev = filp->private_data;
    char data[32];
	int i;
	
	if (count > sizeof(data))
        count = sizeof(data);

    if (copy_from_user(data, buf, count))
        return -EFAULT;
	
	for (i = 0; i < count; i++){
		lcd_send_byte(dev->client, data[i], true);
	}
	
	return count;
}

// File operation structure 
struct file_operations lcd1602_fops = {
    .owner =    THIS_MODULE,
	.open  =    lcd1602_open,
    //.read  =    lcd1602_read,
    .write =    lcd1602_write,
};


// This function sends the commands that need to used to Initialize the LCD.
static void LCD1602_Init(struct lcd1602_dev *dev)
{
	struct i2c_client *client = dev->client;
	
	lcd_send_nibble(client, 0x03, false);
	msleep(5);
	lcd_send_nibble(client, 0x03, false);
	msleep(5);
	lcd_send_nibble(client, 0x03, false);
	msleep(1);
	
	lcd_send_nibble(client, 0x02, false);
	msleep(1);
	
    lcd_send_byte(client, 0x28, false);  // Function Set 4-bit, 2-line
    lcd_send_byte(client, 0x08, false);  // Display off
    lcd_send_byte(client, 0x01, false);  // Clear display
    msleep(2);
    lcd_send_byte(client, 0x06, false);  // Entry mode set
    lcd_send_byte(client, 0x0C, false);  // Display on, cursor off
	msleep(2);
}

// Probe function
static int lcd1602_probe(struct i2c_client *client, const struct i2c_device_id *id);
{
    struct lcd1602_dev *dev;
    int ret = 0;

    PDEBUG("%s: probing lcd1602 device\n", DRIVER_NAME);
	PDEBUG("LCD1602 address = 0x%02x\n", client->addr);

    // Allocate driver dev
    dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->client = client;
    i2c_set_clientdata(client, dev);
    
	// Allocate char device number
    ret = alloc_chrdev_region(&dev->devt, 0, 1, DRIVER_NAME);
    if (ret)
        return ret;

	
	// Initialize cdev
	cdev_init(&dev->cdev, &lcd1602_fops);
    dev->cdev.owner = THIS_MODULE;

    ret = cdev_add (&dev->cdev, dev->devt, 1);
    if (ret) {
        printk(KERN_ERR "Error %d adding lcd1602 cdev", ret);
		goto err_chrdev;
    }
	
	// Creating struct class
	if (!dev_class) {
		if(IS_ERR(dev_class = class_create(THIS_MODULE,"lcd_class"))){
			printk(KERN_ERR "Cannot create the struct class\n");
			dev_class = NULL;
			ret = -1;
			goto err_cdev;
		}
	}
 
    // Creating device
    if(IS_ERR(device_create(dev_class, NULL, dev->devt, NULL, DRIVER_NAME))){
        printk(KERN_ERR "Cannot create the Device \n");
		ret = -EINVAL;
        goto err_device;
    }
    
	// LCD initialization sequence
    LCD1602_Init(dev);
	
	PDEBUG("%s: lcd1602 probe successful\n", DRIVER_NAME);
	return ret;	
	
err_device:
    device_destroy(dev_class, dev->devt);
err_cdev:
    cdev_del(&dev->cdev);
err_chrdev:
    unregister_chrdev_region(dev->devt, 1);
	
    return ret;
}

// Remove function
static int lcd1602_remove(struct i2c_client *client)
{
	struct lcd1602_dev *dev = i2c_get_clientdata(client);
	
	device_destroy(dev_class, dev->devt);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devt, 1);
	
	PDEBUG("%s: removing lcd1602 device\n", DRIVER_NAME);	
	
    return 0;
}

// Device Tree match table
static const struct of_device_id lcd1602_of_match[] = {
    { .compatible = "sunfounder,lcd1602" },
    {}
};
MODULE_DEVICE_TABLE(of, lcd1602_of_match);

// I2C driver struct
static struct i2c_driver lcd1602_driver = {
    .driver = {
        .name           = DRIVER_NAME,
		.owner          = THIS_MODULE,
        .of_match_table = lcd1602_of_match,
    },
    .probe     = lcd1602_probe,
    .remove    = lcd1602_remove,
	//.id_table  = lcd1602_id,
};
	
// Register driver, contains the functionalities of module_init() and module_exit()
module_i2c_driver(lcd1602_driver);

MODULE_AUTHOR("Li-Huan Lu");
MODULE_LICENSE("GPL");

