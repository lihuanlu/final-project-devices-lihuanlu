/**
 * @file ds1722.c
 * @brief Functions and data related to the ds1722 spi temperature sensor driver implementation
 *
 * 
 *
 * @student edit: Li-Huan Lu
 * @date 2025-11-15
 * @reference https://embetronicx.com/tutorials/linux/device-drivers/linux-kernel-spi-device-driver-tutorial/
 *            ChatGPT
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/of.h>      // Device Tree support
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/cdev.h>
#include "../button/device.h"

#define DRIVER_NAME "ds1722"
#define WR_CONFIG (0x80)
#define RD_CONFIG (0x00)
#define TEMP_LSB  (0x01)
#define TEMP_MSB  (0x02)
#define DUMMY     (0x00)

static struct class *dev_class;

static int ds1722_open(struct inode *inode, struct file *f)
{
    PDEBUG("open");
	
	struct ds1722_dev *dev;

    dev = container_of(inode->i_cdev, struct ds1722_dev, cdev);
    f->private_data = dev;

    return 0;
}

static int ds1722_spi_read(struct spi_device *spi, uint8_t reg, uint8_t *val)
{
    int ret;
    uint8_t tx[2] = {reg, DUMMY}; // send address, then dummy byte
    uint8_t rx[2] = {0};

    struct spi_transfer xfers[] = {
        {
            .tx_buf = tx,
            .len = 2,
            .rx_buf = rx,
        }
    };

    ret = spi_sync_transfer(spi, xfers, 1);
    if (ret < 0)
        return ret;

    *val = rx[1]; // temperature value in second byte
    return 0;	
}

static int ds1722_spi_write_config(struct spi_device *spi, uint8_t config)
{
    uint8_t tx[2] = {WR_CONFIG, config }; // write to config register
    struct spi_transfer xfer = {
        .tx_buf = tx,
        .len = 2,
    };

    return spi_sync_transfer(spi, &xfer, 1);
}

static ssize_t ds1722_read(struct file *filp, char __user *buf,
                           size_t count, loff_t *off)
{
    ssize_t retval = 0;
	uint8_t byte_read[2] = {0};
	
	struct ds1722_dev *dev = filp->private_data; 
	
	retval = ds1722_spi_read(dev->spi, TEMP_MSB, &byte_read[0]);
	if (retval < 0)
		return retval;
	retval = ds1722_spi_read(dev->spi, TEMP_LSB, &byte_read[1]);
	if (retval < 0)
		return retval;
	
	if (copy_to_user(buf, byte_read, 2)) {
		retval = -EFAULT;
		return retval;
	}
	
	// success
	retval = 2;
	
	return retval;
}

static ssize_t ds1722_write(struct file *filp, const char __user *buf,
                            size_t count, loff_t *off)
{
	struct ds1722_dev *dev = filp->private_data;
    uint8_t config;
	
	if (count < 1)
        return -EINVAL;

    if (copy_from_user(&config, buf, 1))
        return -EFAULT;
	
	if (ds1722_spi_write_config(dev->spi, config) < 0)
        return -EIO;
	
	return 1;
}

// File operation structure 
struct file_operations ds1722_fops = {
    .owner =    THIS_MODULE,
	.open  =    ds1722_open,
    .read  =    ds1722_read,
    .write =    ds1722_write,
};

// Probe function
static int ds1722_probe(struct spi_device *spi)
{
    struct ds1722_dev *dev;
    int ret = 0;

	
    PDEBUG("%s: probing DS1722 device\n", DRIVER_NAME);

    // Allocate driver dev
    dev = devm_kzalloc(&spi->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->spi = spi;
    spi_set_drvdata(spi, dev);
    
	// Allocate char device number
    ret = alloc_chrdev_region(&dev->devt, 0, 1, DRIVER_NAME);
    if (ret)
        return ret;

	
	// Initialize cdev
	cdev_init(&dev->cdev, &ds1722_fops);
    dev->cdev.owner = THIS_MODULE;

    ret = cdev_add (&dev->cdev, dev->devt, 1);
    if (ret) {
        printk(KERN_ERR "Error %d adding ds1722 cdev", ret);
		goto err_chrdev;
    }
	
	// Creating struct class
	if (!dev_class) {
		if(IS_ERR(dev_class = class_create(THIS_MODULE,"sensor_class"))){
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
    
	PDEBUG("%s: ds1722 probe successful\n", DRIVER_NAME);
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
static int ds1722_remove(struct spi_device *spi)
{
	struct ds1722_dev *dev = spi_get_drvdata(spi);
	
	device_destroy(dev_class, dev->devt);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devt, 1);
	
	PDEBUG("%s: removing DS1722 device\n", DRIVER_NAME);	
	
    return 0;
}

// Device Tree match table
static const struct of_device_id ds1722_of_match[] = {
    { .compatible = "maxim,ds1722" },
    {}
};
MODULE_DEVICE_TABLE(of, ds1722_of_match);

// SPI driver struct
static struct spi_driver ds1722_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = ds1722_of_match,
    },
    .probe = ds1722_probe,
    .remove = ds1722_remove,
};

// Register driver
module_spi_driver(ds1722_driver);

MODULE_AUTHOR("Li-Huan Lu");
MODULE_LICENSE("GPL");

