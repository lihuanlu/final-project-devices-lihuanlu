/**
 * @file button.c
 * @brief Functions and data related to the button driver implementation
 *
 * Modified from aesdchar main.c (@author Dan Walkes)
 *
 * @student edit: Li-Huan Lu
 * @date 2025-11-07
 * @reference https://embetronicx.com/tutorials/linux/device-drivers/gpio-driver-basic-using-raspberry-pi/
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/fs.h> // file_operations
#include <linux/gpio.h>
#include "device.h"
//#include "aesd_ioctl.h"
#define GPIO_16 (16) // select
#define GPIO_20 (20) // up
#define GPIO_21 (21) // down

int button_major =   0; // use dynamic major
int button_minor =   0;

MODULE_AUTHOR("Li-Huan Lu");
MODULE_LICENSE("Dual BSD/GPL");

struct button_dev button_device;
static struct class *dev_class;

int button_open(struct inode *inode, struct file *filp)
{
    PDEBUG("open");
    //handle open
	
	struct button_dev *dev; // device information
	
	dev = container_of(inode->i_cdev, struct button_dev, cdev);
	filp->private_data = dev;
	
    return 0;
}

int button_release(struct inode *inode, struct file *filp)
{
    PDEBUG("release");

    //handle release

    return 0;
}

ssize_t button_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
    ssize_t retval = 0;
    PDEBUG("read gpio value");

    uint8_t gpio_states = 0; // not pressed
    struct button_dev *dev = filp->private_data; 
	
    // reading GPIO value
	// button pressed
	if (gpio_get_value(GPIO_16) == 0)
		gpio_states |= 1 << 0;
    if (gpio_get_value(GPIO_20) == 0)
		gpio_states |= 1 << 1;
	if (gpio_get_value(GPIO_21) == 0)
		gpio_states |= 1 << 2;
	
	if (mutex_lock_interruptible(&dev->lock))
		return -ERESTARTSYS;
	
	if (count < 1) {
		retval = -EFAULT;
		goto out;
	}
	
	/*
	if (*f_pos > 0) { // Reach EOF
        retval = 0;
        goto out;
    }
	*/
	
	if (copy_to_user(buf, &gpio_states, 1)) {
		retval = -EFAULT;
		goto out;
	}
	
	retval = 1;
	*f_pos = 0;
	
out:
	mutex_unlock(&dev->lock);
    return retval;
}

// File operation structure 
struct file_operations button_fops = {
    .owner =    THIS_MODULE,
    .read =     button_read,
    .open =     button_open,
    .release =  button_release,
	//.unlocked_ioctl = button_ioctl,
};

static int button_setup_cdev(struct button_dev *dev)
{
    int err, devno = MKDEV(button_major, button_minor);

    cdev_init(&dev->cdev, &button_fops);
    dev->cdev.owner = THIS_MODULE;
    dev->cdev.ops = &button_fops;
    err = cdev_add (&dev->cdev, devno, 1);
    if (err) {
        printk(KERN_ERR "Error %d adding button cdev", err);
    }
    return err;
}


int button_init_module(void)
{
    dev_t dev = 0;
    int result;
    result = alloc_chrdev_region(&dev, button_minor, 1,
            "button");
    button_major = MAJOR(dev);
    if (result < 0) {
        printk(KERN_WARNING "Can't get major %d\n", button_major);
        return result;
    }
    memset(&button_device,0,sizeof(struct button_dev));

    // Initialization
    mutex_init(&button_device.lock); // initialize locking primitive
    result = button_setup_cdev(&button_device);

    if( result ) {
        unregister_chrdev_region(dev, 1);
		return result;
    }
	
	// Creating struct class
    if(IS_ERR(dev_class = class_create(THIS_MODULE,"button_class"))){
        printk(KERN_ERR "Cannot create the struct class\n");
		result = -1;
        goto r_class;
    }
 
    // Creating device
    if(IS_ERR(device_create(dev_class, NULL, dev, NULL, "button"))){
        printk(KERN_ERR "Cannot create the Device \n");
		result = -1;
        goto r_device;
    }
  
    //Checking the GPIOs are valid or not
    if(gpio_is_valid(GPIO_16) == false){
      printk(KERN_ERR "GPIO %d is not valid\n", GPIO_16);
	  result = -1;
      goto r_device;
    }
	if(gpio_is_valid(GPIO_20) == false){
      printk(KERN_ERR "GPIO %d is not valid\n", GPIO_20);
	  result = -1;
      goto r_device;
    }
	if(gpio_is_valid(GPIO_21) == false){
      printk(KERN_ERR "GPIO %d is not valid\n", GPIO_21);
	  result = -1;
      goto r_device;
    }
  
    // Requesting the GPIO
	result = gpio_request(GPIO_16, "GPIO_16");
    if(result < 0){
      printk(KERN_ERR "ERROR: GPIO %d request\n", GPIO_16);
      goto r_gpio;
    }
	result = gpio_request(GPIO_20, "GPIO_20");
    if(result < 0){
      printk(KERN_ERR "ERROR: GPIO %d request\n", GPIO_20);
      goto r_gpio;
    }
	result = gpio_request(GPIO_21, "GPIO_21");
    if(result < 0){
      printk(KERN_ERR "ERROR: GPIO %d request\n", GPIO_21);
      goto r_gpio;
    }
	
    // Make the GPIOs visible and direction cannot be changed by user
    gpiod_export(GPIO_16, false);
	gpiod_export(GPIO_20, false);
	gpiod_export(GPIO_21, false);
	
	// Configure the GPIOs as input
	gpio_direction_input(GPIO_16);
	gpio_direction_input(GPIO_20);
	gpio_direction_input(GPIO_21);
	
    return result;
	
r_gpio:
    gpio_free(GPIO_16);
	gpio_free(GPIO_20);
	gpio_free(GPIO_21);
r_device:
    device_destroy(dev_class, dev);
r_class:
    class_destroy(dev_class);
  
    return result;
}

void button_cleanup_module(void)
{
    dev_t devno = MKDEV(button_major, button_minor);
    
	gpiod_unexport(GPIO_16);
	gpiod_unexport(GPIO_20);
	gpiod_unexport(GPIO_21);
    gpio_free(GPIO_16);
	gpio_free(GPIO_20);
	gpio_free(GPIO_21);
    device_destroy(dev_class, devno);
    class_destroy(dev_class);
    cdev_del(&button_device.cdev);
    unregister_chrdev_region(devno, 1);
}


module_init(button_init_module);
module_exit(button_cleanup_module);
