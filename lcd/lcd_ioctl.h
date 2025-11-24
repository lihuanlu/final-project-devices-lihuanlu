/*
 * lcd_ioctl.h
 *
 *  Modified from aesd_ioctl.h
 *
 *  @brief Definitins for the ioctl used on lcd device.
 */

#ifndef LCD_IOCTL_H
#define LCD_IOCTL_H

#ifdef __KERNEL__
#include <asm-generic/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

/**
 * A structure to be passed by IOCTL from user space to kernel space, describing the type
 * of seek performed on the aesdchar driver
 */
struct lcd_setcursor {
    uint8_t row; // 0 or 1
    uint8_t col; // 0 to 15
};

// Pick an arbitrary unused value from https://github.com/torvalds/linux/blob/master/Documentation/userspace-api/ioctl/ioctl-number.rst
#define LCD_IOC_MAGIC 0x16

// Define a write command from the user point of view, use command number 1
#define LCD_IOCSETCURSOR _IOW(LCD_IOC_MAGIC, 1, struct lcd_setcursor)
/**
 * The maximum number of commands supported, used for bounds checking
 */
#define LCD_IOC_MAXNR 1

#endif /* LCD_IOCTL_H */
