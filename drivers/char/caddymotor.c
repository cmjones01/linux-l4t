/*
 * linux/drivers/char/podpoint.c
 *
 * file interface for Podpoint hardware on the balloon samosa bus
 * Copyright (c) Chris Jones, Martin-Jones Technology Ltd 2015
 *
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/poll.h>

#if defined(CONFIG_MACH_BALLOON2)
#include <mach/balloon2.h>
#endif
#if defined(CONFIG_MACH_BALLOON3)
#include <mach/balloon3.h>
#endif
#include <asm/cacheflush.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include "linux/cdev.h"
#include <linux/samosa_device.h>
#include "charset.h"

#define SAMOSA_ADDR_DOORS 0x05
#define SAMOSA_ADDR_IO 0x06
/* mask to remove useless and annoying GSM_SYNC bit */
#define SAMOSA_IO_MASK 0xfd
#define SAMOSA_ADDR_MODE3_LEFT 0x41
#define SAMOSA_ADDR_MODE3_RIGHT 0x51
#define POD_BUF_SIZE 100
#define POD_POLL_INTERVAL (HZ/10)

/* character device start number */
static dev_t dev;

static spinlock_t podpoint_lock;

/* queue for timer callback */
static DECLARE_WAIT_QUEUE_HEAD(tickq);

static void podpoint_timer_callback(unsigned long arg);

static struct timer_list podtimer = TIMER_INITIALIZER(podpoint_timer_callback, 0L, 0);

/* podpoint per device data */
static struct podpoint_dev {
	struct fasync_struct *async_queue;
	char *buffer, *end;
	int buffersize;
	char *readp;
	char *dataend;
	struct cdev cdev;
	int nreaders;
	/* records of values from the I/O ports */
	uint8_t doors_value, old_doors_value;
	uint8_t io_value, old_io_value;
	uint8_t mode3_left_value, old_mode3_left_value;
	uint8_t mode3_right_value, old_mode3_right_value;
} podpoint;

/* SAMOSA access/parsing functions */

static ssize_t read_hex(unsigned char val, struct file *file, char *buf, size_t nbytes, loff_t *ppos);
static ssize_t read_samosa_hex(unsigned char reg, struct file *file, char *buf, size_t nbytes, loff_t *ppos);
static ssize_t write_samosa_hex(unsigned char reg, struct file *file, const char *buffer,	size_t count, loff_t *ppos);

static ssize_t read_hex(unsigned char val, struct file *file, char *buf,
		size_t nbytes, loff_t *ppos) {
	char outputbuf[4];
	size_t bytes_to_write;
	size_t written=0;
	unsigned long unwritten;

	bytes_to_write = (nbytes>3)?3:nbytes;

	printk(KERN_WARNING "read_hex nbytes %d ppos %d bytes_to_write %d\n",nbytes,ppos,bytes_to_write);
	if(!bytes_to_write)
		return 0;

	if(*ppos>0)
		return 0;
	sprintf(outputbuf,"%02x\n",val);

	unwritten = copy_to_user(buf,outputbuf,bytes_to_write);
	written += bytes_to_write - unwritten;

	if (signal_pending(current))
		return written ? written : -ERESTARTSYS;
	//*ppos+=bytes_to_write;
	cond_resched();

	return written ? written : -EFAULT;
}

static ssize_t read_samosa_hex(unsigned char reg, struct file *file, char *buf,
		size_t nbytes, loff_t *ppos) {
	char val;
	char outputbuf[3];

	if(*ppos>0)
		return 0;
	val=samosa_read8(reg);
	sprintf(outputbuf,"%02x",val);

	if(copy_to_user(buf,outputbuf,2))
		return -EFAULT;
	*ppos+=2;
	return 2;
}

static ssize_t write_samosa_hex(unsigned char reg, struct file *file, const char *buffer,
		size_t count, loff_t *ppos) {
	unsigned long samosa_data;
	char buf[40];
	char *p = buf;
	char *pp;

	if (count >= (sizeof(buf) -1 ))
		return -EFAULT;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = 0;
	while (isspace(*p))
		p++;

	samosa_data = simple_strtoul(p,&pp,0);
	if (pp && (pp > p)) {
		samosa_write8(reg, samosa_data);
	}
	else
		pr_info("cannot parse data from <%s> reg 0x%02x\n",p,reg);
	return count;
}

/* file access functions */
static ssize_t	podpoint_read(struct file *, char *, size_t, loff_t *);
static ssize_t	podpoint_write(struct file *, const char *, size_t, loff_t *);
static int	podpoint_open(struct inode *, struct file *);
static int	podpoint_release(struct inode *, struct file *);
static int podpoint_fasync(int fd, struct file *filp, int on);
static int	podpoint_mmap(struct file *, struct vm_area_struct *vm);

/* poll function so select() works */
unsigned int podpoint_poll(struct file *filp, poll_table *wait);

static struct file_operations podpoint_fops = {
	read:		podpoint_read,
	write:		podpoint_write,
	open:		podpoint_open,
	release:	podpoint_release,
	mmap:		podpoint_mmap,
	poll:		podpoint_poll,
	fasync: podpoint_fasync,
};

/* proc interface */
static ssize_t proc_read_podpoint(struct file *file, char *buf,
		size_t nbytes, loff_t *ppos);
static ssize_t proc_write_podpoint(struct file *file, const char *buffer,
		size_t count, loff_t *ppos);

static struct file_operations proc_podpoint_operations = {
	read:	proc_read_podpoint,
	write:	proc_write_podpoint
};
static struct proc_dir_entry *proc_podpoint;
#define PROC_PODPOINT "podpoint"

static char * podpoint_hex_to_buf(char *buf, int buflen, uint8_t addr, uint8_t val) {
	char localbuf[6];
	sprintf(localbuf,"%02x:%02x\n",addr,val);
	return strcat(buf,localbuf);
}

static void podpoint_read_regs(struct podpoint_dev *devp) {
	devp->old_doors_value=devp->doors_value=samosa_read8(SAMOSA_ADDR_DOORS);
	devp->old_io_value=devp->io_value=samosa_read8(SAMOSA_ADDR_IO) & SAMOSA_IO_MASK;
	devp->old_mode3_left_value=devp->mode3_left_value=samosa_read8(SAMOSA_ADDR_MODE3_LEFT);
	devp->old_mode3_right_value=devp->mode3_right_value=samosa_read8(SAMOSA_ADDR_MODE3_RIGHT);
	*(devp->buffer)='\0';
	podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_DOORS,devp->doors_value);
	podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_IO,devp->io_value);
	podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_MODE3_LEFT,devp->mode3_left_value);
	podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_MODE3_RIGHT,devp->mode3_right_value);
	devp->dataend=devp->buffer+strlen(devp->buffer);
	devp->readp=devp->buffer;
}

static void podpoint_read_regs_changed(struct podpoint_dev *devp) {
	devp->doors_value=samosa_read8(SAMOSA_ADDR_DOORS);
	devp->io_value=samosa_read8(SAMOSA_ADDR_IO) & SAMOSA_IO_MASK;
	devp->mode3_left_value=samosa_read8(SAMOSA_ADDR_MODE3_LEFT);
	devp->mode3_right_value=samosa_read8(SAMOSA_ADDR_MODE3_RIGHT);
	if(devp->doors_value != devp->old_doors_value ||
		devp->io_value != devp->old_io_value ||
		devp->mode3_left_value != devp->old_mode3_left_value ||
		devp->mode3_right_value != devp->old_mode3_right_value) {
		*(devp->buffer)='\0';
		if(devp->doors_value!=devp->old_doors_value) {
			podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_DOORS,devp->doors_value);
			devp->old_doors_value=devp->doors_value;
		}
		if(devp->io_value!=devp->old_io_value) {
			podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_IO,devp->io_value);
			devp->old_io_value=devp->io_value;
		}
		if(devp->mode3_left_value!=devp->old_mode3_left_value) {
			podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_MODE3_LEFT,devp->mode3_left_value);
			devp->old_mode3_left_value=devp->mode3_left_value;
		}
		if(devp->mode3_right_value!=devp->old_mode3_right_value) {
			podpoint_hex_to_buf(devp->buffer,POD_BUF_SIZE,SAMOSA_ADDR_MODE3_RIGHT,devp->mode3_right_value);
			devp->old_mode3_right_value=devp->mode3_right_value;
		}
		devp->dataend=devp->buffer+strlen(devp->buffer);
		devp->readp=devp->buffer;
	}
}

static void podpoint_timer_callback(unsigned long arg) {
	struct podpoint_dev *devp = (struct podpoint_dev *)arg;
	//printk(KERN_WARNING "podpoint_timer_callback\n");
	podpoint_read_regs_changed(devp);
	wake_up_interruptible(&tickq);
	mod_timer(&podtimer,jiffies+POD_POLL_INTERVAL);
}

static int podpoint_open(struct inode *inode, struct file *filp)
{
	struct podpoint_dev *devp;

	if (samosa_sm_present())
		return -ENODEV;

	devp = container_of(inode->i_cdev, struct podpoint_dev, cdev);
	filp->private_data = devp;

	if(!devp->buffer) {
		devp->buffer = kmalloc(POD_BUF_SIZE, GFP_KERNEL);
		if(!devp->buffer)
			return -ENOMEM;
	}
	devp->buffersize=POD_BUF_SIZE;
	devp->end = devp->buffer + devp->buffersize;

	podpoint_read_regs(devp);

	if(filp->f_mode & FMODE_READ)
		devp->nreaders++;

	// set timer to fire off in a second's time to check for changes
	podtimer.data = (unsigned long)devp;
	mod_timer(&podtimer,jiffies+POD_POLL_INTERVAL);

	return 0;
}

static int podpoint_release(struct inode *inode, struct file *filp)
{
	struct podpoint_dev *devp = filp->private_data;
	// cancel timer
	del_timer(&podtimer);

	if(filp->f_mode & FMODE_READ)
		devp->nreaders--;

	if(devp->nreaders == 0) {
		kfree(devp->buffer);
		devp->buffer=NULL;
	}
	return 0;
}

static ssize_t	podpoint_read(struct file *filp, char __user *buf,
		 size_t size, loff_t *offp)
{
	struct podpoint_dev *devp = (struct podpoint_dev *)filp->private_data;
	size_t written;
	unsigned long unwritten;
	unsigned long bytes_in_buffer = devp->dataend - devp->readp;

	if(devp->readp == devp->dataend) {
		if(wait_event_interruptible(tickq, devp->readp != devp->dataend))
			return -ERESTARTSYS;
	}

	bytes_in_buffer = devp->dataend - devp->readp;
	if(bytes_in_buffer<size)
		size = bytes_in_buffer;

	if (!access_ok(VERIFY_WRITE, buf, size))
		return -EFAULT;

	written = 0;

	unwritten = copy_to_user(buf,devp->readp,size);
	written += size - unwritten;
	if (unwritten)
		goto failure;
	if (signal_pending(current))
		return written ? written : -ERESTARTSYS;
	buf += size;
	devp->readp += size;
//		cond_resched();

failure:
	return written ? written : -EFAULT;
}

static ssize_t	podpoint_write(struct file *filp, const char *buf,
		  size_t size, loff_t *offp)
{
	/* we don't support writing at the moment */
	return -EINVAL;
}

static int podpoint_fasync(int fd, struct file *filp, int on)
{
	struct podpoint_dev *devp = filp->private_data;

	return fasync_helper(fd, filp, on, &devp->async_queue);
}

static int podpoint_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/*struct podpoint_dev *mp = (struct podpoint_dev *)filp->private_data;*/

	return -EINVAL;
}

unsigned int podpoint_poll(struct file *filp, poll_table *wait) {
	struct podpoint_dev *devp;
	unsigned int mask;

	devp = filp->private_data;
	mask = 0;

	//printk(KERN_WARNING "podpoint_poll\n");

	poll_wait(filp, &tickq, wait);

	if(devp->readp < devp->dataend) {
		mask |= POLLIN | POLLRDNORM;
		if(devp->async_queue)
			kill_fasync(&devp->async_queue,SIGIO,POLL_IN);
	}

	return mask;
}

static int proc_read_podpoint(struct file *filp, char *buf,
		size_t nbytes, loff_t *ppos)
{
	char outputbuf[512];
	int count = 0;

	/* all done in a single read */
	if (*ppos > 0)
		return 0;

	count += sprintf(&outputbuf[count], "podpoint: major dev = %d\n", MAJOR(dev));

	if (count > nbytes)  /* Assume output can be read at one time */
		return -EINVAL;
	if (copy_to_user(buf, outputbuf, count))
		return -EFAULT;
	*ppos += count;
	return count;
}

static ssize_t proc_write_podpoint(struct file *filp, const char *buffer,
		size_t count, loff_t *ppos)
{
	/* struct podpoint_dev *mp= (struct podpoint_dev *)filp->private_data;
	 * if (strncmp(buff,"reset:",6)==0)
	 * newRegValue = simple_strtoul(buffer,&endp,0);
	 * a bold but simple claim is to have read it all
	 */
	return count;
}


/* driver initialisation */

static int __init podpoint_probe(struct samosa_device *pdev)
{
#ifndef CONFIG_BALLOON2_BUILD_TCL_PIKEY2
	/* if smart media present - cpld cannot be so declare invalid */
	if (samosa_sm_present()) {
		dev_info(&pdev->dev, "%s: samosa bus not present\n", __func__);
	    return -ENODEV;
	}
#endif

	dev_info(&pdev->dev, "POD Point support installed\n");

	return 0;
}

static int __exit podpoint_remove(struct samosa_device *dev)
{
	samosa_set_drvdata(dev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int podpoint_suspend(struct samosa_device *dev, pm_message_t state)
{
	/*MinipugPowerSave(dev->id);*/
	return 0;
}

static int podpoint_resume(struct samosa_device *dev)
{
	/*MinipugExitPowerSave(dev->id);*/
	return 0;
}
#else /* CONFIG_PM */
#define podpoint_suspend	NULL
#define podpoint_resume	NULL
#endif /* CONFIG_PM */

#define podpoint_shutdown	NULL

/* driver definition */
static struct samosa_driver podpoint_driver = {
	.probe		= podpoint_probe,
	.shutdown	= podpoint_shutdown,
	.remove		= __exit_p(podpoint_remove),
	.suspend	= podpoint_suspend,
	.resume		= podpoint_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "podpoint",
	},
};

/* bus device */
static struct samosa_device *podpoint_device;

/* class object */
static struct class *podpoint_class;

static int __init podpoint_init(void)
{
	int ret;

	/* general initialisation */
	spin_lock_init(&podpoint_lock);

	/* register a range of device nodes */
	ret = alloc_chrdev_region(&dev, 0, 1, "podpoint");
	if (ret)
		goto error;

	/* create the podpoint character device */
	/* initialise character device */
	cdev_init(&podpoint.cdev, &podpoint_fops);
	/* claim ownership */
	podpoint.cdev.owner = THIS_MODULE;
	/* add character device */
	ret = cdev_add(&podpoint.cdev, dev, 1);
	if (ret) {
		goto error_region;
	}

	/* create the class and devices */
	podpoint_class = class_create(THIS_MODULE, "podpoint");

	if (!device_create(podpoint_class, NULL, (dev), NULL, "podpoint"))
		goto error_class_device;
	/* register the device on a bus. */
	podpoint_device=samosa_device_alloc("podpoint",0);
	if(!podpoint_device)
		goto error_bus;
	if(samosa_device_add(podpoint_device))
		goto error_bus;

	/* create proc access*/
	proc_podpoint = create_proc_entry(PROC_PODPOINT, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH, NULL);
	if (proc_podpoint)
		proc_podpoint->proc_fops = &proc_podpoint_operations;

	/* register the driver */
	return samosa_driver_register(&podpoint_driver);

error_bus:
	/* remove samosa device */
	samosa_device_unregister(podpoint_device);
error_class_device:
	device_destroy(podpoint_class, dev);
error_region:
	unregister_chrdev_region(dev, 1);
error:
	return ret;
}

static void __exit podpoint_exit(void)
{

	/* remove proc entry */
	remove_proc_entry(PROC_PODPOINT, NULL);

	/* remove class device */
	device_destroy(podpoint_class, (dev));

	/* remove samosa device */
	samosa_device_unregister(podpoint_device);

	/* remove character device */
	cdev_del(&podpoint.cdev);
	/* remove driver */
	samosa_driver_unregister(&podpoint_driver);

	/* unregister region */
	unregister_chrdev_region(dev, 1);

	/* remove class */
	class_destroy(podpoint_class);
}

module_init(podpoint_init);
module_exit(podpoint_exit);

MODULE_AUTHOR("Chris Jones <chris@martin-jones.com>");
MODULE_DESCRIPTION("POD Point interface via samosa bus on Balloon");
MODULE_LICENSE("GPL");
