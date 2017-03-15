/*
 * linux/drivers/char/caddymotor.c
 *
 * file interface for Opticorder caddy motors
 * Copyright (c) Chris Jones, Martin-Jones Technology Ltd 2017
 *
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/poll.h>

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

define CADDYMOTOR_BUF_SIZE 100
#define CADDYMOTOR_POLL_INTERVAL (HZ/10)

/* character device start number */
static dev_t dev;

static spinlock_t caddymotor_lock;

/* queue for timer callback */
static DECLARE_WAIT_QUEUE_HEAD(tickq);

static void caddymotor_timer_callback(unsigned long arg);

static struct timer_list caddytimer = TIMER_INITIALIZER(caddymotor_timer_callback, 0L, 0);

/* caddymotor per device data */
static struct caddymotor_dev {
	struct fasync_struct *async_queue;
	char *buffer, *end;
	int buffersize;
	char *readp;
	char *dataend;
	struct cdev cdev;
	int nreaders;
	/* records of values from the I/O ports */
} caddymotor;

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
	//val=samosa_read8(reg);
	val = 0;
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
		//samosa_write8(reg, samosa_data);
	}
	else
		pr_info("cannot parse data from <%s> reg 0x%02x\n",p,reg);
	return count;
}

/* file access functions */
static ssize_t	caddymotor_read(struct file *, char *, size_t, loff_t *);
static ssize_t	caddymotor_write(struct file *, const char *, size_t, loff_t *);
static int	caddymotor_open(struct inode *, struct file *);
static int	caddymotor_release(struct inode *, struct file *);
static int caddymotor_fasync(int fd, struct file *filp, int on);
static int	caddymotor_mmap(struct file *, struct vm_area_struct *vm);

/* poll function so select() works */
unsigned int caddymotor_poll(struct file *filp, poll_table *wait);

static struct file_operations caddymotor_fops = {
	read:		caddymotor_read,
	write:		caddymotor_write,
	open:		caddymotor_open,
	release:	caddymotor_release,
	mmap:		caddymotor_mmap,
	poll:		caddymotor_poll,
	fasync: caddymotor_fasync,
};

/* proc interface */
static ssize_t proc_read_caddymotor(struct file *file, char *buf,
		size_t nbytes, loff_t *ppos);
static ssize_t proc_write_caddymotor(struct file *file, const char *buffer,
		size_t count, loff_t *ppos);

static struct file_operations proc_caddymotor_operations = {
	read:	proc_read_caddymotor,
	write:	proc_write_caddymotor
};
static struct proc_dir_entry *proc_caddymotor;
#define PROC_CADDYMOTOR "caddymotor"

static char * caddymotor_hex_to_buf(char *buf, int buflen, uint8_t addr, uint8_t val) {
	char localbuf[6];
	sprintf(localbuf,"%02x:%02x\n",addr,val);
	return strcat(buf,localbuf);
}

static void caddymotor_read_regs(struct caddymotor_dev *devp) {
	*(devp->buffer)='\0';
	devp->dataend=devp->buffer+strlen(devp->buffer);
	devp->readp=devp->buffer;
}

static void caddymotor_timer_callback(unsigned long arg) {
	struct caddymotor_dev *devp = (struct caddymotor_dev *)arg;
	//printk(KERN_WARNING "caddymotor_timer_callback\n");

	wake_up_interruptible(&tickq);
	mod_timer(&caddytimer,jiffies+CADDYMOTOR_POLL_INTERVAL);
}

static int caddymotor_open(struct inode *inode, struct file *filp)
{
	struct caddymotor_dev *devp;

	devp = container_of(inode->i_cdev, struct caddymotor_dev, cdev);
	filp->private_data = devp;

	if(!devp->buffer) {
		devp->buffer = kmalloc(CADDYMOTOR_BUF_SIZE, GFP_KERNEL);
		if(!devp->buffer)
			return -ENOMEM;
	}
	devp->buffersize=CADDYMOTOR_BUF_SIZE;
	devp->end = devp->buffer + devp->buffersize;

	caddymotor_read_regs(devp);

	if(filp->f_mode & FMODE_READ)
		devp->nreaders++;

	// set timer to fire off in a second's time to check for changes
	caddytimer.data = (unsigned long)devp;
	mod_timer(&caddytimer,jiffies+CADDYMOTOR_POLL_INTERVAL);

	return 0;
}

static int caddymotor_release(struct inode *inode, struct file *filp)
{
	struct caddymotor_dev *devp = filp->private_data;
	// cancel timer
	del_timer(&caddytimer);

	if(filp->f_mode & FMODE_READ)
		devp->nreaders--;

	if(devp->nreaders == 0) {
		kfree(devp->buffer);
		devp->buffer=NULL;
	}
	return 0;
}

static ssize_t	caddymotor_read(struct file *filp, char __user *buf,
		 size_t size, loff_t *offp)
{
	struct caddymotor_dev *devp = (struct caddymotor_dev *)filp->private_data;
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

static ssize_t	caddymotor_write(struct file *filp, const char *buf,
		  size_t size, loff_t *offp)
{
	/* we don't support writing at the moment */
	return -EINVAL;
}

static int caddymotor_fasync(int fd, struct file *filp, int on)
{
	struct caddymotor_dev *devp = filp->private_data;

	return fasync_helper(fd, filp, on, &devp->async_queue);
}

static int caddymotor_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/*struct caddymotor_dev *mp = (struct caddymotor_dev *)filp->private_data;*/

	return -EINVAL;
}

unsigned int caddymotor_poll(struct file *filp, poll_table *wait) {
	struct caddymotor_dev *devp;
	unsigned int mask;

	devp = filp->private_data;
	mask = 0;

	//printk(KERN_WARNING "caddymotor_poll\n");

	poll_wait(filp, &tickq, wait);

	if(devp->readp < devp->dataend) {
		mask |= POLLIN | POLLRDNORM;
		if(devp->async_queue)
			kill_fasync(&devp->async_queue,SIGIO,POLL_IN);
	}

	return mask;
}

static int proc_read_caddymotor(struct file *filp, char *buf,
		size_t nbytes, loff_t *ppos)
{
	char outputbuf[512];
	int count = 0;

	/* all done in a single read */
	if (*ppos > 0)
		return 0;

	count += sprintf(&outputbuf[count], "caddymotor: major dev = %d\n", MAJOR(dev));

	if (count > nbytes)  /* Assume output can be read at one time */
		return -EINVAL;
	if (copy_to_user(buf, outputbuf, count))
		return -EFAULT;
	*ppos += count;
	return count;
}

static ssize_t proc_write_caddymotor(struct file *filp, const char *buffer,
		size_t count, loff_t *ppos)
{
	/* struct caddymotor_dev *mp= (struct caddymotor_dev *)filp->private_data;
	 * if (strncmp(buff,"reset:",6)==0)
	 * newRegValue = simple_strtoul(buffer,&endp,0);
	 * a bold but simple claim is to have read it all
	 */
	return count;
}


/* driver initialisation */

static int __init caddymotor_probe(struct platform_device *pdev)
{

	dev_info(&pdev->dev, "Opticorder caddy motor support installed\n");

	return 0;
}

static int __exit caddymotor_remove(struct platform_device *dev)
{
	platform_set_drvdata(dev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int caddymotor_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int caddymotor_resume(struct platform_device *dev)
{
	return 0;
}
#else /* CONFIG_PM */
#define caddymotor_suspend	NULL
#define caddymotor_resume	NULL
#endif /* CONFIG_PM */

#define caddymotor_shutdown	NULL

/* driver definition */
static struct platform_driver caddymotor_driver = {
	.probe		= caddymotor_probe,
	.shutdown	= caddymotor_shutdown,
	.remove		= __exit_p(caddymotor_remove),
	.suspend	= caddymotor_suspend,
	.resume		= caddymotor_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "caddymotor",
	},
};

/* bus device */
static struct platform_device *caddymotor_device;

/* class object */
static struct class *caddymotor_class;

static int __init caddymotor_init(void)
{
	int ret;

	/* general initialisation */
	spin_lock_init(&caddymotor_lock);

	/* register a range of device nodes */
	ret = alloc_chrdev_region(&dev, 0, 1, "caddymotor");
	if (ret)
		goto error;

	/* create the caddymotor character device */
	/* initialise character device */
	cdev_init(&caddymotor.cdev, &caddymotor_fops);
	/* claim ownership */
	caddymotor.cdev.owner = THIS_MODULE;
	/* add character device */
	ret = cdev_add(&caddymotor.cdev, dev, 1);
	if (ret) {
		goto error_region;
	}

	/* create the class and devices */
	caddymotor_class = class_create(THIS_MODULE, "caddymotor");

	if (!device_create(caddymotor_class, NULL, (dev), NULL, "caddymotor"))
		goto error_class_device;
	/* register the device on a bus. */
	caddymotor_device=platform_device_alloc("caddymotor",0);
	if(!caddymotor_device)
		goto error_bus;
	if(platform_device_add(caddymotor_device))
		goto error_bus;

	/* create proc access*/
	proc_caddymotor = create_proc_entry(PROC_CADDYMOTOR, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH, NULL);
	if (proc_caddymotor)
		proc_caddymotor->proc_fops = &proc_caddymotor_operations;

	/* register the driver */
	return platform_driver_register(&caddymotor_driver);

error_bus:
	/* remove platform device */
	platform_device_unregister(caddymotor_device);
error_class_device:
	device_destroy(caddymotor_class, dev);
error_region:
	unregister_chrdev_region(dev, 1);
error:
	return ret;
}

static void __exit caddymotor_exit(void)
{

	/* remove proc entry */
	remove_proc_entry(PROC_CADDYMOTOR, NULL);

	/* remove class device */
	device_destroy(caddymotor_class, (dev));

	/* remove platform device */
	platform_device_unregister(caddymotor_device);

	/* remove character device */
	cdev_del(&caddymotor.cdev);
	/* remove driver */
	platform_driver_unregister(&caddymotor_driver);

	/* unregister region */
	unregister_chrdev_region(dev, 1);

	/* remove class */
	class_destroy(caddymotor_class);
}

module_init(caddymotor_init);
module_exit(caddymotor_exit);

MODULE_AUTHOR("Chris Jones <chris@martin-jones.com>");
MODULE_DESCRIPTION("Opticorder caddy motor driver");
MODULE_LICENSE("GPL");
