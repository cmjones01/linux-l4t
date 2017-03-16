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

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include "linux/cdev.h"

#define CADDYMOTOR_BUF_SIZE 100
#define CADDYMOTOR_POLL_INTERVAL (HZ/100)
#define NUM_MOTORS 2

/* character device start number */
static dev_t dev;

static spinlock_t caddymotor_lock;

/* queue for timer callback */
static DECLARE_WAIT_QUEUE_HEAD(tickq);

static void caddymotor_timer_callback(unsigned long arg);

static struct timer_list caddytimer = TIMER_INITIALIZER(caddymotor_timer_callback, 0L, 0);

//static  struct of_device_id caddymotor_of_match[] __initdata = {
static  struct of_device_id caddymotor_of_match[] = {
  { .compatible = "cerebriam,caddymotor", },
  {}
};

struct caddymotorstate_t {
	/* GPIO pins */
	int gpio_clk;
	int gpio_en;
	int gpio_dir;
	int gpio_reset;
	int gpio_half_full;
	int gpio_control;
	/* state */
	volatile int speed;
	volatile int distance;
};

static struct caddymotorstate_t motors[NUM_MOTORS];

/* caddymotor per device data */
static struct caddymotor_dev {
	struct fasync_struct *async_queue;
	char *buffer, *end;
	int buffersize;
	char *readp;
	char *dataend;
	struct cdev cdev;
	int nreaders;
	/* motors */
	int num_motors;
} caddymotor;

/* class object */
static struct class *caddymotor_class;

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

static struct proc_dir_entry *proc_file_entry;
static const struct file_operations proc_caddymotor_operations = {
	.owner = THIS_MODULE,
	.read = proc_read_caddymotor,
};

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
	int speed = motors[0].speed;
	int distance = motors[0].distance;
	//struct caddymotor_dev *devp = (struct caddymotor_dev *)arg;
	if(speed != 0) {
		gpio_set_value(motors[0].gpio_en,1);
		gpio_set_value(motors[0].gpio_reset,1);
		gpio_set_value(motors[0].gpio_control,1);
		gpio_set_value(motors[0].gpio_dir,(speed<0)?0:1);
		if(distance > 0) {
			gpio_set_value(motors[0].gpio_clk,1);
			udelay(5);
			motors[0].distance--;
			gpio_set_value(motors[0].gpio_clk,0);
			mod_timer(&caddytimer,jiffies+CADDYMOTOR_POLL_INTERVAL);
		} else {
			motors[0].speed = 0;
			gpio_set_value(motors[0].gpio_en,0);
		}
	} else {
		gpio_set_value(motors[0].gpio_en,0);
	}
	wake_up_interruptible(&tickq);
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


	return 0;
}

static int caddymotor_release(struct inode *inode, struct file *filp)
{
	struct caddymotor_dev *devp = filp->private_data;

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
	int speed;
	int distance;
	char parse_buf[20];
	char *p;
	char *q;
	
	if(size >= (sizeof(parse_buf)-1))
		return -EFAULT;
	if(copy_from_user(parse_buf,buf,size))
		return -EFAULT;
	
	parse_buf[size]=0;
	p=parse_buf;
	while(isspace(*p))
		p++;
	speed = simple_strtol(p,&q,0);
	if(!q || q==p)
		return -EINVAL;
	p=q;
	while(isspace(*p))
		p++;
	/* did we find another token? */
	if(!(*p))
		return -EINVAL;
	distance = simple_strtol(p,&q,0);
	if(!q || q==p)
		return -EINVAL;
	if(distance<0)
		return -EINVAL;
		
	pr_info("speed %d distance %d\n",speed,distance);
	motors[0].speed = speed;
	motors[0].distance = distance;
	if(!timer_pending(&caddytimer))
		mod_timer(&caddytimer,jiffies+CADDYMOTOR_POLL_INTERVAL);
	return size;
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

static int caddymotor_add_gpio(struct platform_device *pdev, struct device_node *of_node, char *node_name, char *gpio_description, int *caddymotor_gpio) {
	int ret;
  *caddymotor_gpio = of_get_named_gpio(of_node, node_name, 0);
  if(gpio_is_valid(*caddymotor_gpio)) {
  	dev_info(&pdev->dev, "%s %d\n",node_name,*caddymotor_gpio);
  	ret = gpio_request_one(*caddymotor_gpio,GPIOF_OUT_INIT_LOW, gpio_description);
  	if(ret) {
  		dev_err(&pdev->dev,"Failed to get gpio\n");
  		return ret;
  	}
	} else {
 		dev_err(&pdev->dev,"Invalid gpio\n");
		return -EINVAL;
	}
	return 0;
}

static void caddymotor_del_gpio(struct platform_device *pdev, int gpio) {
 	dev_info(&pdev->dev, "freeing gpio %d\n",gpio);

	if(gpio_is_valid(gpio))
		gpio_free(gpio);
}
/* driver initialisation */

//static int __init caddymotor_probe(struct platform_device *pdev)
static int caddymotor_probe(struct platform_device *pdev)
{
	int ret;
	const struct of_device_id *match;
	struct device_node *node, *pp;
	int motor;

	dev_info(&pdev->dev, "Opticorder caddy motor support installing...\n");
	match = of_match_device(caddymotor_of_match, &pdev->dev);
	if(!match) {
		dev_err(&pdev->dev, "Couldn't find device tree node, exiting\n");
		return -EINVAL;
	}
	node = pdev->dev.of_node;
	caddymotor.num_motors = of_get_child_count(node);
	if(caddymotor.num_motors > NUM_MOTORS) {
		dev_warn(&pdev->dev, "Found %d motors, using only the first %d\n",caddymotor.num_motors,NUM_MOTORS);
		caddymotor.num_motors = NUM_MOTORS;
	}
	if(caddymotor.num_motors == 0) {
		dev_warn(&pdev->dev, "No motors found in device tree\n");
		return 0;
	}
	/* create the caddymotor character device */
	/* initialise character device */
	cdev_init(&caddymotor.cdev, &caddymotor_fops);
	/* claim ownership */
	caddymotor.cdev.owner = THIS_MODULE;
	/* add character device */
	ret = cdev_add(&caddymotor.cdev, dev, caddymotor.num_motors);
	if (ret) {
		goto error_bus;
	}
	motor = 0;
	for_each_child_of_node(node, pp) {
		dev_info(&pdev->dev, "Motor %d:\n",motor);
		if(motor>=caddymotor.num_motors)
			break;
	  /* set up GPIOs */
		if(caddymotor_add_gpio(pdev, pp, "gpio-clk", "CADDYMOTOR_CLK", &(motors[motor].gpio_clk)))
			continue;
		if(caddymotor_add_gpio(pdev, pp, "gpio-en", "CADDYMOTOR_EN", &(motors[motor].gpio_en)))
			continue;
		if(caddymotor_add_gpio(pdev, pp, "gpio-dir", "CADDYMOTOR_DIR", &(motors[motor].gpio_dir)))
			continue;
		if(caddymotor_add_gpio(pdev, pp, "gpio-reset", "CADDYMOTOR_RESET", &(motors[motor].gpio_reset)))
			continue;
		if(caddymotor_add_gpio(pdev, pp, "gpio-half-full", "CADDYMOTOR_HALF_FULL", &(motors[motor].gpio_half_full)))
			continue;
		if(caddymotor_add_gpio(pdev, pp, "gpio-control", "CADDYMOTOR_CONTROL", &(motors[motor].gpio_control)))
			continue;
		if (!device_create(caddymotor_class, NULL, (dev), NULL, "caddymotor"))
			continue;
	}		

	caddytimer.data = (unsigned long)pdev;

	return 0;
error_bus:
	dev_err(&pdev->dev,"failed.\n");
	return ret;
}

static int __exit caddymotor_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Opticorder caddy motor support removing\n");
	if(timer_pending(&caddytimer))
		del_timer(&caddytimer);
	caddymotor_del_gpio(pdev, motors[0].gpio_clk);
	caddymotor_del_gpio(pdev, motors[0].gpio_en);
	caddymotor_del_gpio(pdev, motors[0].gpio_dir);
	caddymotor_del_gpio(pdev, motors[0].gpio_reset);
	caddymotor_del_gpio(pdev, motors[0].gpio_half_full);
	caddymotor_del_gpio(pdev, motors[0].gpio_control);
	platform_set_drvdata(pdev, NULL);
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
		.of_match_table = caddymotor_of_match,
	},
};

/* bus device */
static struct platform_device *caddymotor_device;


static int __init caddymotor_init(void)
{
	int ret;

	/* general initialisation */
	spin_lock_init(&caddymotor_lock);

	/* register a range of device nodes */
	ret = alloc_chrdev_region(&dev, 0, NUM_MOTORS, "caddymotor");
	if (ret)
		goto error;

	/* create the class and devices */
	caddymotor_class = class_create(THIS_MODULE, "caddymotor");

	/* register the device on a bus. */
	caddymotor_device=platform_device_alloc("caddymotor",0);
	if(!caddymotor_device)
		goto error_bus;
	if(platform_device_add(caddymotor_device))
		goto error_bus;

	/* create proc access*/
	proc_file_entry = proc_create(PROC_CADDYMOTOR, 0, NULL, &proc_caddymotor_operations);
	if(proc_file_entry == NULL)
		goto error_bus;

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

MODULE_DEVICE_TABLE(of, caddymotor_of_match);

module_init(caddymotor_init);
module_exit(caddymotor_exit);

MODULE_AUTHOR("Chris Jones <chris@martin-jones.com>");
MODULE_DESCRIPTION("Opticorder caddy motor driver");
MODULE_LICENSE("GPL");
