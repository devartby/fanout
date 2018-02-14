/*
 * fanout.c:  A one-to-many multiplexer
 *
 * Copyright (C) 2010-2015, Bob Smith
 * This software is released under your choice of either
 * the GPLv2 or the 3-clause BSD license.
 * 
 * Initial release: Bob Smith
 * changes, added more locking: Edwin van den Oetelaar (www.oetelaar.com)
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include "fanout.h"


/* Limits and other defines */
/* The # fanout devices.  Max minor # is one less than this */
#define NUM_FO_DEVS (255)
#define DEVNAME "fanout"
#define DEBUGLEVEL (2)


/* Data structure definitions */
/* This data structure describes one fanout device.  There
 * is one of these for each instance (minor #) of fanout */
struct fo {
	int minor;		/* minor number of this fanout instance */
	char *buf;		/* points to circular buffer, first char */
	int indx;		/* where to put next char received */
	loff_t count;		/* number chars received */
	int readers_num;
	int writers_num;
	wait_queue_head_t inq;	/* readers wait on this queue */
	struct semaphore sem;	/* lock to keep buf/indx sane */
};

struct fo_user {
	struct fo *dev;
	int fix;
	int seek_set;
	int seek_alignment;
};

/*  Function prototypes.  */
int fanout_init_module(void);
void fanout_exit_module(void);
static int fanout_open(struct inode *, struct file *);
static int fanout_release(struct inode *, struct file *);
static ssize_t fanout_read(struct file *, char *, size_t, loff_t *);
static ssize_t fanout_write(struct file *, const char *, size_t, loff_t *);
static unsigned int fanout_poll(struct file *, poll_table *);
static long fanout_ioctl(struct file *, unsigned int, unsigned long);


/* Global variables */
static int buffersize = 1048576;	/* Size of the circular buffer 1M */
static unsigned int numberofdevs = NUM_FO_DEVS;
static int fo_major = 0;	/* major device number */
/* Debuglvl controls whether a printk is executed
 * 0 = no printk at all
 * 1 = printk on error only
 * 2 = printk on errors and on init/remove
 * 3 = debug prink to trace calls into fanout
 * 4 = debug trace inside of fanout calls 
 */
static unsigned int debuglevel = DEBUGLEVEL;	/* printk verbosity */

struct cdev fo_cdev;		/* a char device global just 1 */
dev_t fo_devicenumber;		/* first device number */

module_param(buffersize, int, S_IRUSR);
module_param(debuglevel, int, S_IRUSR);
module_param(numberofdevs, int, S_IRUSR);

static struct fo *fo_devs;	/* point to devices (minors) */


/* map the callbacks into this driver */
static struct file_operations fanout_fops = {
	.owner = THIS_MODULE,
	.read = fanout_read,
	.open = fanout_open,
	.write = fanout_write,
	.poll = fanout_poll,
	.unlocked_ioctl = fanout_ioctl,
	.release = fanout_release
};


/* Module description and macros */
MODULE_DESCRIPTION
	("A device to replicate input (writer) on all outputs (readers), readers block, writer never blocks");
MODULE_AUTHOR("Bob Smith");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(buffersize, "Size of each buffer. default=16384 (16K) ");
MODULE_PARM_DESC(debuglevel, "Debug level. Higher=verbose. default=2");
MODULE_PARM_DESC(numberofdevs,
		 "Create this many minor devices. default=16");


int fanout_init_module(void)
{
	int i, err;

	buffersize = buffersize / 2 * 2;

	fo_devs = kmalloc(numberofdevs * sizeof(struct fo), GFP_KERNEL);
	if (fo_devs == NULL) {
		if (debuglevel >= 1)
			printk(KERN_ALERT "%s: init fails: no memory\n",
					DEVNAME);
		return 0;
	}
	/* clean memory and init device structures */
	memset(fo_devs, 0, numberofdevs * sizeof(struct fo));
	for (i = 0; i < numberofdevs; i++) {	/* for every minor device */
		fo_devs[i].minor = i;		/* set number */
		fo_devs[i].buf = (char *) 0;	/* init buf */
		fo_devs[i].indx = 0;		/* init index */
		fo_devs[i].count = 0;		/* init count */
		init_waitqueue_head(&fo_devs[i].inq);
#ifdef init_MUTEX
		init_MUTEX(&fo_devs[i].sem);	/* init sema */
#else
		sema_init(&fo_devs[i].sem,1);	/* init sema */
#endif

	}

	err = alloc_chrdev_region(&fo_devicenumber, 0, numberofdevs, DEVNAME);
	if (err < 0) {
		if (debuglevel >= 1)
			printk(KERN_ALERT "%s: init fails: err=%d\n",
				DEVNAME, err);
		return err;
	}
	fo_major = MAJOR(fo_devicenumber);	/* save assign major */
	cdev_init(&fo_cdev, &fanout_fops);	/* init dev structures */
	kobject_set_name(&(fo_cdev.kobj), "%s%d", DEVNAME, fo_devicenumber);

	err = cdev_add(&fo_cdev, fo_devicenumber, numberofdevs);
	if (err < 0) {
		if (debuglevel >= 1)
			printk(KERN_ALERT "%s: init fails: err=%d\n",
					DEVNAME, err);
		return err;
	}

	if (debuglevel >= 2) {
		printk(KERN_INFO
			"%s: install %d minor devices on major number %d\n",
		   			DEVNAME, numberofdevs, fo_major);
	}
	return 0;			/* success */
}


void fanout_exit_module(void)
{
	int i;

	if (!fo_devs)		/* anything to release ? */
		return;

	for (i = 0; i < numberofdevs; i++) {	/* for every minor */
		if (fo_devs[i].buf)
			kfree(fo_devs[i].buf);	/* free alloced memory */
	}

	cdev_del(&fo_cdev);		/* delete major device */
	kfree(fo_devs);			/* free */
	fo_devs = NULL;			/* reset pointer */
	unregister_chrdev_region(fo_devicenumber, numberofdevs);

	if (debuglevel >= 2)
		printk(KERN_INFO "%s: uninstalled\n", DEVNAME);
}

static int fanout_open(struct inode *inode, struct file *filp)
{
	int mnr = iminor(inode);
	struct fo_user *usr;
	struct fo *dev = &fo_devs[mnr];

	if (debuglevel >= 3) {
		printk(KERN_DEBUG "%s: dev=%d open\n", DEVNAME, mnr);
	}

	if (down_interruptible(&dev->sem))	/* prevent races on open */
		return -ERESTARTSYS;

	if (!dev->buf) {
		/* alloc the buffer, shared by all readers */
		dev->buf = kmalloc(buffersize, GFP_KERNEL);
		if (!dev->buf) {
			if (debuglevel >= 1) {
				printk(KERN_ALERT "%s: dev=%d open fails: no memory\n",
						DEVNAME, mnr);
			}
			up(&dev->sem);	/* unlock sema */
			return -ENOMEM;
		}
	}

	usr = kmalloc(sizeof(struct fo_user), GFP_KERNEL);
	if (!usr) {
		if (debuglevel >= 1) {
			printk(KERN_ALERT "%s: dev=%d open fails: no memory\n",
					DEVNAME, mnr);
		}
		up(&dev->sem);	/* unlock sema */
		return -ENOMEM;
	}

	usr->dev = dev;
	usr->fix = 0;
	usr->seek_set = 0;
	usr->seek_alignment = 0;

	/* store which fanout device in the file's private data */
	filp->private_data = usr;

	filp->f_pos = dev->count;

	if (filp->f_mode & FMODE_READ) {
		dev->readers_num++;
	}
	if (filp->f_mode & FMODE_WRITE) {
		dev->writers_num++;
	}

	up(&dev->sem);		/* unlock semaphore we are done */

	return nonseekable_open(inode, filp);	/* success */
}


static int fanout_release(struct inode *inode, struct file *filp)
{
	struct fo_user *usr = filp->private_data;
	struct fo *dev = usr->dev;

	if (debuglevel >= 3) {
		printk(KERN_DEBUG "%s: dev=%d close\n",
				DEVNAME, dev->minor);
	}

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	if (filp->f_mode & FMODE_READ) {
		dev->readers_num--;
	}
	if (filp->f_mode & FMODE_WRITE) {
		dev->writers_num--;
	}

	if (dev->readers_num <= 0 &&
	    dev->writers_num <= 0) {
		if (debuglevel >= 3) {
			printk(KERN_DEBUG "%s: dev=%d reset\n",
					DEVNAME, dev->minor);
		}
		dev->count = dev->indx = 0;
	}

	kfree(usr);

	up(&dev->sem);

	return 0;			/* success */
}


static ssize_t fanout_read(
	struct file *filp,
	char __user * buff,
	size_t count, loff_t * off)
{
	int ret;
	loff_t xfer;		/* num bytes read from fanout buf */
	int cpcnt, cpstrt;	/* cp count and start location */
	struct fo_user *usr = filp->private_data;
	struct fo *dev = usr->dev;

	if (down_interruptible(&dev->sem))	/* lock semaphore */
		return -ERESTARTSYS;

	if (debuglevel >= 3) {
		printk(KERN_DEBUG "%s: dev=%d read: count=%zu offset=%lld overall=%lld\n",
				DEVNAME, dev->minor, count, *off, dev->count);
	}

	if (usr->fix)
	{
		loff_t offset;

		usr->fix = 0;

		if (usr->seek_set) {
			if (dev->count > buffersize) {
				offset = dev->count - buffersize;
			} else {
				offset = 0;
			}
		} else {
			offset = dev->count;
		}

		if (usr->seek_alignment) {
			offset = offset / 2 * 2;
		}

		*off = offset;
	}

	/* Wait here until new data is available */
	while (*off == dev->count) {
		up(&dev->sem);		/* unlock sema */
		if (wait_event_interruptible(dev->inq, (*off != dev->count)))
			return -ERESTARTSYS;
		if (down_interruptible(&dev->sem))	/* lock */
			return -ERESTARTSYS;
	}

	/* Verify that data requested is in the buffer or is next byte */
	xfer = dev->count - *off;	/* send count minus requested pointer */
	if ((xfer > (loff_t) buffersize) || (xfer < 0)) {
		printk(KERN_DEBUG "%s: dev=%d overrun: xfer=%lld buffersize=%d",
				DEVNAME, dev->minor, xfer, buffersize);
		up(&dev->sem);		/* unlock sema */
		return -EPIPE;		/* buffer overrun */
	}

	/* Copy the new data out to the user */
	xfer = dev->count - *off;	/* amount of data available to copy */

	/* BUG: we need to check for a wrap on offset and count */

	 /* xfer less then available when requested */
	xfer = ((loff_t)count < xfer) ? (loff_t)count : xfer;
	ret = xfer;			/* we will handle these bytes */
	while (xfer) {
		/* copy start is where the reader last read (indx - (count - off)) */
		cpstrt = dev->indx - (dev->count - *off);
		if (cpstrt < 0) {	/* adjust copy count if needed */
			cpcnt = ((loff_t)(-cpstrt) <  xfer) ? (loff_t)(-cpstrt) : xfer;
			cpstrt += buffersize;
		} else {
			cpcnt = xfer;
		}

		if (copy_to_user(buff, dev->buf + cpstrt, cpcnt)) {
			up(&dev->sem);
			return -EFAULT;
		}

		buff += cpcnt;
		xfer -= cpcnt;
		*off += cpcnt;
	}

	if (debuglevel >= 3) {
		printk(KERN_DEBUG "%s: dev=%d ret: offset=%lld ret=%zd\n",
				DEVNAME, dev->minor, *off, ret);
	}

	up(&dev->sem);		/* unlock sema */

	return ret;
}


static ssize_t fanout_write(
	struct file *filp,
	const char __user * buff,
	size_t count, loff_t * off)
{
	int ret;
	int xfer;			/* num bytes to read from user */
	int cpcnt;		/* num bytes in a copy */
	struct fo_user *usr = filp->private_data;
	struct fo *dev = usr->dev;

	if (down_interruptible(&dev->sem)) {	/* lock semaphore */
		return -ERESTARTSYS;
	}

	if (debuglevel >= 3)
		printk(KERN_DEBUG "%s: dev=%d write: count=%zu offset=%lld\n",
				DEVNAME, dev->minor, count, *off);

	/* Copy at most one-quarter of the circular buffer size.  This
	 * gives readers more of a chance to wake up and get some data 
	 * In other words feed the reader little chuncks of data, they will
	 * call again if they still want more
	 */
	ret = xfer = min((int) count, buffersize / 4);

	/* loop over the amount since the buffer is not a single block
	 * but wraps arround
	 */
	while (xfer) {
		cpcnt = buffersize - dev->indx;
		cpcnt = min(cpcnt, xfer);

		if (copy_from_user(dev->buf + dev->indx, buff, cpcnt)) {
			up(&dev->sem);	/* unlock semaphore */
			return -EFAULT;
		}
		*off += cpcnt;
		dev->indx += cpcnt;
		dev->indx = (dev->indx == buffersize) ? 0 : dev->indx;
		xfer -= cpcnt;	
		buff += cpcnt;
	}

	dev->count += ret;		/* update file size */
	up(&dev->sem);			/* unlock semaphore */

	/* This is what the readers have been waiting for */
	wake_up_interruptible(&dev->inq);

	return ret;
}

static unsigned int fanout_poll(struct file *filp, poll_table * ppt)
{
	/* The circular buffer is always available for writing */
	int ready_mask = POLLOUT | POLLWRNORM;
	struct fo_user *usr = filp->private_data;
	struct fo *dev = usr->dev;

	poll_wait(filp, &dev->inq, ppt);

	if (filp->f_pos != dev->count) {
		ready_mask = (POLLIN | POLLRDNORM);
	}

	if (debuglevel >= 3) {
		printk(KERN_DEBUG "%s: dev=%d poll: ready_mask=0x%x\n",
				DEVNAME, dev->minor, ready_mask);
	}

	return ready_mask;
}

static long fanout_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	long ret = 0;
	struct fo_user *usr = filp->private_data;
	struct fo *dev = usr->dev;

	if (down_interruptible(&dev->sem)) {	/* lock semaphore */
		return -ERESTARTSYS;
	}

	switch (cmd) {
	case FANOUT_GET_RD:
		if (copy_to_user((int __user *)arg,
				&dev->readers_num, sizeof(int))) {
			ret = -EFAULT;
		} else {
			if (debuglevel >= 3) {
				printk(KERN_DEBUG "%s: dev=%d ioctl: readers_num=%d\n",
						DEVNAME, dev->minor, dev->readers_num);
			}
		}
		break;
	case FANOUT_GET_WR:
		if (copy_to_user((int __user *)arg,
				&dev->writers_num, sizeof(int))) {
			ret = -EFAULT;
		} else {
			if (debuglevel >= 3) {
				printk(KERN_DEBUG "%s: dev=%d ioctl: writers_num=%d\n",
						DEVNAME, dev->minor, dev->writers_num);
			}
		}
		break;
	case FANOUT_SET_SEEK_SET:
		if (copy_from_user(&usr->seek_set,
				(int __user *)arg, sizeof(int)))  {
			ret = -EFAULT;
		} else {
			usr->fix = 1;
			if (debuglevel >= 3) {
				printk(KERN_DEBUG "%s: dev=%d ioctl: seek_set=%d\n",
						DEVNAME, dev->minor, usr->seek_set);
			}
		}
		break;
	case FANOUT_SET_SEEK_ALIGNMENT:
		if (copy_from_user(&usr->seek_alignment,
				(int __user *)arg, sizeof(int))) {
			ret = -EFAULT;
		} else {
			usr->fix = 1;
			if (debuglevel >= 3) {
				printk(KERN_DEBUG "%s: dev=%d ioctl: seek_alignment=%d\n",
						DEVNAME, dev->minor, usr->seek_alignment);
			}
		}
		break;
	default:
		ret = -EFAULT;
		break;
	}

	up(&dev->sem);

	return ret;
}

module_init(fanout_init_module);
module_exit(fanout_exit_module);
