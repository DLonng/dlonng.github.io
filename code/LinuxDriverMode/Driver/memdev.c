#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm/uaccess.h>

dev_t devnum;
struct cdev mdev;

int dev1_register[5] = { 0 };
int dev2_register[5] = { 0 };

int memdev_open(struct inode *, struct file *);
int memdev_release(struct inode *, struct file *);
ssize_t memdev_read(struct file *, char __user *, size_t, loff_t *);
ssize_t memedev_write(struct file *, const char __user *, size_t, loff_t *);
loff_t memdev_llseek(struct file *, loff_t, int);


/*
 * Memdev module file operations.
 */
struct file_operations memdev_ops = {
	.open = memdev_open,
	.release = memdev_release,
	.read = memdev_read,
	.write = memedev_write,
	.llseek = memdev_llseek,
};


/*
 * Memdev module open. 
 */
int memdev_open(struct inode *inode, struct file *pfile) {
	/* Get Minor device number. */
	int num = MINOR(inode->i_rdev);

	if (0 == num)
		pfile->private_data = dev1_register;
	else if (1 == num)
		pfile->private_data = dev2_register;
	else
		return -ENODEV;

	return 0;
}

/*
 * Memdev module release.
 */
int memdev_release(struct inode *inode, struct file *pfile) {
	return 0;
}


/*
 * Memdev module read. 
 */
ssize_t memdev_read(struct file *pfile, char __user *buf, size_t size, loff_t *pos) {
	int ret_num = 0;
	unsigned int count = size;
	unsigned long temp_pos = *pos;

	/* Get devices reg base address. */
	int *reg_addr = pfile->private_data;

	/* Determines wherther the read position is valid. */
	if (temp_pos >= 5 * sizeof(int))
		return 0;
	
	/* Get the number read. */
	if (size > 5 * sizeof(int) - temp_pos)
		count = 5 * sizeof(int) - temp_pos;

	/* Read data to use space. */
	if (copy_to_user(buf, reg_addr + temp_pos, count)) {
		ret_num = -EFAULT;
	} else {
		*pos += count;
		ret_num = count;
	}

	return ret_num;      
}


/*
 * Memdev module write. 
 */
ssize_t memedev_write(struct file *pfile, const char __user *buf, size_t size, loff_t *pos) {
	int ret_num = 0;
	unsigned int count = size;
	unsigned long temp_pos = *pos;

	/* Get devices reg base address. */
	int *reg_addr = pfile->private_data;

	/* Determines wherther the read position is valid. */
	if (temp_pos >= 5 * sizeof(int))
		return 0;
	
	/* Get the number read. */
	if (size > 5 * sizeof(int) - temp_pos)
		count = 5 * sizeof(int) - temp_pos;

	/* Write data to kernel space from use space. */
	if (copy_from_user(reg_addr + temp_pos, buf, count)) {
		ret_num = -EFAULT;
	} else {
		*pos += count;
		ret_num = count;
	}

	return ret_num;      
}


/*
 * Memdev module llseek. 
 */
loff_t memdev_llseek(struct file *pfile, loff_t offset, int where) {
	loff_t new_pos = 0;

	switch (where) {
		/* File start pos. */
		case SEEK_SET:
			new_pos = 0 + offset;
			break;
		/* File current pos. */
		case SEEK_CUR:
			new_pos = pfile->f_pos + offset;
			break;
		/* File end pos. */
		case SEEK_END:
			new_pos = (5 * sizeof(int) - 1) + offset;
			break;
		default:
			return -EINVAL;
	}

	if ((new_pos < 0) || (new_pos > 5 * sizeof(int)))
		return -EINVAL;
	/* Update file point. */
	pfile->f_pos = new_pos;

	return new_pos;
}



/* 
 * Memdev module init. 
 */
static int __init memdev_init(void) {
	/* Init cdev struct. */
	cdev_init(&mdev, &memdev_ops);

	/* Register char devices. */
	alloc_chrdev_region(&devnum, 0, 2, "memdev");
	cdev_add(&mdev, devnum, 2);

	return 0;
}


/*
 * Memdev module exit.
 */
static void __exit memdev_exit(void) {
	/* Del mdev. */
	cdev_del(&mdev);

	/* Free devnum. */
	unregister_chrdev_region(devnum, 2);
}



MODULE_LICENSE("GPL");

module_init(memdev_init);
module_exit(memdev_exit);
