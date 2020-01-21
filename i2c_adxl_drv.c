/* An i2c  driver for adxl345 accelometer */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/ioctl.h>

#define ADXL_DEVICE_NAME "adxl_i2c"
#define ADXL_CLASS "adxl-class "
#define SIZE_TRIPLE_AXIS (32*3)
#define WR_VALUE _IOW('a', 'a', uint8_t *)
#define RD_VALUE _IOR('a', 'b', uint8_t *)

/* Function Prototypes */
//static int adxl_read(struct device *, unsigned char);
//static int adxl_write(struct device *, unsigned char, unsigned char);
int adxl_open(struct inode *, struct file *);
int adxl_release(struct inode *inode, struct file *filp);
static long adxl_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static unsigned int adxl_major = 0;
//static unsigned int minor = 0;
static struct class *adxl_class = NULL;
struct device *dev = NULL;
dev_t dev_num;
uint8_t value = 0;

struct axis_triple {
	int x;
	int y;
	int z;
};
struct adxl345_dev {	
	struct i2c_client *client;
	struct input_dev *input;
	struct mutex mutex;
	struct axis_triple saved;
       	int current_pointer;
	bool disabled;
	bool opened;
	struct cdev cdev;	
};

/* file operations structure */
struct file_operations adxl_fops = {
	.owner = THIS_MODULE,
//	.read = adxl_read,
//	.write = adxl_write,
	.open = adxl_open,
	.unlocked_ioctl = adxl_ioctl,
	.release = adxl_release	
};
static const struct of_device_id adxl345_ids[] = {
	{ .compatible = "adi,adxl345"},
	{}
};

int adxl_open(struct inode *inode, struct file *filp)
{
	struct adxl345_dev *dev = NULL;

	dev = container_of(inode->i_cdev, struct adxl345_dev, cdev);
	/* Check if device exists */
	if (dev == NULL) {
		pr_err("container of didnt find any valid data\n");
		return -ENODEV;
	}

	dev->current_pointer = 0;

	filp->private_data = dev;

	if (inode->i_cdev != &dev->cdev) {
		pr_err("Device open: failed");
		return -ENODEV;
	}

	dev = (struct adxl345_dev*) kzalloc(SIZE_TRIPLE_AXIS, GFP_KERNEL);
	if (dev == NULL) {
	
		pr_err("Error allocating memory \n");
		return -ENOMEM;
	}

	return 0;

}

int adxl_release(struct inode *inode, struct file *filp)
{
	struct adxl345_dev *dev;
	dev = filp->private_data;

	if (dev != NULL) {
		kfree(dev);
		dev= NULL;
	}
	dev->current_pointer = 0;

	return 0;

}
static int adxl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct adxl345_dev *dev = NULL;

	
	
	int err, devno;
	
	/* Obtain a range of minor numbers, starting from 0, to work with */
	err = alloc_chrdev_region(&devno, 0, 1, ADXL_DEVICE_NAME);
	if(err < 0) {
		pr_err("alloc_chrdev_region failed for %s\n", ADXL_DEVICE_NAME);
		return err;
	}

	/* Create a device class */
	adxl_class = class_create(THIS_MODULE, ADXL_DEVICE_NAME);
	if (IS_ERR(adxl_class)) {
		err = PTR_ERR(adxl_class);
		return err;
	}
	cdev_init(&dev->cdev, &adxl_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &adxl_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding adxl_dev \n", err);

	i2c_set_clientdata(client, dev);

	return 0;
}

/* ioctl code for i2c read/write */
static long adxl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	uint8_t reg;
	int data_recvd;
	int ret;
	
	struct i2c_client *client = to_i2c_client(dev);

	switch(cmd) {
		case WR_VALUE:
				ret = copy_from_user(&value, (uint8_t*)arg, sizeof(value));
				if(ret != 0) {
					pr_err("copy from user failed %d", ret);
					return -EFAULT;	
				}
				/* obtain 8 bit register address from lsb */
				reg = value >> 8;
				i2c_smbus_write_byte_data(client, reg, value);
				break;
		case RD_VALUE:
				reg = value >> 8;
				data_recvd = i2c_smbus_read_byte_data(client, reg);
				ret = copy_to_user((uint8_t*)arg, &data_recvd, sizeof(data_recvd));
				if(ret != 0) {
					pr_err("copy to user failed %d", ret);
					return -EFAULT;	
				}
				break;
		
	}
	return 0;
}
#if 0
static int adxl_read(struct device *dev, unsigned char reg)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_smbus_read_byte_data(client, reg);

}

static int adxl_write(struct device *dev, unsigned char reg, unsigned char val)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_smbus_write_byte_data(client, reg, val);
}
#endif

static int adxl_remove(struct i2c_client *client)
{
	struct adxl345_dev *adxl345_dev1;
	adxl345_dev1 = i2c_get_clientdata(client);

	device_destroy(adxl_class, MKDEV(adxl_major, 0));
	kfree(adxl345_dev1);
	class_destroy(adxl_class);
	unregister_chrdev_region(MKDEV(adxl_major, 0), 1);
	return 0;
}

static const struct i2c_device_id adxl345_id[] = {
	{"adi, adxl345"},
	{}
};

static struct i2c_driver adxl345_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "adxl345",
		.of_match_table = of_match_ptr(adxl345_ids)
	},

	.probe = adxl_probe,
	.remove = adxl_remove,
	.id_table = adxl345_id,
	
};


MODULE_DEVICE_TABLE(i2c, adxl345_id);
module_i2c_driver(adxl345_i2c_driver);
MODULE_AUTHOR("Suraj Vanjeeswaran <surajv1986@gmail.com>");
MODULE_LICENSE("GPL");

