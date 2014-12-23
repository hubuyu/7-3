#include <linux/module.h> 
#include <linux/init.h>   
#include <linux/fs.h>   
#include <linux/spi/spi.h> 
#include <asm/uaccess.h>    
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/gpio.h>		//gpio API
#include <linux/interrupt.h>		//gpio API
#include "register.h"

#define DEMO_MAJOR		256	//need to be check 
#define DEMO_ADDR		0

#define COMMAND1		1
#define COMMAND2		3

#define BUF_SIZE		256

struct demo_data {
	dev_t					devt ;
	struct spi_device		*spi_device ;
	struct list_head		device_entry ;
	unsigned char			addr ;
	unsigned char			buffer[BUF_SIZE];
	int						irq ;
	char					test ;
};

static LIST_HEAD( device_list );
static struct class *demo_class ;

static int mcp23s17_read(struct demo_data *demo , unsigned reg)
{
    int status = 0 ;
    unsigned char tx[2],rx[2];
    tx[0] = OP_CODE | ( demo->addr << 1 ) | READ_CMD ;
    tx[1] = reg ;
    gpio_direction_output( PIFACE_CS_PIN , ENABLE );
    status = spi_write_then_read( demo->spi_device , tx , sizeof(tx),
                                    rx , sizeof(rx));
    gpio_direction_output( PIFACE_CS_PIN , DISABLE );
    return (status < 0)?status : (rx[0]|(rx[1]<<8)) ;
}

static int mcp23s17_write(struct demo_data *demo , unsigned reg , unsigned val )
{
    unsigned char tx[3];
    int status = 0  ;
//step 1    cs to be low
    gpio_direction_output( PIFACE_CS_PIN , ENABLE );
//step 2    transfer the data
    tx[0] = OP_CODE | ( demo->addr << 1 ) | WRITE_CMD ;
    tx[1] = reg  ;
    tx[2] = val;
    status = spi_write(demo->spi_device, tx , sizeof(tx) );
//step 3    cs to be high
    gpio_direction_output( PIFACE_CS_PIN , DISABLE );
	return status ;
}


static int demo_open(struct inode *inode , struct file *filp )
{	
	struct demo_data *demo ;
	unsigned char iocfg ;
	int status = -ENXIO ;

	printk("Drv info : %s()\n", __FUNCTION__);

	list_for_each_entry(demo , &device_list , device_entry ){
		if(demo->devt == inode->i_rdev){
			status = 0 ;
			printk("demo test char = %c \n", demo->test);
			break ;
		}
	}

	if (status != 0){
		printk("get the demo structure failured ");
		return status ;	
	}
		
	filp->private_data = demo ;
	demo->addr = DEMO_ADDR ;
#if 1
	//initialize the device
    gpio_request( PIFACE_CS_PIN ,"demo");
    iocfg = BANK_OFF | INT_MIRROR_OFF | SEQOP_OFF | DISSLW_OFF | HAEN_ON | ODR_OFF | INTPOL_LOW;
    mcp23s17_write(demo , IOCON , iocfg);     //enable hardware address
    mcp23s17_write(demo , IODIRA , 0x00 );    //set port A as output
    mcp23s17_write(demo , IODIRB , 0xFF );    //set port B as input

    mcp23s17_write(demo , GPPUB , 0xFF );     //GPIOB PULL UP

//    mcp23s17_write(demo , GPINTENB , ENABLE_INTB);    //enable interrupt

    demo->irq = gpio_to_irq(GPIO_INTERRUPT_PIN);
    printk("demo IRQ is %d \n" , demo->irq );
//    status = request_irq(demo->irq, demo_isr , IRQF_TRIGGER_FALLING, "pi    face_irq", NULL);
    if(status<0){
		printk("Could not register interrupt \n");
		return status ;
	}
#endif
	return status ;
}

static int demo_release(struct inode *inode , struct file *filp )
{
	unsigned char intenb ;
	struct demo_data *demo ;
	demo = filp->private_data ;
	printk("Drv info : %s()\n", __FUNCTION__);
	if ( demo == NULL )
		return -ENODEV ;
	intenb = mcp23s17_read(demo ,GPINTENB );
	if( intenb ){
		mcp23s17_write(demo , GPINTENB , DISABLE_INTB);//enable interru    pt
	}
//	free_irq(demo->irq , NULL );
	gpio_free(GPIO_INTERRUPT_PIN);
	filp->private_data = NULL ;
	return 0 ;
}

static long demo_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int status=1 ;
	struct demo_data *demo ;
	demo = filp->private_data ;

	switch (cmd){
		case	COMMAND1 :
			printk("Case - Command 1\n" );
			break;
		case	COMMAND2 :
			printk("Case - Command 2\n" );
			break;
		default :
			printk("%s() cmd-%d is not supprt at this moment\n" , __FUNCTION__ , cmd);
			break;
	}
	
	return status ;
}

static ssize_t demo_write(struct file *filp ,const char __user *buf , size_t count , loff_t *f_ops )
{
	int err ;
	struct demo_data *demo ;
	demo = filp->private_data ;
	printk("Drv info : %s()\n", __FUNCTION__);
	err = copy_from_user(demo->buffer+*f_ops , buf ,count );
	if(err )
		return -EFAULT ;
	err=mcp23s17_write( demo , GPIOA , demo->buffer[0] );  //write the value
	
	return err ;
}

static ssize_t demo_read( struct file *filp , char __user *buf , size_t count , loff_t *f_ops )
{
	int err ;
	struct demo_data *demo ;
	demo = filp->private_data ;
	printk("Drv info : %s()\n", __FUNCTION__);
	demo->buffer[0] =  mcp23s17_read(demo , GPIOB );
	err = copy_to_user(buf, demo->buffer+*f_ops , count );
	if (err)
		return -EFAULT ;
	return err ;
}

static const struct file_operations demo_ops = {
	.open			= demo_open ,
	.release		= demo_release ,
	.write			= demo_write ,
	.read			= demo_read ,
	.unlocked_ioctl = demo_ioctl ,
};

static int demo_probe (struct spi_device *spidev)
{
	struct demo_data *demo = NULL;
	unsigned long minor ;

	printk("Drv info : %s()\n", __FUNCTION__);
	demo = kzalloc(sizeof(*demo), GFP_KERNEL);
	if(!demo)
		return -ENOMEM ;

	demo->spi_device = spidev ;
	INIT_LIST_HEAD(&demo->device_entry);

	minor = 0 ;
	demo->devt = MKDEV(DEMO_MAJOR , minor);
	device_create(demo_class , &spidev->dev,demo->devt , 
				  demo , "hubuyu");
	printk("device number is %d \n", demo->devt);

	demo->test = 'X';
	list_add( &demo->device_entry , &device_list );
	spi_set_drvdata( spidev , demo);
	printk("Drv info : %s()\n", __FUNCTION__);
	return 0 ;
}

static int demo_remove (struct spi_device *spidev)
{
	struct demo_data *demo ;
	demo = spi_get_drvdata(spidev);
	demo->spi_device = NULL ;
	spi_set_drvdata(spidev , NULL);
	device_destroy(demo_class , demo->devt);
	kfree(demo);
	return 0 ;
}

static struct spi_driver demo_driver = {
	.driver = {
		.name = "spidemo",
		.owner = THIS_MODULE ,	
	},
	.probe = demo_probe ,
	.remove = demo_remove ,
};

static int __init demo_init(void)
{	int status ;
	status = register_chrdev(DEMO_MAJOR , "demo" , &demo_ops);
	if (status < 0 )
		return status ;

	demo_class = class_create(THIS_MODULE , "demo-class");
	if (IS_ERR(demo_class)){
		unregister_chrdev(DEMO_MAJOR , demo_driver.driver.name);
	}

	printk("hello , Kernel-%s()\n", __FUNCTION__);
	return spi_register_driver(&demo_driver);
}

static void __exit demo_exit(void)
{
	printk("Goodbye , Kernel-%s()\n", __FUNCTION__);
	spi_unregister_driver(&demo_driver);
	class_destroy(demo_class);
	unregister_chrdev(DEMO_MAJOR , demo_driver.driver.name);
}

module_init(demo_init);
module_exit(demo_exit);

MODULE_AUTHOR("hubuyu");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:drver_demo");
MODULE_DESCRIPTION("This is spi device driver architecture");
