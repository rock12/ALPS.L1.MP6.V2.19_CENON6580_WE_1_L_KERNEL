#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <mach/mt_pwm.h>
#include <mach/mt_pwm_hal.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>

#include "mt_irtx.h"
#include <linux/regulator/consumer.h>


#define IRTX_GPIO_FREQ 0x80000000|GPIO67

struct regulator* g_irtx_reg = NULL;
struct mt_irtx mt_irtx_dev;
unsigned int irtx_irq;

struct pwm_spec_config irtx_pwm_config = {
    .pwm_no = 0,
    .mode = PWM_MODE_MEMORY,
    .clk_div = CLK_DIV1,
    .clk_src = PWM_CLK_NEW_MODE_BLOCK,
    .pmic_pad = 0,
    .PWM_MODE_MEMORY_REGS.IDLE_VALUE = IDLE_FALSE,
    .PWM_MODE_MEMORY_REGS.GUARD_VALUE = GUARD_FALSE,
    .PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31,
    .PWM_MODE_MEMORY_REGS.HDURATION = 25, // 1 microseconds, assume clock source is 26M
    .PWM_MODE_MEMORY_REGS.LDURATION = 25,
    .PWM_MODE_MEMORY_REGS.GDURATION = 0,
    .PWM_MODE_MEMORY_REGS.WAVE_NUM = 1,
};


static int dev_char_open(struct inode *inode, struct file *file)
{

    if(atomic_read(&mt_irtx_dev.usage_cnt))
        return -EBUSY;

    pr_err("[IRTX] open by %s\n", current->comm);
    nonseekable_open(inode,file);
    atomic_inc(&mt_irtx_dev.usage_cnt);
    return 0;
}

static int dev_char_close(struct inode *inode, struct file *file)
{
    pr_warning("[IRTX] close by %s\n", current->comm);
    atomic_dec(&mt_irtx_dev.usage_cnt);
    return 0;
}

static ssize_t dev_char_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    return 0;
}

static long dev_char_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned int para = 0, gpio_id = -1, en = 0;
    unsigned long mode = 0, dir = 0, outp = 0;

    switch(cmd) {
        case IRTX_IOC_SET_CARRIER_FREQ:
            if(copy_from_user(&mt_irtx_dev.carrier_freq, (void __user *)arg, sizeof(unsigned int))) {
                pr_err("[IRTX] IRTX_IOC_SET_CARRIER_FREQ: copy_from_user fail!\n");
                ret = -EFAULT;
            } else {
                pr_err("[IRTX] IRTX_IOC_SET_CARRIER_FREQ: %d\n", mt_irtx_dev.carrier_freq);
                if(!mt_irtx_dev.carrier_freq) {
                    ret = -EINVAL;
                    mt_irtx_dev.carrier_freq = 38000;
                }
            }
        break;

        case IRTX_IOC_SET_IRTX_LED_EN:
            if(copy_from_user(&para, (void __user *)arg, sizeof(unsigned int))) {
                pr_err("[IRTX] IRTX_IOC_SET_IRTX_LED_EN: copy_from_user fail!\n");
                ret = -EFAULT;
            } else {
                // en: bit 12;
                // gpio: bit 0-11
                gpio_id = (unsigned long)((para & 0x0FFF0000) > 16);
                en = (para & 0xF);
                pr_err("[IRTX] IRTX_IOC_SET_IRTX_LED_EN: 0x%x, gpio_id:%ul, en:%ul\n", para, gpio_id, en);
                if (en) {
                    mode = GPIO_MODE_02;
                    dir = GPIO_DIR_OUT;
                    outp = GPIO_OUT_ONE;    // high means enable LED
                } else {
                    mode = GPIO_MODE_00;
                    dir = GPIO_DIR_OUT;
                    outp = GPIO_OUT_ZERO;  // low means disable LED
                }
                //gpio_id = IRTX_GPIO;

                mt_set_gpio_mode(mt_irtx_dev.gpio_no, mode);
                mt_set_gpio_dir(mt_irtx_dev.gpio_no, dir);
                mt_set_gpio_out(mt_irtx_dev.gpio_no, outp);

            }

        break;

        default:
            pr_err("[IRTX] unknown ioctl cmd 0x%x\n", cmd);
            ret = -ENOTTY;
            break;
    }
    return ret;
}


static ssize_t dev_char_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    dma_addr_t wave_phy;
    void *wave_vir;
    int ret;
    int buf_size = (count + 3) / 4;  // when count is 5...

    pr_err("[IRTX] irtx write len=0x%x, pwm=%d\n", (unsigned int)count, (unsigned int)irtx_pwm_config.pwm_no);
    wave_vir = dma_alloc_coherent(&mt_irtx_dev.plat_dev->dev, count, &wave_phy, GFP_KERNEL);
    if(!wave_vir) {
        pr_err("[IRTX] alloc memory fail\n");
        return -ENOMEM;
    }
    ret = copy_from_user(wave_vir, buf, count);
    if(ret) {
        pr_err("[IRTX] write, copy from user fail %d\n", ret);
        goto exit;
    }
    
    mt_set_intr_enable(0);
    mt_set_intr_enable(1);
    mt_pwm_26M_clk_enable_hal(1);
	
    irtx_pwm_config.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = wave_phy;
    irtx_pwm_config.PWM_MODE_MEMORY_REGS.BUF0_SIZE = (buf_size ? (buf_size -1) : 0);

    mt_set_gpio_mode(mt_irtx_dev.gpio_no, mt_irtx_dev.pwm_mode);
    
    mt_set_intr_ack(0);
    mt_set_intr_ack(1);
    ret = pwm_set_spec_config(&irtx_pwm_config);
    pr_err("[IRTX] pwm is triggered, %d\n", ret);

	msleep(count * 8 / 1000);
    msleep(100);
    ret = count;

	//mt_pwm_dump_regs();

exit:
    pr_err("[IRTX] done, clean up\n");
    dma_free_coherent(&mt_irtx_dev.plat_dev->dev, count, wave_vir, wave_phy);
    mt_pwm_disable(irtx_pwm_config.pwm_no, irtx_pwm_config.pmic_pad);

#ifndef IRTX_GPIO_OUT_ZERO
	mt_set_gpio_mode(mt_irtx_dev.gpio_no, GPIO_MODE_00);
	mt_set_gpio_dir(mt_irtx_dev.gpio_no, GPIO_DIR_OUT);
	mt_set_gpio_out(mt_irtx_dev.gpio_no, GPIO_OUT_ONE);
#else
	mt_set_gpio_mode(mt_irtx_dev.gpio_no, GPIO_MODE_00);
	mt_set_gpio_dir(mt_irtx_dev.gpio_no, GPIO_DIR_OUT);
	mt_set_gpio_out(mt_irtx_dev.gpio_no, GPIO_OUT_ZERO);
#endif
	
#ifndef IRTX_ONE_GPIO_CONTROL
	mt_set_gpio_mode(IRTX_GPIO_FREQ, GPIO_MODE_00);
	mt_set_gpio_dir(IRTX_GPIO_FREQ, GPIO_DIR_OUT);
	mt_set_gpio_out(IRTX_GPIO_FREQ, GPIO_OUT_ONE);
#endif

    return ret;
}

static irqreturn_t irtx_isr(int irq, void *data)
{
    return IRQ_HANDLED;
}

#define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
static u64 irtx_dma_mask = DMA_BIT_MASK((sizeof(unsigned long)<<3)); // TODO: 3?

static struct file_operations char_dev_fops = {
    .owner = THIS_MODULE,
    .open = &dev_char_open,
    .read = &dev_char_read,
    .write = &dev_char_write,
    .release = &dev_char_close,
    .unlocked_ioctl = &dev_char_ioctl,
};

#define irtx_driver_name "mt_irtx"
static int irtx_probe(struct platform_device *plat_dev)
{
    struct cdev *c_dev;
    dev_t dev_t_irtx;
    struct device *dev = NULL;
    static void *dev_class;
    u32 major = 0, minor = 0;
    int ret = 0;
	unsigned int gpio_id_freq = -1;
    int error = 0;

	gpio_id_freq = IRTX_GPIO_FREQ;

#ifdef CONFIG_OF
    if(plat_dev->dev.of_node == NULL) {
        pr_err("[IRTX] irtx OF node is NULL\n");
        return 0;
    }

    of_property_read_u32(plat_dev->dev.of_node, "major", &major);
    mt_irtx_dev.irq = irq_of_parse_and_map(plat_dev->dev.of_node, 0);
	
    of_property_read_u32(plat_dev->dev.of_node, "pwm_ch", &mt_irtx_dev.pwm_ch);
	of_property_read_u32_index(plat_dev->dev.of_node, "gpio_pwm", 0, &mt_irtx_dev.gpio_no);
	of_property_read_u32_index(plat_dev->dev.of_node, "gpio_pwm", 1, &mt_irtx_dev.pwm_mode);

	//of_property_read_u32(plat_dev->dev.of_node, "pwm_freq", &mt_irtx_dev.pwm_freq);
    pr_err("[IRTX] device tree info: major=%d irq=%d pwm=%d gpio_no=0x%x pwm_mode = 0x%x\n", 
        major, mt_irtx_dev.irq, mt_irtx_dev.pwm_ch, mt_irtx_dev.gpio_no, mt_irtx_dev.pwm_mode);
#endif

    if (!major) {
        error = alloc_chrdev_region(&dev_t_irtx, 0, 1, irtx_driver_name);
		pr_err("[IRTX] alloc_chrdev_region error = %d \n", error);
        if (!error) {
            major = MAJOR(dev_t_irtx);
            minor = MINOR(dev_t_irtx);
			pr_err("[IRTX] alloc_chrdev_region major = %d minor = %d \n", major, minor);
        }
    } else {
        dev_t_irtx = MKDEV(major, minor);
    }

    ret = request_irq(mt_irtx_dev.irq, irtx_isr, IRQF_TRIGGER_FALLING, "IRTX", NULL); // TODO: trigger
    if(ret) {
        pr_err("[IRTX] request IRQ(%d) fail ret=%d\n", mt_irtx_dev.irq, ret);
        goto exit;
    }
    irtx_pwm_config.pwm_no = mt_irtx_dev.pwm_ch;
	//irtx_freq_pwm_config.pwm_no = mt_irtx_dev.pwm_freq;

	pr_err("[IRTX] PWM: pwm_no=%d \n", irtx_pwm_config.pwm_no);

    mt_irtx_dev.plat_dev = plat_dev;
    mt_irtx_dev.plat_dev->dev.dma_mask = &irtx_dma_mask;
    mt_irtx_dev.plat_dev->dev.coherent_dma_mask = irtx_dma_mask;
    atomic_set(&mt_irtx_dev.usage_cnt, 0);
    mt_irtx_dev.carrier_freq = 38000; // NEC as default

    ret = register_chrdev_region(dev_t_irtx, 1, irtx_driver_name);
    if(ret) {
        pr_err("[IRTX] register_chrdev_region fail ret=%d\n", ret);
        goto exit;
    }
    c_dev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
    if(!c_dev) {
        pr_err("[IRTX] kmalloc cdev fail\n");
        goto exit;
    }
    cdev_init(c_dev, &char_dev_fops);
    c_dev->owner = THIS_MODULE;
    ret = cdev_add(c_dev, dev_t_irtx, 1);
    if(ret) {
        pr_err("[IRTX] cdev_add fail ret=%d\n", ret);
        goto exit;
    }
    dev_class = class_create(THIS_MODULE, irtx_driver_name);
    dev = device_create(dev_class, NULL, dev_t_irtx, NULL, "irtx");
    if(IS_ERR(dev)) {
        ret = PTR_ERR(dev);
        pr_err("[IRTX] device_create fail ret=%d\n", ret);
        goto exit;
    }

#ifndef IRTX_GPIO_OUT_ZERO
	mt_set_gpio_mode(mt_irtx_dev.gpio_no, GPIO_MODE_00);
	mt_set_gpio_dir(mt_irtx_dev.gpio_no, GPIO_DIR_OUT);
	mt_set_gpio_out(mt_irtx_dev.gpio_no, GPIO_OUT_ONE);
#else
	mt_set_gpio_mode(mt_irtx_dev.gpio_no, GPIO_MODE_00);
	mt_set_gpio_dir(mt_irtx_dev.gpio_no, GPIO_DIR_OUT);
	mt_set_gpio_out(mt_irtx_dev.gpio_no, GPIO_OUT_ZERO);
#endif

#ifndef IRTX_ONE_GPIO_CONTROL
	mt_set_gpio_mode(gpio_id_freq, GPIO_MODE_00);
	mt_set_gpio_dir(gpio_id_freq, GPIO_DIR_OUT);
	mt_set_gpio_out(gpio_id_freq, GPIO_OUT_ONE);
#endif

exit:
    pr_warning("[IRTX] irtx probe ret=%d\n", ret);
    return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id irtx_of_ids[] = {
    {.compatible = "mediatek,IRTX", },
    {}
};
#endif

static struct platform_driver irtx_driver =
{
    .driver = {
        .name = "mt_irtx",
    },
    .probe = irtx_probe,
};

static int __init irtx_init(void)
{
    int ret = 0;
    
    pr_warning("[IRTX] irtx init\n");
#ifdef CONFIG_OF
    irtx_driver.driver.of_match_table = irtx_of_ids;
#else
    pr_err("[IRTX] irtx needs device tree!\n");
    //BUG_ON(1);
#endif

    ret = platform_driver_register(&irtx_driver);
    if (ret) {
        pr_err("[IRTX] irtx platform driver register fail %d\n", ret);
        goto exit;
    }

exit:
    return ret;
}


late_initcall(irtx_init);

MODULE_AUTHOR("Xiaolei Yin <xiaolei.yin@mediatek.com>");
MODULE_DESCRIPTION("Consumer IR transmitter driver v0.1");
