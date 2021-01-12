#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <stdio.h>

#define GPIO_IF   "GPIO_1"
#define SCL_PIN 0
#define SDA_PIN 1

struct device *i2c_port[2]={NULL,NULL};
static struct gpio_callback i2c_cb_data[2];

void i2c_irq(struct device *dev, struct gpio_callback *cb,
		    u32_t pins)
{
    if(pins == 0)
        printk("scl_irq at %" PRIu32 "\n", k_cycle_get_32());
    if(pins == 1)
        printk("sda_irq at %" PRIu32 "\n", k_cycle_get_32());
}

static void softi2c_pin_init(gpio_pin_t pin, u32_t dir)
{
    int ret=0;

    i2c_port[pin] = device_get_binding(GPIO_IF);
    if(!i2c_port[pin])
        return;
    gpio_pin_configure(i2c_port[pin],pin,dir);
    if(dir==GPIO_DIR_IN)
    {
	ret = gpio_pin_interrupt_configure(i2c_port[pin],
					   pin,
					   GPIO_INT_EDGE_BOTH);	
        if (ret != 0) 
        {
            printk("Error %d: failed to configure GPIO_1 pin %d\n",pin);
            return;
        }
        gpio_init_callback(&i2c_cb_data[pin], i2c_irq, BIT(pin));
        gpio_add_callback(i2c_port[pin], &i2c_cb_data[pin]);
    }
}

void softi2c_init()
{
    printf("%s\n",__func__);

    softi2c_pin_init(SCL_PIN,GPIO_DIR_IN);
    softi2c_pin_init(SDA_PIN,GPIO_DIR_OUT);
}

void softi2c_test(int value)
{
    //if(i2c_portSCL)
    //    gpio_pin_write(i2c_portSCL,0, value);
    if(i2c_port[SDA_PIN])
        gpio_pin_write(i2c_port[SDA_PIN],1, value);
}