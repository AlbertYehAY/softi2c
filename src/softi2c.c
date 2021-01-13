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

#define BIT7 (1<<7)
#define BIT6 (1<<6)
#define BIT5 (1<<5)
#define BIT4 (1<<4)
#define BIT3 (1<<3)
#define BIT2 (1<<2)
#define BIT1 (1<<1)
#define BIT0 (1<<0)

static inline void sda_irq(void);
static inline void scl_irq(void);

/*--------------------------------------------------------------*/
struct device *i2c_port[2]={NULL,NULL};
static struct gpio_callback i2c_cb_data[2];

void i2c_irq(struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
    if(pins == SCL_PIN)
    {
        printk("scl_irq at %" PRIu32 "\n", k_cycle_get_32());
        scl_irq();
    }
    else if(pins == SDA_PIN)
    {
        printk("sda_irq at %" PRIu32 "\n", k_cycle_get_32());
        sda_irq();
    }
}

static void softi2c_pin_init(gpio_pin_t pin, uint32_t dir)
{
    int ret=0;

    i2c_port[pin] = device_get_binding(GPIO_IF);
    if(!i2c_port[pin])
        return;    
    if(dir==GPIO_INPUT)
    {
        gpio_pin_configure(i2c_port[pin],pin,dir | GPIO_PULL_UP);
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
    else
    {
        gpio_pin_configure(i2c_port[pin],pin,dir);
    }
}

static void softi2c_pin_init_irq(gpio_pin_t pin,uint32_t flag)
{
    int ret=0;

    if(!i2c_port[pin])
        return;
    ret = gpio_pin_interrupt_configure(i2c_port[pin],
                                       pin,
                                       flag);	
    if (ret != 0) 
    {
        printk("Error %d: failed to configure GPIO_1 pin %d\n",pin);
        return;
    }
    gpio_init_callback(&i2c_cb_data[pin], i2c_irq, BIT(pin));
    gpio_add_callback(i2c_port[pin], &i2c_cb_data[pin]);
}

void softi2c_init()
{
    printf("%s\n",__func__);

    InitBuffer();
    INIT_PORT();
    //softi2c_pin_init(SCL_PIN,GPIO_INPUT);
    //softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_BOTH);
    //softi2c_pin_init(SDA_PIN,GPIO_OUTPUT);
}

void softi2c_test(int value)
{
    //if(i2c_portSCL)
    //    gpio_pin_write(i2c_portSCL,0, value);
    if(i2c_port[SDA_PIN])
    {
        gpio_pin_set(i2c_port[SDA_PIN],SDA_PIN, value);    
    }
}


/************************************************************/

#define SCL BIT0
#define SDA BIT0
#define I2COA 0x0A               // Slave address

#define I2C_START     1          // Start condition
#define I2C_STOP      2          // Stop condition
#define SCL_W1LH      3          // Bit 1 (first clk rising edge)
#define SCL_W1HL      4          // Bit 1 (first clk falling edge)
#define SCL_W2to6LH   5          // Bit 2-6(2nd~6th  clk rising edge)
#define SCL_W7LH      6          // Bit 7 (7th  clk rising edge)
#define SCL_W8LH      7          // Bit 8 (8th  clk rising edge)
#define SCL_W8HL      8          // Set ACK (8th  clk falling edge)
#define SCL_W9HL      9          // Re-init R4 (9th  clk falling edge)
#define SCL_R1HL      10         // Bit 1 (first clk falling edge)
#define SCL_R1LH      11         // Bit 1 (first clk rising edge)
#define SCL_R2to8HL   12         // Bit 2-8 (2nd~8th clk falling edge)
#define SCL_R9HL      13         // Free SDA (9th clk falling edge)
#define SCL_R9LH      14         // Check ACK (9th clk rising edge)

#define MCLK_FREQ_MHZ 8          // MCLK = 8MHz

unsigned char ram_data[16];
unsigned char i2c_data = 0;
unsigned int I2C_state = 0;
unsigned char cnt_2to6=0;
unsigned char cnt_2to8=0;
unsigned int RW_flag=0;
unsigned char ram_cnt=0;

void InitBuffer();
void INIT_PORT();
void Software_Trim();
void InitBuffer()
{
	unsigned char i;
	for(i=0;i<16;i++)
	{
		ram_data[i] = 1;
	}
}

// Set SCL and SDA to inputs
// SDA interrupts on high-to-low transition
// SCL interrupts on low-to-high transition
// initially, just SDA interrupt is set
void INIT_PORT()
{
	//P2OUT &= ~SDA;    // When the pins are set to
        gpio_pin_set(i2c_port[SDA_PIN],SDA_PIN, 0); 

	//P1OUT &= ~SCL;    // output, low level should be seen
        gpio_pin_set(i2c_port[SCL_PIN],SCL_PIN, 0); 

	//P1DIR &= ~SCL;    // SCL and SDA defined as inputs
        softi2c_pin_init(SCL_PIN,GPIO_INPUT);   

	//P2DIR &= ~SDA;
        softi2c_pin_init(SDA_PIN,GPIO_INPUT);
	
        //P2IES |= SDA;     //INT. on high-to-low transition
	softi2c_pin_init_irq(SDA_PIN,GPIO_INT_EDGE_FALLING);

        //P1IES &= ~SCL;    // INT. on low-to-high transition
	softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_RISING);

        //P1IE  &= ~SCL;    // Disable SCL interrupt
        gpio_pin_interrupt_configure(i2c_port[SCL_PIN],
					   SCL_PIN,
					   GPIO_INT_DISABLE);	


        //P2IE  |= SDA;     // Enable SDA interrupt
        gpio_pin_interrupt_configure(i2c_port[SDA_PIN],
                                     SDA_PIN,
                                     GPIO_INT_ENABLE);	

        //P1IFG &= ~SCL;    // Reset interrupt flag
	
        //P2IFG &= ~SDA;

	//PM5CTL0 &= ~LOCKLPM5;  // Disable the GPIO power-on default high-impedance mode
	                       // to activate previously configured port settings

}

/*
https://courses.cs.washington.edu/courses/cse466/11au/calendar/04-Interrupts-posted.pdf
Only transitions (low to hi or hi to low) cause interrupts
ƒÞ P1IFG & P2IFG (Port 1 & 2 Interrupt FlaG registers)
  gpio_get_pending_int(i2c_port[SCL_PIN])  gpio_get_pending_int(i2c_port[SDA_PIN])
  Bit 0: no interrupt pending
  Bit 1: interrupt pending
ƒÞ P1IES & P2IES (Port 1 & 2 Interrupt Edge Select reg)
  Bit 0: PxIFG is set on low to high transition
  Bit 1: PxIFG is set on high to low transition
ƒÞ P1IE & P2IE (Port 1 & 2 Interrupt Enable reg)
  Bit 0: interrupt disabled
  Bit 1: interrupt enabled
*/
static inline void sda_irq(void)
{
      //if((P1IN & SCL == 1) && (P2IFG & SDA == 1) && (P2IES & SDA == 1))
      if(gpio_pin_get(i2c_port[SCL_PIN],SCL_PIN)==1 && 
        (gpio_get_pending_int(i2c_port[SDA_PIN]) & SDA == 1) )
      {
              I2C_state = I2C_START; // I2C start condition
      }
      else
      {
              I2C_state = I2C_STOP;
      }
      //P2IFG = 0;          // Clear INT flag

      switch(I2C_state)
      {
      case I2C_START:
              //P1IFG &= ~SCL;  // P1.0 clear INT flag

              //P1IE |= SCL;    // P1.0 INT enable for SCL
              gpio_pin_interrupt_configure(i2c_port[SCL_PIN],
					   SCL_PIN,
					   GPIO_INT_ENABLE);	

              //P2IES &= ~SDA;  // P2.0 set to rising edge for SDA
              softi2c_pin_init_irq(SDA_PIN,GPIO_INT_EDGE_RISING);

              //P2IFG &= ~SDA;  // P2.0 clear INT flag

              //P2IE &= ~ SDA;  // P2.0 INT disable for SDA
              gpio_pin_interrupt_configure(i2c_port[SCL_PIN],
					   SCL_PIN,
					   GPIO_INT_DISABLE);	

              I2C_state = SCL_W1LH;
              break;
      case I2C_STOP:
              INIT_PORT();
              i2c_data = 0;   // Reset I2CDATA for addr detect
              RW_flag = 0;    // Reset RW_flag for addr detect
              ram_cnt = 0;    // Reset buffer pointer
              break;
      default:
              break;
      }
}

static inline void scl_irq(void)
{
      switch(I2C_state)
      {
      case SCL_W1LH:
              //P1IES |= SCL;      // Set falling edge for SCL
              softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_FALLING);

              //if(P2IN & SDA)     // Check SDA and update i2c_data
              if(gpio_pin_get(i2c_port[SDA_PIN],SDA_PIN)==1)
                      i2c_data = 1;  // Shift SDA bit into the buffer
              else
                      i2c_data = 0;
              //P1IFG &= ~SCL;     // Reset SCL interrupt flag
              //P2IFG &= ~SDA;     // Clear flag, FOR STP_CON DETECT
              
              //P2IE |= SDA;       // Enable SDA INT
              gpio_pin_interrupt_configure(i2c_port[SDA_PIN],
					   SDA_PIN,
					   GPIO_INT_ENABLE);	
              I2C_state = SCL_W1HL;
              break;
      case SCL_W1HL:
              //P1IES &= ~ SCL;    // Set rising edge for SCL
              //P1IFG &= ~SCL;     // Reset SCL interrupt flag
              softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_RISING);

              //P2IE &= ~SDA;      // 1st FALLING SCL TO disable SDA INT
              gpio_pin_interrupt_configure(i2c_port[SDA_PIN],
					   SDA_PIN,
					   GPIO_INT_DISABLE);	

              I2C_state = SCL_W2to6LH;
              break;
      case SCL_W2to6LH:
              //P1IFG &= ~SCL;     // Reset interrupt flag
              
              i2c_data <<= 1;
              
              //if(P2IN & SDA)     // Check SDA and and update i2c_data
              if(gpio_pin_get(i2c_port[SDA_PIN],SDA_PIN)==1)
                    i2c_data++;
              if(cnt_2to6 < 4)   // 5bits(bit2 to bit6)
                      cnt_2to6++;
              else
              {
                      cnt_2to6 = 0;  // Reset counter
                      I2C_state = SCL_W7LH;
              }
              break;
      case SCL_W7LH:
              //P1IFG &= ~SCL;     // Reset interrupt flag
              
              i2c_data <<= 1;
              
              //if(P2IN & SDA)     // Check SDA and and update i2c_data
              if(gpio_pin_get(i2c_port[SDA_PIN],SDA_PIN)==1)
              {
                      i2c_data++;
              }
              if((RW_flag == 0) && (i2c_data != I2COA) )
                      I2C_state = I2C_STOP;
              else
                      I2C_state = SCL_W8LH;
              break;
      case SCL_W8LH:
              //P1IES |= SCL;        // Set falling edge for SCL.
              softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_FALLING);

              //P1IFG &= ~SCL;       // Reset interrupt flag

              if(RW_flag == 0)
              {
                      //if(P2IN & SDA)   // Check SDA and and update i2c_data
                      if(gpio_pin_get(i2c_port[SDA_PIN],SDA_PIN)==1)
                            RW_flag = 1; // Read CMD
                      else
                            RW_flag = 2; // Write CMD
              }
              else
              {
                      i2c_data <<= 1;
                      //if(P2IN & SDA)   // Check SDA and and update i2c_data
                      if(gpio_pin_get(i2c_port[SDA_PIN],SDA_PIN)==1)
                          i2c_data++;
              }
              I2C_state = SCL_W8HL;
              break;
      case SCL_W8HL:
              //P2DIR |= SDA;        // Output "0" for SDA
              softi2c_pin_init(SDA_PIN,GPIO_OUTPUT);   

              //P1IFG &= ~SCL;       //Reset SCL interrupt flag

              if(RW_flag == 1)
              {
                      i2c_data = ram_data[ram_cnt];
                      ram_cnt++;
                      ram_cnt &= 0x0f;
                      I2C_state = SCL_R1HL;// Go to read state
              }
              else if(RW_flag == 2)
              {
                      RW_flag = 3;
                      I2C_state = SCL_W9HL;// Go to write state
              }
              else if(RW_flag == 3)
              {
                      ram_data[ram_cnt] = i2c_data;
                      ram_cnt++;
                      ram_cnt &= 0x0f;
                      I2C_state = SCL_W9HL;// Go to write state
              }
              else
                      I2C_state = I2C_STOP;
              break;
      case SCL_W9HL:
              //P2DIR &= ~SDA;        // Release SDA
              softi2c_pin_init(SDA_PIN,GPIO_INPUT);   

              //P1IES &= ~SCL;        // Set rising edge for SCL
              softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_RISING);

              //P1IFG &= ~SCL;        // Reset interrupt flag
              I2C_state = SCL_W1LH;
              break;
      case SCL_R1HL:
              //P1IES &= ~SCL;        // Set rising edge SCL for SCL_R1LH
              softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_RISING);

              if(i2c_data & BIT7)
                    //P2DIR &= ~SDA;    // SDA = 1
                    softi2c_pin_init(SDA_PIN,GPIO_INPUT);   
              else
                    //P2DIR |= SDA;     // SDA = 0
                    softi2c_pin_init(SDA_PIN,GPIO_OUTPUT);   
              i2c_data <<= 1;
              //P1IFG &= ~ SCL;       // Clear SCL INT flag
              
              //P2IE  &= ~ SDA;       // Disable SDA INT, NO STP CON
              gpio_pin_interrupt_configure(i2c_port[SDA_PIN],
					   SDA_PIN,
					   GPIO_INT_DISABLE);	
              I2C_state = SCL_R1LH;
              break;
      case SCL_R1LH:
              //P1IES |= SCL;         // Set falling edge for SCL
              softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_FALLING);

              //P1IFG &= ~SCL;        // Clear SCL INT flag
              
              //P2IFG &= ~SDA;        // Clear SDA INT Flag

              //P2IE |= SDA;          // Enable SDA INT for STOP CON
              gpio_pin_interrupt_configure(i2c_port[SDA_PIN],
					   SDA_PIN,
					   GPIO_INT_ENABLE);	
              I2C_state = SCL_R2to8HL;
              break;
      case SCL_R2to8HL:
              if(i2c_data & BIT7)
                    //P2DIR &= ~SDA;    // SDA = 1
                    softi2c_pin_init(SDA_PIN,GPIO_INPUT);   
              else
                    //P2DIR |= SDA;     // SDA = 0
                    softi2c_pin_init(SDA_PIN,GPIO_OUTPUT);   
              i2c_data <<= 1;
              
              //P1IFG &= ~ SCL;       // Clear SCL INT flag
              
              //P2IE  &= ~ SDA;       // Disable SDA INT, NO STP CON
              gpio_pin_interrupt_configure(i2c_port[SDA_PIN],
					   SDA_PIN,
					   GPIO_INT_DISABLE);	

              if(cnt_2to8 < 6)
              {
                      cnt_2to8++;       // 7bits(bit2 to bit8)
              }
              else
              {
                      cnt_2to8 = 0;     // Clear counter
                      I2C_state = SCL_R9HL;
              }
              break;
      case SCL_R9HL:
              //P2DIR &= ~SDA;        // Release SDA

              //P1IES &= ~SCL;        // Set Rising edge of SCL
              softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_RISING);

              //P1IFG &= ~SCL;        // Clear SCL INT flag

              I2C_state = SCL_R9LH;
              break;
      case SCL_R9LH:
              //if(P2IN & SDA)        // NACK_READ
              //if(gpio_pin_get(i2c_port[SDA_PIN],SDA_PIN))
              if(gpio_pin_get(i2c_port[SDA_PIN],SDA_PIN))
              {
                      //P1IES &= ~SCL;    // Set rising edge of SCL
                      softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_RISING);
                      
                      //P1IFG &= ~SCL;    // Clear SCL INT flag

                      I2C_state = SCL_W1LH; // Go to SCL_W1LH to detect stop condition
              }
              else                  // ACK_READ
              {
                      //P1IES |= SCL;     // Set falling edge of SCL
                      softi2c_pin_init_irq(SCL_PIN,GPIO_INT_EDGE_FALLING);

                      //P1IFG &= ~SCL;    // Clear SCL INT flag
                      i2c_data = ram_data[ram_cnt]; // Move out next byte from RAM buffer
                      ram_cnt++;
                      ram_cnt &= 0x0f;
                      I2C_state = SCL_R1HL;// Go to read state
              }
              break;
      case I2C_STOP:
              INIT_PORT();
              i2c_data = 0;   // Reset I2CDATA for addr detect
              RW_flag = 0;    // Reset RW_flag for addr detect
              ram_cnt = 0;    // Reset buffer pointer
              break;
      default:
              break;
      }
}
