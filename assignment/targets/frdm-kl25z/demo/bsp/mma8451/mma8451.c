#include <MKL25Z4.h>
#include <math.h>

#include "mma8451.h"
#include "bsp.h"

#if (CLOCK_SETUP != 1)
  #warning This driver does not work as designed
#endif

#define M_PI (3.14159265f)

int16_t x_out_14_bit = 0,
        y_out_14_bit = 0,
        z_out_14_bit = 0;
float x_out_mg = 0,
      y_out_mg = 0,
      z_out_mg = 0;

bool mma8451_ready_flag = false;
float dt = 0;

// Local function prototypes
void pit_init(void);
    
bool mma8451_init(void)
{
    i2c0_init();
	
    uint8_t value;

    // Read the WHO_AM_I_REG register
    if(!(i2c0_read_byte(MMA8451_ADDRESS, WHO_AM_I_REG, &value)))
    {
        return false;
    }
    
    // Check for device
    if(value != WHO_AM_I_VAL)
    {
        return false;
    }

    // Reset all register to POR values
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG2, 0x40)))
    {
        return false;
    }

    // Wait for RST bit to clear
    value = 0;
    do
    {           
        // Read the register
        if(!(i2c0_read_byte(MMA8451_ADDRESS, CTRL_REG2, &value)))
        {
            return false;
        }
    }
    while((value & 0x40) == 0);

    // +/-2g range -> 1g = 16384/4 = 4096 counts 
    if(!(i2c0_write_byte(MMA8451_ADDRESS, XYZ_DATA_CFG_REG, 0x00)))
    {
        return false;
    }
    
    // High Resolution mode
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG2, 0x02)))
    {
        return false;
    }
                       
    // ODR = 100 Hz, Reduced noise, Active mode
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG1, 0x1D)))
    {
        return false;
    }
    
	// Configure the PTA14, connected to the INT1 of the MMA8451Q, for 
    // interrupts on falling edges
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	PORTA->PCR[14] |= PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x1)| PORT_PCR_IRQC(0xA);
    
    NVIC_SetPriority(PORTA_IRQn, 0); // 0, 64, 128 or 192
    NVIC_ClearPendingIRQ(PORTA_IRQn); 
    NVIC_EnableIRQ(PORTA_IRQn);

    return true;
}

bool mma8451_calibrate(void)
{
    uint8_t value = 0;
    
    // Wait for data
    do
    {
        // Read the status register
        if(!(i2c0_read_byte(MMA8451_ADDRESS, STATUS_REG, &value)))
        {
            return false;
        }       
    }
    while((value & 0x08) == 0);
        
    // Read values
    mma8451_read();
  
    // Calculate offsets as described in AN4069
    signed char x_offset = (signed char)(-1 * (x_out_14_bit >> 3));
    signed char y_offset = (signed char)(-1 * (y_out_14_bit >> 3));
    signed char z_offset = (signed char)(COUNTS_PER_G - (z_out_14_bit >> 3));
        
    // Standby mode
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG1, 0x00)))
    {
        return false;
    } 
    
    // Offsets
    if(!(i2c0_write_byte(MMA8451_ADDRESS, OFF_X_REG, x_offset)))
    {
        return false;
    }
    
    if(!(i2c0_write_byte(MMA8451_ADDRESS, OFF_Y_REG, y_offset)))
    {
        return false;
    }
    
    if(!(i2c0_write_byte(MMA8451_ADDRESS, OFF_Z_REG, z_offset)))
    {
        return false;
    }
    
    // Push-pull, active low interrupt 
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG3, 0x00)))
    {
        return false;
    }
    
    // Enable DRDY interrupt
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG4, 0x01)))
    {
        return false;
    }
    
    // DRDY interrupt routed to INT1 - PTA14
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG5, 0x01)))
    {
        return false;
    }
    
    // ODR = 100 Hz, Reduced noise, Active mode
    // Notice that delta dt is fixed, because it is set by ODR and DRDY
    // interrupts are enabled. This doesn't require a timer to measure delta t.
    dt = 0.01f;
    if(!(i2c0_write_byte(MMA8451_ADDRESS, CTRL_REG1, 0x1D)))
    {
        return false;
    }
    
    return true;
}

void mma8451_read(void)
{
	int i;
	uint8_t data[6];

    i2c0_start();
    
	if(!(i2c0_read_setup(MMA8451_ADDRESS, OUT_X_MSB_REG)))
    {
        mma8451_init();
        return;
    }
	
	// Read five bytes in repeated mode
	for(i=0; i<5; i++)
    {        
        if(!(i2c0_repeated_read(false, &data[i])))
        {
            mma8451_init();
            return;
        }
	}
        
	// Read last byte ending repeated mode
	if(!(i2c0_repeated_read(true, &data[i])))
    {
        mma8451_init();
        return;
    }
	
    // Combine the read bytes to 16-bit values
    x_out_14_bit = (int16_t)((data[0]<<8) | data[1]);
    y_out_14_bit = (int16_t)((data[2]<<8) | data[3]);
    z_out_14_bit = (int16_t)((data[4]<<8) | data[5]);
    
    // Compute 14-bit signed result
    x_out_14_bit >>= 2;
    y_out_14_bit >>= 2;
    z_out_14_bit >>= 2;
    
    // Compute result in mg's
    x_out_mg = ((float)x_out_14_bit / COUNTS_PER_G) * 1000;
    y_out_mg = ((float)y_out_14_bit / COUNTS_PER_G) * 1000;
    z_out_mg = ((float)z_out_14_bit / COUNTS_PER_G) * 1000;
}

void PORTA_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    
    // Clear the interrupt
    PORTA->PCR[14] |= PORT_PCR_ISF_MASK;

    mma8451_ready_flag = 1;	
}
