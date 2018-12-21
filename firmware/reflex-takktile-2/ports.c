#include "ports.h"

// GLOBAL VARIABLES
volatile ports_t handPorts;

void portsInit()
{
	printf("initializing ports ...\n");
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
	              RCC_AHB1ENR_GPIOBEN |
	              RCC_AHB1ENR_GPIOCEN |
	              RCC_AHB1ENR_GPIODEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN |
	              RCC_APB1ENR_I2C3EN |
	              RCC_APB1ENR_SPI2EN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	pin_set_alternate_function(GPIOB, PORTB_I2C1_SCL, 4);
	pin_set_alternate_function(GPIOB, PORTB_I2C1_SDA, 4);
	pin_set_output_type(GPIOB, PORTB_I2C1_SCL, PIN_OUTPUT_TYPE_OPEN_DRAIN);
	pin_set_output_type(GPIOB, PORTB_I2C1_SDA, PIN_OUTPUT_TYPE_OPEN_DRAIN);

	pin_set_alternate_function(GPIOA, PORTA_I2C3_SCL, 4);
	pin_set_alternate_function(GPIOC, PORTC_I2C3_SDA, 4);
	pin_set_output_type(GPIOA, PORTA_I2C3_SCL, PIN_OUTPUT_TYPE_OPEN_DRAIN);
	pin_set_output_type(GPIOC, PORTC_I2C3_SDA, PIN_OUTPUT_TYPE_OPEN_DRAIN);

	//I2C1->CCR |= I2C_CCR_FS | // set fast mode
	//             35; // 42 MHz / (3 * 400 kHz) == 35
	//I2C1->TRISE = 42 * 300 / 1000 + 1; // not sure here.
	I2C1->CR2   |= APB_MHZ; // 42MHz -> 101010
	I2C1->CCR   |= I2C_CCR; // I2C_CCR = 210
	I2C1->TRISE &= ~0x3f; // 0x3f = 0b00111111 -> 0b11000000, clearing register
	I2C1->TRISE |= I2C_TRISE; // = I2C_TRISE = (APB_MHZ * 200 / 1000 + 1), maximum rise time
	I2C1->CR1   |= I2C_CR1_PE; // peripheral enable

	I2C3->CR2   |= APB_MHZ;
	I2C3->CCR   |= I2C_CCR;
	I2C3->TRISE &= ~0x3f;
	I2C3->TRISE |= I2C_TRISE;
	I2C3->CR1   |= I2C_CR1_PE;

	// now, set up the spi-to-i2c bridges
	pin_set_output(GPIOC, PORTC_I2C_BRIDGE_RESET);
	pin_set_output(GPIOA, PORTA_BRIDGE0_CS);
	pin_set_output(GPIOB, PORTB_BRIDGE1_CS);
	pin_set_output_level(GPIOA, PORTA_BRIDGE0_CS, 1);
	pin_set_output_level(GPIOA, PORTB_BRIDGE1_CS, 1);

	pin_set_alternate_function(GPIOA, PORTA_BRIDGE0_MISO, 5);
	pin_set_alternate_function(GPIOB, PORTB_BRIDGE0_MOSI, 5);
	pin_set_alternate_function(GPIOA, PORTA_BRIDGE0_SCLK, 5);
	pin_set_alternate_function(GPIOC, PORTC_BRIDGE1_MISO, 5);
	pin_set_alternate_function(GPIOC, PORTC_BRIDGE1_MOSI, 5);
	pin_set_alternate_function(GPIOD, PORTD_BRIDGE1_SCLK, 5);

	// SPI1
	// running from a 84 MHz pclk. set it up with
	// sclk = pclk/64 to stay within datasheet limits.
	SPI1->CR1 = SPI_CR1_BR_2 | // sclk = pclk/64
	          SPI_CR1_BR_0 | // sclk = pclk/64
	          SPI_CR1_MSTR | // master mode
	          SPI_CR1_CPOL | // clock to 1 when idle
	          SPI_CR1_CPHA | // the second clock transition is the first data capture edge
	          SPI_CR1_SSM  | // software slave-select mode
	          SPI_CR1_SSI  | // the value of this bit is forced onto the NSS pin and the IO value of the NSS pin is ignored.
	          SPI_CR1_SPE;   // SPI enable

	// bit rate = 42 mhz / 32 = 1.313 MHz
	SPI2->CR1 = SPI_CR1_BR_2 | // sclk = pclk/32
	          SPI_CR1_MSTR | // master mode
	          SPI_CR1_CPOL | // clock to 1 when idle
	          SPI_CR1_CPHA | // the second clock transition is the first data capture edge
	          SPI_CR1_SSM  | // software slave-select mode
	          SPI_CR1_SSI  | // the value of this bit is forced onto the NSS pin and the IO value of the NSS pin is ignored.
	          SPI_CR1_SPE;   // SPI enable


	// to do: automatically find ports of each finger/encoder
	// for (int i = 0; i < NUM_FINGERS; ++i)
	// {
	// 	handPorts.takktile[i] = ...
	// }



	 //Deprecated code for automatically determining IMU connection configuration
	/*if(writeRegisterI2C((uint32_t*) I2C1_BASE, I2C_MULTIPLEXER_ADDRESS, 0))
	{
		printf("\tMultiplexer on port 1, imus on multiplexer port 0, 1, 2, 3\n");
		handPorts.multiplexer = 1;
		handPorts.imu[0] = (uint32_t*) I2C1_BASE; // finger 1
    	handPorts.imu[1] = (uint32_t*) I2C1_BASE; // finger 2
    	handPorts.imu[2] = (uint32_t*) I2C1_BASE; // finger 3
    	handPorts.imu[3] = (uint32_t*) I2C1_BASE; // palm

    	handPorts.imuI2CAddress[0] = BNO055_ADDRESS_A; // finger 1
    	handPorts.imuI2CAddress[1] = BNO055_ADDRESS_A; // finger 2
    	handPorts.imuI2CAddress[2] = BNO055_ADDRESS_A; // finger 3
    	handPorts.imuI2CAddress[3] = BNO055_ADDRESS_A; // palm
	}
    else if (writeRegisterI2C((uint32_t*) I2C3_BASE, I2C_MULTIPLEXER_ADDRESS, 0))
    {
    	printf("\tMultiplexer on port 3, imus on multiplexer port 0, 1, 2, 3\n");
    	handPorts.multiplexer = 3;
    	handPorts.imu[0] = (uint32_t*) I2C3_BASE; // finger 1
    	handPorts.imu[1] = (uint32_t*) I2C3_BASE; // finger 2
    	handPorts.imu[2] = (uint32_t*) I2C3_BASE; // finger 3
    	handPorts.imu[3] = (uint32_t*) I2C3_BASE; // palm

    	handPorts.imuI2CAddress[0] = BNO055_ADDRESS_A; // finger 1
    	handPorts.imuI2CAddress[1] = BNO055_ADDRESS_A; // finger 2
    	handPorts.imuI2CAddress[2] = BNO055_ADDRESS_A; // finger 3
    	handPorts.imuI2CAddress[3] = BNO055_ADDRESS_A; // palm
    }
    else //default: (writeRegisterI2C((uint32_t*) I2C1_BASE, BNO055_ADDRESS_A, 0) && writeRegisterI2C((uint32_t*) I2C1_BASE, BNO055_ADDRESS_B) && writeRegisterI2C((uint32_t*) I2C3_BASE, BNO055_ADDRESS_A, 0) && writeRegisterI2C((uint32_t*) I2C3_BASE, BNO055_ADDRESS_B))
    {
    	printf("\tNo multiplexer, imus on 1 (0x28), 1 (0x29), 3 (0x28), 3 (0x29)\n");
    	handPorts.multiplexer = 0;
    	handPorts.imu[0] = (uint32_t*) I2C1_BASE; // finger 1
    	handPorts.imu[1] = (uint32_t*) I2C1_BASE; // finger 2
    	handPorts.imu[2] = (uint32_t*) I2C3_BASE; // finger 3
    	handPorts.imu[3] = (uint32_t*) I2C3_BASE; // palm

    	handPorts.imuI2CAddress[0] = BNO055_ADDRESS_A; // finger 1
    	handPorts.imuI2CAddress[1] = BNO055_ADDRESS_B; // finger 2
    	handPorts.imuI2CAddress[2] = BNO055_ADDRESS_A; // finger 3
    	handPorts.imuI2CAddress[3] = BNO055_ADDRESS_B; // palm
    }*/

	printf("\tNo multiplexer, imus on 1 (0x28), 2 (0x28), 3 (0x28), 3 (0x29)\n");
	handPorts.multiplexer = 0;
	handPorts.imu[0] = (uint32_t*) I2C1_BASE; // finger 1
	handPorts.imu[1] = (uint32_t*) SPI1_BASE; // finger 2
	handPorts.imu[2] = (uint32_t*) I2C3_BASE; // finger 3
	handPorts.imu[3] = (uint32_t*) I2C3_BASE; // palm

	handPorts.imuI2CAddress[0] = BNO055_ADDRESS_A; // finger 1
	handPorts.imuI2CAddress[1] = BNO055_ADDRESS_A; // finger 2
	handPorts.imuI2CAddress[2] = BNO055_ADDRESS_A; // finger 3
	handPorts.imuI2CAddress[3] = BNO055_ADDRESS_B; // palm

	handPorts.takktile[0] = (uint32_t*) I2C1_BASE;
	handPorts.takktile[1] = (uint32_t*) SPI1_BASE;
	handPorts.takktile[2] = (uint32_t*) I2C3_BASE;

	handPorts.encoder[0] = (uint32_t*) I2C1_BASE;
	handPorts.encoder[1] = (uint32_t*) SPI1_BASE;
	handPorts.encoder[2] = (uint32_t*) I2C3_BASE;

	handPorts.encoderI2CAddress[0] = AS5048_7BIT_ADDRESS;
	handPorts.encoderI2CAddress[1] = AS5048_7BIT_ADDRESS_SPI;
	handPorts.encoderI2CAddress[2] = AS5048_7BIT_ADDRESS;

}