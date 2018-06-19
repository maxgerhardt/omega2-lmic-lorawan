#include "SC18IS602B.h"
#include <cstdlib>
#include <cstdint>
#include <onion-i2c.h>
#include <onion-debug.h>
#include <ugpio.h>

/* use /dev/i2c-0 */
#define ONION_I2C_DEV_NUM 0

/* Define bitSet, bitClear and bitWrite macros for 32-bit values */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

SC18IS602B::SC18IS602B(int interruptPin, int resetPin, bool a0, bool a1, bool a2)
    : resetPin(resetPin), interruptPin(interruptPin) {
    //calculate the module's address here.
    //last 3 bit are the value of the address pin
    address = 0B0101000 | (a2 << 2) | (a1 << 1) | (a0);
}

SC18IS602B::~SC18IS602B() {
}

void SC18IS602B::reset() {
    if(resetPin != -1) {
    	int rq = 0, rv = 0;
    	//request pin. Pulse reset. release it.
    	if ((rq = gpio_is_requested(resetPin)) < 0)
    	{
    		perror("gpio_is_requested");
    	}
    	// export the gpio
    	if (!rq) {
    		printf("> exporting gpio\n");
    		if ((rv = gpio_request(resetPin, NULL)) < 0)
    		{
    			perror("gpio_request");
    		}
    	}

    	// set to input direction
    	printf("> setting to input\n");
    	if ((rv = gpio_direction_input(resetPin)) < 0)
    	{
    		perror("gpio_direction_input");
    	}

        //RESET is low active, LOW
        //Generate a high-to-low-to-high transition
        //must be at least 50ns long (t_sa). 1ms is enough.
    	gpio_set_value(resetPin, 1);
    	usleep(1000 * 1);
    	gpio_set_value(resetPin, 0);
    	usleep(1000 * 1);
    	gpio_set_value(resetPin, 1);
    	usleep(1000 * 1);
    }
}

void SC18IS602B::free() {
	if(resetPin != -1) {
		gpio_free(resetPin);
	}
}

bool SC18IS602B::enableGPIO(int num, bool enable) {
    //sanity check
    if(num < 0 || num > 3)
        return false;
    //enable this GPIO while leaving the others untouched.
    bitWrite(gpioEnable, num, enable);
    //Send the new enable configuration
    return this->i2c_write(SC18IS601B_GPIO_ENABLE_CMD, &gpioEnable, sizeof(gpioEnable));
}

bool SC18IS602B::setupGPIO(int num, SC18IS601B_GPIOPinMode mode) {
    //sanity check
    if(num < 0 || num > 3)
        return false;

    //Cast the enum back to the bits
    //mode is a 2-bit wide bitfield
    uint8_t modeAsBitfield = (uint8_t) mode;

    //write 2 the bits into our last config value
    //refer to table 10 in the datasheet
    bitWrite(gpioConfig, 2*num, modeAsBitfield & 1);
    bitWrite(gpioConfig, 2*num + 1, modeAsBitfield >> 1);

    return this->i2c_write(SC18IS601B_GPIO_CONFIGURATION_CMD, &gpioConfig, sizeof(gpioConfig));
}

bool SC18IS602B::writeGPIO(int num, bool val) {
    if(num < 0 || num > 3)
        return false;
    //Re-write old value
    bitWrite(gpioWrite, num, val);
    return this->i2c_write(SC18IS601B_GPIO_WRITE_CMD, &gpioWrite, sizeof(gpioWrite));
}

bool SC18IS602B::writeGPIOBank(uint8_t value) {
    //remember new value
    gpioWrite = value;
    return this->i2c_write(SC18IS601B_GPIO_WRITE_CMD, &gpioWrite, sizeof(gpioWrite));
}


bool SC18IS602B::writeGPIOBank(bool gpio0, bool gpio1, bool gpio2, bool gpio3) {
    //Writes all gpio values to the pin
    uint8_t gpioVal = (gpio3 << 3) | (gpio2 << 2) | (gpio1 << 1) | gpio0;
    return writeGPIOBank(gpioVal);
}

bool SC18IS602B::readGPIO(int num) {
    if(num < 0 || num > 3)
        return false;

    //refer chapter 7.1.9
    //issue a read command.
    //this will cause the storage of 1 byte in the data buffer
    if(!this->i2c_write(SC18IS601B_GPIO_READ_CMD, nullptr, 0))
        return false;

    //Now try to read the buffer
    uint8_t gpioReadBuf = 0;
    size_t readBytes = this->i2c_read(&gpioReadBuf, sizeof(gpioReadBuf));

    if(readBytes == 0) {
        return false;
    }

    //return the bit at the needed position
    return bitRead((int)gpioReadBuf, num);
}

bool SC18IS602B::setLowPowerMode() {
    return this->i2c_write(SC18IS601B_IDLE_CMD, nullptr, 0);
}

bool SC18IS602B::clearInterrupt() {
    return this->i2c_write(SC18IS601B_CLEAR_INTERRUPT_CMD, nullptr, 0);
}

void SC18IS602B::configureIntPin() {
	//put the interrupt pin in INPUT mode.
	if(interruptPin != -1 && !intWasConfigured) {
		int rq = 0, rv = 0;
		// check if gpio is already exported
		if ((rq = gpio_is_requested(interruptPin)) < 0)
		{
			perror("gpio_is_requested");
			return;
		}
		// export the gpio
		if (!rq) {
			printf("> exporting gpio\n");
			if ((rv = gpio_request(interruptPin, NULL)) < 0)
			{
				perror("gpio_request");
				return;
			}
		}

		// set to input direction
		printf("> setting to input\n");
		if ((rv = gpio_direction_input(interruptPin)) < 0)
		{
			perror("gpio_direction_input");
			return;
		}
		intWasConfigured = true;
	}
}

bool SC18IS602B::configureSPI(SC18IS601B_SPI_BitOrder lsbFirst, SC18IS601B_SPI_Mode spiMode,
        SC18IS601B_SPI_Speed clockSpeed) {
    //sanity check on parameters
    if(spiMode > SC18IS601B_SPIMODE_3)
        return false;
    uint8_t clk = (uint8_t)((uint8_t)(clockSpeed) & 0B11);

    //see chapter 7.1.5
    uint8_t configByte = ((int)lsbFirst << 5) | (spiMode << 2) | clk;
    return this->i2c_write(SC18IS601B_CONFIG_SPI_CMD, &configByte, sizeof(configByte));
}

uint8_t SC18IS602B::spiTransfer(int slaveNum, uint8_t txByte) {
    uint8_t readBuf = 0;
    this->spiTransfer(slaveNum, &txByte, 1, &readBuf);
    return readBuf;
}

bool SC18IS602B::i2c_write(uint8_t cmdByte, const uint8_t* data, size_t len) {
	bool r =
			i2c_writeBuffer(ONION_I2C_DEV_NUM, address, (int)cmdByte, (uint8_t*)data, (int)len) == EXIT_SUCCESS;
	return r;
}

size_t SC18IS602B::i2c_read(uint8_t* readBuf, size_t len) {
	return
		i2c_readRaw(ONION_I2C_DEV_NUM, address, readBuf, (int)len)
			== EXIT_SUCCESS;
}

bool SC18IS602B::spiTransfer(int slaveNum, const uint8_t* txData, size_t txLen,
        uint8_t* readBuf) {
    //sanity check
    if(slaveNum < 0 || slaveNum > 3)
        return false;

    //Overly long data?
    if(txLen > SC18IS601B_DATABUFFER_DEPTH)
        return false;

    if(interruptPin != -1) {
    	configureIntPin();
    	//Clear interrupt before every SPI transfer.
    	//INT pin should now be HIGH.
		int state = gpio_get_value(interruptPin);
		printf("STATE BEFORE : %d\n", state);

    	clearInterrupt();

		state = gpio_get_value(interruptPin);
		printf("STATE AFTER: %d\n", state);

    }


    //the function ID will have the lower 4 bits set to the
    //activated slave selects. We use only 1 at a time here.
    uint8_t functionID = (1 << slaveNum);
    //transmit our TX buffer
    if(!this->i2c_write(functionID, txData, txLen))
        return false;

    if(interruptPin != -1) {
    	//spin-lock until an interrupt occurs (NOT INT got LOW => INT is fired)
    	while(1) {
    		int state = gpio_get_value(interruptPin);
    		printf("STATE: %d\n", state);
    		if(state == 0) {
    			break;
    		}
    		usleep(500 *1000);
    	}
    }

    usleep(500 * txLen);

    //read in the data that came from MISO
    return i2c_read(readBuf, txLen);
}
