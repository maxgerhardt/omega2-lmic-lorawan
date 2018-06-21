/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/


#include "../lmic.h"
#include "hal.h"
#include <stdio.h>
#include "onion-spi.h"
#include "ugpio.h"
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
// -----------------------------------------------------------------------------
// I/O

#define OUTPUT 1
#define INPUT 0

//#define USE_NATIVE_SPI

#ifndef USE_NATIVE_SPI
#include "SC18IS602B.h"
SC18IS602B spiBridge(-1, -1, 0, 0, 0);
#endif

void pinMode(int gpio, int mode) {

	//CS is handled in software because LMIC is kinda stupid
	//if(gpio == lmic_pins.nss)
	//	return;

	if(gpio < 0) {
		printf("Will not register GPIO %d\n", gpio);
	}

	//printf("Registering GPIO %d\n", gpio);

	if(mode == OUTPUT || mode ==INPUT ) {

		unsigned int gpio_u = (unsigned int) gpio;
		//use ugpio.
		//is the gpio already requested by somebody else?
		int err;
		if ((err = gpio_is_requested(gpio_u)) < 0) {
			printf("[-] GPIO %d already requested.\n", gpio);
			return;
		}
		//export GPIO if necessary
		if (!err) {
			printf("Exporting GPIO %d\n", gpio);
			if ((err = gpio_request(gpio_u, NULL)) < 0) {
				printf("[-] Requesting GPIO %d failed.\n", gpio);
				return;
			}
		}
		if(mode == INPUT) {
			if((err = gpio_direction_input(gpio_u)) < 0) {
				printf("[-] Setting GPIO %d input mode failed\n", gpio);
			}
		} else {
			if((err = gpio_direction_output(gpio_u, 1)) < 0) {
				printf("[-] Setting GPIO %d to output mode failed.\n", gpio);
				return;
			}
		}
	} else {
		//unknown
		printf("UNKNOWN PIN MODE\n");
		hal_failed(__FILE__, __LINE__);
	}
}

void digitalWrite(int pin, int value) {
	gpio_set_value((unsigned) pin, value);
}

int digitalRead(int pin) {
	return gpio_get_value((unsigned)pin);
}

uint32_t micros() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return 1000000 * tv.tv_sec + tv.tv_usec;
}

void delay(int ms) {
	usleep(ms * 1000);
}

void delayMicroseconds(int micros) {
	usleep(micros);
}

static void hal_io_init () {
    // NSS and DIO0 are required, DIO1 is required for LoRa, DIO2 for FSK
//only when using native SPI we need CS, otherwise handled by bridge
#ifdef USE_NATIVE_SPI
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
#endif
    ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN || lmic_pins.dio[2] != LMIC_UNUSED_PIN);

#ifdef USE_NATIVE_SPI
    pinMode(lmic_pins.nss, OUTPUT);
#endif
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rxtx, OUTPUT);
    if (lmic_pins.rst != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rst, OUTPUT);

    pinMode(lmic_pins.dio[0], INPUT);
    if (lmic_pins.dio[1] != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.dio[1], INPUT);
    if (lmic_pins.dio[2] != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.dio[2], INPUT);
}

// val == 1  => tx 1
void hal_pin_rxtx (u1_t val) {
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.rxtx, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return;

    if(val == 0 || val == 1) { // drive pin
        pinMode(lmic_pins.rst, OUTPUT);
        digitalWrite(lmic_pins.rst, val);
    } else { // keep pin floating
        pinMode(lmic_pins.rst, INPUT);
    }
}

static bool dio_states[NUM_DIO] = {0};

static void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
            continue;

        if (dio_states[i] != digitalRead(lmic_pins.dio[i])) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i])
                radio_irq_handler(i);
        }
    }
}

// -----------------------------------------------------------------------------
// SPI

/* global SPI config object */
struct spiParams params;

static void hal_spi_init () {
#ifdef USE_NATIVE_SPI
	//use default values at first
	int err = 0;
	spiParamInit(&params);
	params.misoGpio = 9;
	params.mosiGpio = 8;
	params.sckGpio = 7;
	params.csGpio = -1;
	params.speedInHz = 1* 1000 * 1000 / 2 / 2;
	params.modeBits = SPI_DEFAULT_MODE_BITS; //only SPIMODE_0, no dual TX or RX as "default"..
	params.busNum = 1; //this comes from my local machine. no idea if it works elsewhere.. (ls /dev/spi*)
	params.deviceId = 32766;
	params.delayInUs = 10;

	//is our device already mapped?
	err = spiCheckDevice(params.busNum, params.deviceId, ONION_SEVERITY_DEBUG);
	if(err == EXIT_FAILURE) {
		printf("[-] spiCheckDevice() failed.\n");
		return;
	}

	//register ourselves
	err = spiRegisterDevice(&params);
	if(err == EXIT_FAILURE)  {
		printf("spiRegisterDevice() failed.\n");
		return;
	}
	printf("[+] SPI register device okay.\n");

	err = spiSetupDevice(&params);
	if(err == EXIT_FAILURE)  {
		printf("spiSetupDevice() failed.\n");
		return;
	}
#else
#ifndef USE_SPI_TRANSFER_CALLS
	//set GPIO0 to SlaveSelect mode (NOT GPIO mode)
	//use a trick here: put GPIO0 in non-GPIO mode (slave select)
	//but actually use GPIO1 as software-controlled SPI
	//LMIC wants to do own transfers so we can't have the usually controlled CS.
	spiBridge.enableGPIO(0, false);
	spiBridge.enableGPIO(1, true);
#else
	//if using SPI transfer calls, connect slave select to GPIO0 normally
	spiBridge.enableGPIO(0, false);
#endif
	spiBridge.configureSPI(SC18IS601B_SPI_MSBFIRST, SC18IS601B_SPIMODE_0, SC18IS601B_SPICLK_1843_kHz);
#endif
}

void hal_pin_nss (u1_t val) {
    //Serial.println(val?">>":"<<");
#ifdef USE_NATIVE_SPI
    digitalWrite(lmic_pins.nss, val);
#else
	spiBridge.writeGPIO(1, val);
#endif
}

void hal_spi_transfer(uint8_t* txBuf, size_t txLen, uint8_t* rxBuf) {
#ifdef USE_NATIVE_SPI
	int err = spiTransfer(&params, txBuf, rxBuf, txLen);
	if(err == EXIT_FAILURE)
		printf("[-] SPI transfer failed!\n");
#else
	spiBridge.spiTransfer(0, txBuf, txLen, rxBuf);
#endif
}

// perform SPI transaction with radio1
u1_t hal_spi (u1_t out) {
	uint8_t recv = 0;
#ifdef USE_NATIVE_SPI
	onionSetVerbosity(ONION_VERBOSITY_VERBOSE);
	uint8_t outBuf[2] = { recv, 0};
	uint8_t inBuf[2] = {0,0};
	int err = spiTransfer(&params, outBuf, inBuf, 2);
	printf("[!] > %02x < %02x %02x\n", (int)out, (int)inBuf[0], (int)inBuf[1]);
	//int err = spiTransfer(&params, &out, &recv, 1);
	if(err == EXIT_FAILURE)
		printf("[-] SPI transfer failed!\n");
#else
	spiBridge.spiTransfer(0, &out, 1, &recv);
#endif
	//printf("> %02x < %02x\n", (int) out, (int)recv);

	return recv;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    // Nothing to do
}

u4_t hal_ticks () {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    static uint8_t overflow = 0;

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    overflow += (msb ^ overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)overflow << 24);

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383.
    while (delta > (16000 / US_PER_OSTICK)) {
        delay(16);
        delta -= (16000 / US_PER_OSTICK);
    }
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    //noInterrupts();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        //interrupts();

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

void hal_sleep () {
    // Not implemented
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}
#endif // defined(LMIC_PRINTF_TO)

void hal_init () {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
}

void hal_failed (const char *file, u2_t line) {
#if defined(LMIC_FAILURE_TO)
/*    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
    */

	printf("FAILURE: %s: %d\n", file, (int)line);

#endif
    hal_disableIRQs();
    while(1);
}
