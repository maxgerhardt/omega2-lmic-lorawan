#include <cstdlib>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <lmic.h>
#include <hal/hal.h>
#include "argparse.hpp"

//ABP parameters

// LoRaWAN NwkSKey, network session key
static u1_t NWKSKEY[16];
// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16];

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR;

void do_send(osjob_t* j);

//max lora frame on SF7 is actually a little bit smaller
uint8_t payload_buf[256];
size_t payload_len = 0;

void loop();
void setup();

int char2int(char input)
{
  if(input >= '0' && input <= '9')
    return input - '0';
  if(input >= 'A' && input <= 'F')
    return input - 'A' + 10;
  if(input >= 'a' && input <= 'f')
    return input - 'a' + 10;
  throw std::invalid_argument("Invalid input string");
}

// This function assumes src to be a zero terminated sanitized string with
// an even number of [0-9a-f] characters, and target to be sufficiently large
void hex2bin(const char* src, char* target)
{
  while(*src && src[1])
  {
    *(target++) = char2int(*src)*16 + char2int(src[1]);
    src += 2;
  }
}


void hexDump (const char *desc, const void *addr, size_t len) {
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL)
        printf ("%s:\n", desc);

    if (len == 0) {
        printf("  ZERO LENGTH\n");
        return;
    }

    // Process every byte in the data.
    for (i = 0; i < (int)len; i++) {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
                printf ("  %s\n", buff);

            // Output the offset.
            printf ("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf (" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e))
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printf ("   ");
        i++;
    }

    // And print the final ASCII bit.
    printf ("  %s\n", buff);
}


int main(int argc, const char** argv) {

	// make a new ArgumentParser
	ArgumentParser parser;

	// add some arguments to search for
	parser.addArgument("-p", "--payload", 1, false);
	parser.addArgument("-d", "--dev-adr", 1, false);
	parser.addArgument("-n", "--nws-key", 1, false);
	parser.addArgument("-a", "--apps-key", 1, false);
	parser.addArgument("-f", "--format", 1, true);
	//parser.addArgument("-c", "--confirmed", 1, true);

	//const String& name, char nargs = 0, bool optional = true

	try {
		// parse the command-line arguments - throws if invalid format
		parser.parse(argc, argv);
	} catch(std::exception& exc) {
		printf("Exception parsing arguments: %s\n", exc.what());
		return -1;
	}

	// if we get here, the configuration is valid
	std::string payload = parser.retrieve<std::string>("payload");
	std::string devaddr = parser.retrieve<std::string>("dev-adr");
	std::string nwskey = parser.retrieve<std::string>("nws-key");
	std::string appskey = parser.retrieve<std::string>("apps-key");

	std::string format = parser.retrieve<std::string>("format");

	if(format.empty()) {
		format = "hex";
	}


	if(format == "hex") {
		//try to decrypt the payload in hex
		if(payload.length() % 2 != 0) {
			printf("Hex payload must have even length!\n");
			return -1;
		}

		if(payload.length() / 2 > sizeof(payload_buf)) {
			printf("Payload too big!\n");
			return -1;
		}

		hex2bin(payload.c_str(), (char*) payload_buf);
		payload_len = payload.length() / 2;
	} else if(format == "ascii") {
		if(payload.length() > sizeof(payload_buf)) {
			printf("Payload too big!\n");
			return -1;
		}
		memcpy(payload_buf, payload.c_str(), payload.length());
		payload_len = payload.length();
	} else {
		printf("Unrecognized \"format\" option! Must be hex or ascii\n");
	}

	printf("Payload: \"%s\"\n", payload.c_str());
	hexDump("payload decoded", payload_buf, payload_len);

	if(devaddr.length() != sizeof(DEVADDR)*2) {
		printf("Device Address string has invalid length.\n");
		return -1;
	}

	if(nwskey.length() != sizeof(NWKSKEY)*2) {
		printf("Network session key string has invalid length.\n");
		return -1;
	}


	if(appskey.length() != sizeof(APPSKEY)*2) {
		printf("App session key string has invalid length.\n");
		return -1;
	}

	printf("DEV ADDR: \"%s\"\n", devaddr.c_str());
	printf("NWS KEY: \"%s\"\n", nwskey.c_str());
	printf("APS KEY: \"%s\"\n", appskey.c_str());

	printf("all keys loaded\n");
	hex2bin(devaddr.c_str(), (char*)&DEVADDR);
	//correct endianness
	DEVADDR = __builtin_bswap32 (DEVADDR);
	//printf("DEV ADDR: %08x \"%s\"\n", DEVADDR, devaddr.c_str());
	hex2bin(nwskey.c_str(), (char*)NWKSKEY);
	//hexDump("nwskey", NWKSKEY, sizeof(NWKSKEY));
	hex2bin(appskey.c_str(), (char*)APPSKEY);
	//hexDump("appskey", APPSKEY, sizeof(APPSKEY));

	setup();

	while (1) {
		loop();
	}

	return 0;
}

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t* buf) { (void) buf;
}
void os_getDevEui(u1_t* buf) { (void) buf;
}
void os_getDevKey(u1_t* buf) { (void) buf;
}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
		.nss = LMIC_UNUSED_PIN,
		.rxtx = LMIC_UNUSED_PIN,
		.rst = LMIC_UNUSED_PIN,
		.dio = { 0, 1, LMIC_UNUSED_PIN }
};

void onEvent(ev_t ev) {
	printf("%d", os_getTime());
	printf(": ");
	switch (ev) {
	case EV_SCAN_TIMEOUT:
		printf("EV_SCAN_TIMEOUT\n");
		break;
	case EV_BEACON_FOUND:
		printf("EV_BEACON_FOUND\n");
		break;
	case EV_BEACON_MISSED:
		printf("EV_BEACON_MISSED\n");
		break;
	case EV_BEACON_TRACKED:
		printf("EV_BEACON_TRACKED");
		break;
	case EV_JOINING:
		printf("EV_JOINING\n");
		break;
	case EV_JOINED:
		printf("EV_JOINED\n");
		break;
	case EV_RFU1:
		printf("EV_RFU1\n");
		break;
	case EV_JOIN_FAILED:
		printf("EV_JOIN_FAILED\n");
		break;
	case EV_REJOIN_FAILED:
		printf("EV_REJOIN_FAILED\n");
		break;
	case EV_TXCOMPLETE:
		printf("EV_TXCOMPLETE (includes waiting for RX windows)\n");
		if (LMIC.txrxFlags & TXRX_ACK)
			printf("Received ack\n");
		if (LMIC.dataLen) {
			printf("Received ");
			printf("%d", (int) LMIC.dataLen);
			printf(" bytes of payload\n");

			uint8_t fport = LMIC.frame[LMIC.dataBeg - 1];
			uint8_t* data = (uint8_t*) &LMIC.frame[LMIC.dataBeg];
			size_t len = (size_t) LMIC.dataLen;
			printf("From FPort %d\n", (int) fport);
			hexDump("lora rx data", data, len);
		}

		//the program is done
		printf("[+] TX + RX done, exiting\n");
		//exit(0);

		// Schedule next transmission
		os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
		break;
	case EV_LOST_TSYNC:
		printf("EV_LOST_TSYNC\n");
		break;
	case EV_RESET:
		printf("EV_RESET\n");
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		printf("EV_RXCOMPLETE\n");
		break;
	case EV_LINK_DEAD:
		printf("EV_LINK_DEAD\n");
		break;
	case EV_LINK_ALIVE:
		printf("EV_LINK_ALIVE\n");
		break;
	default:
		printf("Unknown event\n");
		break;
	}
}

void do_send(osjob_t* j) {
	(void) j;
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
		printf("OP_TXRXPEND, not sending\n");
	} else {
		// Prepare upstream data transmission at the next possible time.
		LMIC_setTxData2(1, payload_buf, payload_len, 0);
		printf("Packet queued.\n");
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// Set static session parameters. Instead of dynamically establishing a session
	// by joining the network, precomputed session parameters are be provided.
	LMIC_setSession(0x1, DEVADDR, (u1_t*) NWKSKEY, (u1_t*) APPSKEY);

#if defined(CFG_eu868)
	// Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
	// channels from the LoRaWAN specification are used, which certainly
	// works, so it is good for debugging, but can overload those
	// frequencies, so be sure to configure the full frequency range of
	// your network here (unless your network autoconfigures them).
	// Setting up channels should happen after LMIC_setSession, as that
	// configures the minimal channel set.
	// NA-US channels 0-71 are configured automatically
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
	LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI); // g2-band
	// TTN defines an additional channel at 869.525Mhz using SF9 for class B
	// devices' ping slots. LMIC does not have an easy way to define set this
	// frequency and support for class B is spotty and untested, so this
	// frequency is not configured here.
#elif defined(CFG_us915)
	// NA-US channels 0-71 are configured automatically
	// but only one group of 8 should (a subband) should be active
	// TTN recommends the second sub band, 1 in a zero based count.
	// https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
	LMIC_selectSubBand(1);
#endif

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF9, 14);

	// Start job
	do_send(&sendjob);
}

void loop() {
	os_runloop_once();
}

