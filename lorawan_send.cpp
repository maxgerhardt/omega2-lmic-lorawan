#include <cstdlib>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <pthread.h>
#include <sched.h>
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

//OTAA parameters
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static u1_t APPEUI[8];
// This should also be in little endian format, see above.
static u1_t DEVEUI[8];
// This key should be in big endian format
static u1_t APPKEY[16];

//to be used data rate
dr_t datarate = DR_SF9;

void do_send(osjob_t* j);

//max lora frame on SF7 is actually a little bit smaller
uint8_t payload_buf[256];
size_t payload_len = 0;

//Up and down framecounter to be used
int upCounter = 0;
int downCounter = 0;

//frame port
uint8_t fport = 1;

//whether to send confirmed uplink or not
bool sendConfirmed = false;

//ABP is standard connection method
bool useABP = true;

//for testing: do an infinite loop or only send once?
bool infiniteLoop = false;

void loop();
void setup();

//copied from http://www.yonch.com/tech/82-linux-thread-priority
void set_realtime_priority() {
	int ret;

	// We'll operate on the currently running thread.
	pthread_t this_thread = pthread_self();

	// struct sched_param is used to store the scheduling priority
	struct sched_param params;

	// We'll set the priority to the maximum.
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);

	//std::cout << "Trying to set thread realtime prio = " << params.sched_priority << std::endl;

	// Attempt to set thread real-time priority to the SCHED_FIFO policy
	ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
	if (ret != 0) {
		// Print the error
		std::cout << "Unsuccessful in setting thread realtime prio"
				<< std::endl;
		return;
	}

	// Now verify the change in thread priority
	int policy = 0;
	ret = pthread_getschedparam(this_thread, &policy, &params);
	if (ret != 0) {
		std::cout << "Couldn't retrieve real-time scheduling paramers"
				<< std::endl;
		return;
	}

	// Check the correct policy was applied
	if (policy != SCHED_FIFO) {
		std::cout << "Scheduling is NOT SCHED_FIFO!" << std::endl;
	} else {
		//std::cout << "SCHED_FIFO OK" << std::endl;
	}

	// Print thread scheduling priority
	//std::cout << "Thread priority is " << params.sched_priority << std::endl;
	std::cout << "Thread priority set to realtime." << std::endl;
}

int char2int(char input) {
	if (input >= '0' && input <= '9')
		return input - '0';
	if (input >= 'A' && input <= 'F')
		return input - 'A' + 10;
	if (input >= 'a' && input <= 'f')
		return input - 'a' + 10;
	throw std::invalid_argument("Invalid input string");
}

// This function assumes src to be a zero terminated sanitized string with
// an even number of [0-9a-f] characters, and target to be sufficiently large
void hex2bin(const char* src, char* target) {
	while (*src && src[1]) {
		*(target++) = char2int(*src) * 16 + char2int(src[1]);
		src += 2;
	}
}

void hexDump(const char *desc, const void *addr, size_t len) {
	int i;
	unsigned char buff[17];
	unsigned char *pc = (unsigned char*) addr;

	// Output description if given.
	if (desc != NULL)
		printf("%s:\n", desc);

	if (len == 0) {
		printf("  ZERO LENGTH\n");
		return;
	}

	// Process every byte in the data.
	for (i = 0; i < (int) len; i++) {
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0) {
			// Just don't print ASCII for the zeroth line.
			if (i != 0)
				printf("  %s\n", buff);

			// Output the offset.
			printf("  %04x ", i);
		}

		// Now the hex code for the specific character.
		printf(" %02x", pc[i]);

		// And store a printable ASCII character for later.
		if ((pc[i] < 0x20) || (pc[i] > 0x7e))
			buff[i % 16] = '.';
		else
			buff[i % 16] = pc[i];
		buff[(i % 16) + 1] = '\0';
	}

	// Pad out last line if not exactly 16 characters.
	while ((i % 16) != 0) {
		printf("   ");
		i++;
	}

	// And print the final ASCII bit.
	printf("  %s\n", buff);
}

dr_t SFToDR(int sf) {
	switch (sf) {
#ifndef CFG_us915
	case 12:
		return DR_SF12;
	case 11:
		return DR_SF11;
#endif
	case 10:
		return DR_SF10;
	case 9:
		return DR_SF9;
	case 8:
		return DR_SF8;
	case 7:
		return DR_SF7;
	default:
		//default if not otherwise specified
		return DR_SF9;
	}
}

int DRToSF(dr_t dr) {
	switch (dr) {
#ifndef CFG_us915
	case DR_SF12:
		return 12;
	case DR_SF11:
		return 11;
#endif
	case DR_SF10:
		return 10;
	case DR_SF9:
		return 9;
	case DR_SF8:
		return 8;
	case DR_SF7:
		return 7;
	default:
		return -1;
	}
}

int main(int argc, const char** argv) {

	// make a new ArgumentParser
	ArgumentParser parser;

	// add some arguments to search for
	//payload
	parser.addArgument("-P", "--payload", 1, false);
	//connection method
	parser.addArgument("-m", "--method", 1, false);
	//ABP parameters
	parser.addArgument("-d", "--dev-adr", 1, true);
	parser.addArgument("-n", "--nws-key", 1, true);
	parser.addArgument("-a", "--apps-key", 1, true);
	//payload format
	parser.addArgument("-f", "--format", 1, true);
	//counters
	parser.addArgument("-U", "--up-counter", 1, true);
	parser.addArgument("-D", "--down-counter", 1, true);
	//OTAA parameters
	//i'm running out of letters for good option names, so I'll just use different ones..
	parser.addArgument("-E", "--dev-eui", 1, true);
	parser.addArgument("-A", "--app-eui", 1, true);
	parser.addArgument("-K", "--app-key", 1, true);

	//spreading factor
	parser.addArgument("-s", "--spreading-factor", 1, true);
	//target port number
	parser.addArgument("-f", "--fport", 1, true);

	//confirmed uplink
	parser.addArgument("-c", "--confirmed");

	//do not exit after data has been sent, but reschedule ASAP
	parser.addArgument("-i", "--infinite-loop");

	try {
		// parse the command-line arguments - throws if invalid format
		parser.parse(argc, argv);
	} catch (std::exception& exc) {
		printf("Exception parsing arguments: %s\n", exc.what());
		return -1;
	}

	// if we get here, the configuration is valid
	//get payload
	std::string payload = parser.retrieve<std::string>("payload");

	//get ABP keys (if present)
	std::string devaddr = parser.retrieve<std::string>("dev-adr");
	std::string nwskey = parser.retrieve<std::string>("nws-key");
	std::string appskey = parser.retrieve<std::string>("apps-key");

	//get OTAA keys
	std::string deveui = parser.retrieve<std::string>("dev-eui");
	std::string appeui = parser.retrieve<std::string>("app-eui");
	std::string appkey = parser.retrieve<std::string>("app-key");

	//get up / down counter
	upCounter = std::atoi(parser.retrieve<std::string>("up-counter").c_str());
	downCounter = std::atoi(
			parser.retrieve<std::string>("down-counter").c_str());

	//get format
	std::string format = parser.retrieve<std::string>("format");

	//get connection method
	std::string method = parser.retrieve<std::string>("method");

	int sf = std::atoi(
			parser.retrieve<std::string>("spreading-factor").c_str());
	datarate = SFToDR(sf);

	//set fport
	std::string szFport = parser.retrieve<std::string>("fport");
	if (szFport.empty()) {
		fport = 1;
	} else {
		fport = (uint8_t) std::atoi(szFport.c_str());
	}

	//use different method for checking whether "-c" or "--confirmed" was given --
	//the parser library doesn't like me and never returns the correct value
	//for parser.exists() or parser.count()...

	bool isConfirmedUp = false;

	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--confirmed") == 0) {
			isConfirmedUp = true;
		}

		if (strcmp(argv[i], "-i") == 0
				|| strcmp(argv[i], "--infinite-loop") == 0) {
			infiniteLoop = true;
		}
	}

	sendConfirmed = isConfirmedUp;
	printf("is confirmed up: %d\n", (int) isConfirmedUp);

	//validate counters (if present)
	if (upCounter < 0 || downCounter < 0) {
		printf("Given up/down counters must be non-negative!\n");
		return -1;
	}

	//apply standard settings
	if (format.empty()) {
		format = "hex";
	}

	if (format.empty()) {
		method = "ABP";
	}

	if (format == "hex") {
		//try to decrypt the payload in hex
		if (payload.length() % 2 != 0) {
			printf("Hex payload must have even length!\n");
			return -1;
		}

		if (payload.length() / 2 > sizeof(payload_buf)) {
			printf("Payload too big!\n");
			return -1;
		}

		hex2bin(payload.c_str(), (char*) payload_buf);
		payload_len = payload.length() / 2;
	} else if (format == "ascii") {
		if (payload.length() > sizeof(payload_buf)) {
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

	if (method == "ABP") {

		printf("Using connection method ABP.\n");
		useABP = true;

		//validate formats
		if (devaddr.length() != sizeof(DEVADDR) * 2) {
			printf("Device Address string has invalid length.\n");
			return -1;
		}

		if (nwskey.length() != sizeof(NWKSKEY) * 2) {
			printf("Network session key string has invalid length.\n");
			return -1;
		}

		if (appskey.length() != sizeof(APPSKEY) * 2) {
			printf("App session key string has invalid length.\n");
			return -1;
		}

		printf("DEV ADDR: \"%s\"\n", devaddr.c_str());
		printf("NWS KEY: \"%s\"\n", nwskey.c_str());
		printf("APS KEY: \"%s\"\n", appskey.c_str());

		printf("all keys loaded\n");
		hex2bin(devaddr.c_str(), (char*) &DEVADDR);
		//correct endianness
		DEVADDR = __builtin_bswap32(DEVADDR);
		//printf("DEV ADDR: %08x \"%s\"\n", DEVADDR, devaddr.c_str());
		hex2bin(nwskey.c_str(), (char*) NWKSKEY);
		//hexDump("nwskey", NWKSKEY, sizeof(NWKSKEY));
		hex2bin(appskey.c_str(), (char*) APPSKEY);
		//hexDump("appskey", APPSKEY, sizeof(APPSKEY));
	} else if (method == "OTAA") {
		useABP = false;

		//	std::string deveui = parser.retrieve<std::string>("dev-eui");
		//std::string appeui = parser.retrieve<std::string>("app-eui");
		//std::string appkey = parser.retrieve<std::string>("app-key");

		//validate formats
		if (deveui.length() != sizeof(DEVEUI) * 2) {
			printf("Device EUI string has invalid length.\n");
			return -1;
		}

		if (appeui.length() != sizeof(APPEUI) * 2) {
			printf("APP EUI key string has invalid length.\n");
			return -1;
		}

		if (appkey.length() != sizeof(APPKEY) * 2) {
			printf("App key string has invalid length.\n");
			return -1;
		}

		printf("DEV EUI: \"%s\"\n", deveui.c_str());
		printf("APP EUI: \"%s\"\n", appeui.c_str());
		printf("APP KEY: \"%s\"\n", appkey.c_str());

		hex2bin(deveui.c_str(), (char*) DEVEUI);
		//printf("DEV ADDR: %08x \"%s\"\n", DEVADDR, devaddr.c_str());
		hex2bin(appeui.c_str(), (char*) APPEUI);
		//hexDump("nwskey", NWKSKEY, sizeof(NWKSKEY));
		hex2bin(appkey.c_str(), (char*) APPKEY);
		//hexDump("appskey", APPSKEY, sizeof(APPSKEY));
	} else {
		printf("Unknown connection method given: \"%s\". Valid: ABP,OTAA.\n",
				method.c_str());
		return -1;
	}

	//NEEDED for timing accuracy or receiving / OTAA joining might not work
	set_realtime_priority();
	setup();

	while (1) {
		loop();
	}

	return 0;
}

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t* buf) {
	if (!useABP) {
		memcpy(buf, APPEUI, 8);
	}
}
void os_getDevEui(u1_t* buf) {
	if (!useABP) {
		memcpy(buf, DEVEUI, 8);
	}
}
void os_getDevKey(u1_t* buf) {
	if (!useABP) {
		memcpy(buf, APPKEY, 16);
	}
}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = { .nss = 6, .rxtx = LMIC_UNUSED_PIN, .rst =
		LMIC_UNUSED_PIN, .dio = { 0, 1, LMIC_UNUSED_PIN } };

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
		// Disable link check validation (automatically enabled
		// during join, but not supported by TTN at this time).
		LMIC_setLinkCheckMode(0);
		//change RX2 SF now to be able to receive normal payload
		LMIC.dn2Dr = DR_SF9;
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
		printf("[+] TX + RX done.\n");

		if (!infiniteLoop) {
			exit(0);
		} else {
			// Schedule next transmission
			os_setTimedCallback(&sendjob,
					os_getTime() + sec2osticks(TX_INTERVAL), do_send);
		}

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
		LMIC_setTxData2(1, payload_buf, payload_len, sendConfirmed);
		printf("Packet queued (len=%d, confirmed=%d).\n", (int) payload_len,
				(int) sendConfirmed);
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	if (useABP) {
		// Set static session parameters. Instead of dynamically establishing a session
		// by joining the network, precomputed session parameters are be provided.
		LMIC_setSession(0x1, DEVADDR, (u1_t*) NWKSKEY, (u1_t*) APPSKEY);
	}

	//setup counter
	if (useABP) {
		printf("Using counters up %d down %d\n", upCounter, downCounter);
		LMIC.seqnoUp = upCounter;
		LMIC.seqnoDn = downCounter;
	}


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
	//if we use OTAA, we get this info automatically.
	if (useABP)
		LMIC.dn2Dr = DR_SF9;

	LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(datarate, 14);
	printf("Using SF%d\n", DRToSF(datarate));

	//Sending data will trigger an OTAA join if it is configured to do so.
	do_send(&sendjob);
}

void loop() {
	os_runloop_once();
}

