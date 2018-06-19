# Omega2 LMIC

### Description

A port of [Arduino-LMIC](https://github.com/matthijskooijman/arduino-lmic) to the Omega2. Will support native SPI and SPI bridges.

This is *WORK IN PROGRESS*!

### Wireup

Schematic to follow.

Basically:
* connect SC18IS602B via I2C to Omega
   * all address pins to GND
   * attach bridge's SPI bus to the radio's SPI bus (MISO, MOSI, SCLK, CS)
   * use GPIO1 of the bridge for chip select
* attach DIO0 and DIO1 to GPIO 0 and 1 of the Omega2 (rest of DIOs not needed)
* power everything (3.3V, GND), add decloupling capacitor for radio

Refer to the datasheets:
* https://www.nxp.com/docs/en/data-sheet/SC18IS602B.pdf
* www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf
* or your original Semtech chips

### Features

* a port of LMIC for the Omega2
  * supports SX1272 and SX1276 type radios  
  * communication to radio via native SPI interface or an I²C to SPI bridge (NXP SC18IS602B)
* includes the `lorawan_send` commandline tool 
  * send abitratry payload given the necessary ABP keys
  
```sh
Usage: ./lorawan_send --payload PAYLOAD --dev-adr DEV-ADR --nws-key NWS-KEY --apps-key APPS-KEY
                     [--format FORMAT]
```

### In Action

```
root@Omega-C465:~# ./lorawan_send  --payload "Hello LoRa!" --dev-adr "26011A84" --nws-key "removed" --apps-key "removed" -f "ascii"
Payload: "Hello LoRa!"
payload decoded:
  0000  48 65 6c 6c 6f 20 4c 6f 52 61 21                 Hello LoRa!
DEV ADDR: "26011A84"
NWS KEY: "removed"
APS KEY: "removed"
Packet queued.
83288589: EV_TXCOMPLETE (includes waiting for RX windows)
[+] TX + RX done, exiting
```

Inside TTN console:

![TTN](https://github.com/maxgerhardt/omega2-lmic-lorawan/raw/master/img/TTN.png)

Wireup:

![wireup](https://github.com/maxgerhardt/omega2-lmic-lorawan/raw/master/img/IMG_20180619_135413.jpg)

### Quickstart

You want to try this out? You need: 
* a SX1276-like radio (e.g. HopeRF RFM95W)
* NXP SC18IS602B I²C to SPI bridge
* wires
* a TTN account

In the TTN console, create a new application and a new device. Set the device to use ABP, 16-bit framecounter and disable frame counter checks. 

The displayed keys in the console (device address, network session key, application session key) can then be used in the console applications.

You need the dependencies installed on your Omega2:

```sh
opkg update
opkg install libonioni2c 
opkg install libonionspi
```

Wireup the module.

Transfer the binary executable `lorawan_send` to your Omeag2 and execute it with the needed parameters.

### TODO

* adjustable spreading factor (SF) and coding rate (CR)
* make radio type selectable (currently SX1276 type)
* selectable frequency plan (currently hardcoded on EU868), make US frequencies selectable 
* make LoRa reception work (timing issues right now?)
* make example with sensors (temp sensor, display, whatever) 

### License

The original code from LMIC see the License section at https://github.com/matthijskooijman/arduino-lmic.
The command line tool code is also derived from the ABP example code.

### Contributors

* Maximilian Gerhardt (port for LMIC HAL, SC18IS602B bridge library)
* Matthijs Kooijman (LMIC author)