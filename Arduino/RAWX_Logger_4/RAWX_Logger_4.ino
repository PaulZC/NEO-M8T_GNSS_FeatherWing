// RAWX_Logger V4

// Logs RXM-RAWX, RXM-SFRBX and TIM-TM2 data from u-blox NEO-M8T GNSS to SD card

// Changes to a new log file every INTERVAL minutes

// Version 4 uses a RingBufferN to store the incomming serial data so you don't need to worry about increasing
// the serial receive buffer size

// This version also includes NeoPixel support

// This code is written for the Adalogger M0 Feather
// https://www.adafruit.com/products/2796
// https://learn.adafruit.com/adafruit-feather-m0-adalogger

// GNSS data is provided by Paul's u-blox NEO-M8T FeatherWing
// https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing

// Solder the GRESET link so that pressing the reset button
// resets the NEO-M8T as well as the Adalogger.

// Choose a good quality SD card. Some cheap cards can't handle the write rate.
// Ensure the card is formatted as FAT32.

// The RAWX log file has a .ubx suffix instead of .bin
// The log filename starts with "r_" for the rover and "b_" for the static base

// Define how long we should log in minutes before changing to a new file
// Sensible values are: 5, 10, 15, 20, 30, 60
// Must be <= 60 (or RTC alarm code needs to be updated to match on HHMMSS)
const int INTERVAL = 15;

// Define how long we should wait in msec (approx.) for residual RAWX data before closing the last log file
// For a measurement rate of 4Hz (250msec), 300msec is a sensible value. i.e. slightly more than one measurement interval
const int dwell = 300;

// Define if this is the static base logger
//#define STATIC // Comment this line out for the mobile rover logger

// LEDs

//#define NoLED // Uncomment this line to completely disable the LEDs
//#define NoLogLED // Uncomment this line to disable the LEDs during logging only

// NeoPixel Settings
//#define NeoPixel // Uncomment this line to enable a NeoPixel on the same pin as RedLED

// The red LED flashes during SD card writes
#define RedLED 13 // The red LED on the Adalogger is connected to Digital Pin 13
// The green LED indicates that the GNSS has established a fix 
#define GreenLED 8 // The green LED on the Adalogger is connected to Digital Pin 8

// Support for the WB2812B NeoPixel is provided by Adafruit:
// https://github.com/adafruit/Adafruit_NeoPixel

#ifdef NeoPixel
#include <Adafruit_NeoPixel.h> // Support for the WB2812B
#define swap_red_green // Uncomment this line if your WB2812B has red and green reversed
#ifdef swap_red_green
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RedLED, NEO_GRB + NEO_KHZ800); // GRB WB2812B
#else
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RedLED, NEO_RGB + NEO_KHZ800); // RGB WB2812B
#endif
#define LED_Brightness 32 // 0 - 255 for WB2812B
#endif

// Define SerialBuffer as a large RingBuffer which we will use to store the Serial1 receive data
// Actual Serial1 receive data will be copied into SerialBuffer by a timer interrupt
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
// That way, we do not need to increase the size of the Serial1 receive buffer (by editing RingBuffer.h)
RingBufferN<8192> SerialBuffer; // Define SerialBuffer as a RingBuffer of size 8192 bytes

// Define the GNSS configuration: GPS + Galileo + either GLONASS or BeiDou
#define GLONASS // Comment this line out to use BeiDou

// Send serial debug messages
//#define DEBUG // Comment this line out to disable debug messages

// Connect a normally-open push-to-close switch between swPin and GND.
// Press it to stop logging and close the log file.
#define swPin 15 // Digital Pin 15 (0.2" away from the GND pin on the Adalogger)

// Include the Adafruit GPS Library
// https://github.com/adafruit/Adafruit_GPS
// This is used at the start of the code to establish a fix and
// provide the date and time for the RAWX log file filename
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial1); // M0 hardware serial
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

// Fast SD card logging using Bill Greiman's SdFat
// https://github.com/greiman/SdFat
// From FatFile.h:
//   * \note Data is moved to the cache but may not be written to the
//   * storage device until sync() is called.

#include <SPI.h>
#include <SdFat.h>
const uint8_t cardSelect = 4; // Adalogger uses D4 as the SD SPI select
SdFat sd;
SdFile rawx_dataFile;
#ifdef STATIC
char rawx_filename[] = "20000000/b_000000.ubx"; // base logfile
#else
char rawx_filename[] = "20000000/r_000000.ubx"; // rover logfile
#endif
char dirname[] = "20000000";
long bytes_written = 0;

// Define packet size, buffer and buffer pointer for SD card writes
const size_t SDpacket = 512;
uint8_t serBuffer[SDpacket];
size_t bufferPointer = 0;
int numBytes;

// Michael P. Flaga's MemoryFree:
// https://github.com/mpflaga/Arduino-MemoryFree
#include <MemoryFree.h>;

// Battery voltage
float vbat;
#define LOWBAT 3.55 // Low battery voltage

#include <RTCZero.h> // M0 Real Time Clock
RTCZero rtc; // Create an rtc object
volatile bool alarmFlag = false; // RTC alarm (interrupt) flag

// Count number of valid fixes before starting to log
#define maxvalfix 10 // Collect at least this many valid fixes before logging starts
int valfix = 0;

// Loop Steps
#define init          0
#define start_rawx    1
#define open_file     2
#define write_file    3
#define new_file      4
#define close_file    5
#define restart_file  6
int loop_step = init;

// UBX State
#define looking_for_B5          0
#define looking_for_62          1
#define looking_for_class       2
#define looking_for_ID          3
#define looking_for_length_LSB  4
#define looking_for_length_MSB  5
#define processing_payload      6
#define looking_for_checksum_A  7
#define looking_for_checksum_B  8
#define sync_lost               9
int ubx_state = looking_for_B5;
int ubx_length = 0;
int ubx_class = 0;
int ubx_ID = 0;
int ubx_checksum_A = 0;
int ubx_checksum_B = 0;
int ubx_expected_checksum_A = 0;
int ubx_expected_checksum_B = 0;

// Definitions for u-blox M8 UBX-format (binary) messages
// The message definitions need to include the 0xB5 and 0x62 sync chars 
// The message definitions don't contain the checksum bytes - these are calculated and appended by sendUBX
// Each message needs to have a length defined

// Set Nav Mode to Static
static const uint8_t setNavStatic[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const int len_setNav = 42;

// Set Nav Mode to Portable
static const uint8_t setNavPortable[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Pedestrian
static const uint8_t setNavPedestrian[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Automotive
static const uint8_t setNavAutomotive[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Sea
static const uint8_t setNavSea[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set Nav Mode to Airborne <1G
static const uint8_t setNavAir[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Set NMEA Config
// Set trackFilt to 1 to ensure course (COG) is always output
// Set Main Talker ID to 'GP' instead of 'GN'
static const uint8_t setNMEA[] = {
  0xb5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x20, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const int len_setNMEA = 26;

// Set GNSS Config (Causes a restart of the M8!)
#ifdef GLONASS
// GPS + Galileo + GLONASS + SBAS
static const uint8_t setGNSS[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05,
  0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01 };
/*
// GPS + Galileo + GLONASS
static const uint8_t setGNSS[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01,
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05,
  0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01 };
*/
#else
// GPS + Galileo + BeiDou + SBAS
static const uint8_t setGNSS[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05,
  0x06, 0x08, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x01 };
/*
// GPS + Galileo + BeiDou
static const uint8_t setGNSS[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01,
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05,
  0x06, 0x08, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x01 };
*/
#endif
static const int len_setGNSS = 66;

// Set Navigation/Measurement Rate (CFG-RATE): set measRate to 250msec; align to UTC time
static const uint8_t setRATE[] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xfa, 0x00, 0x01, 0x00, 0x00, 0x00 };
// Set Navigation/Measurement Rate (CFG-RATE): set measRate to 250msec; align to GPS time
//static const uint8_t setRATE[] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xfa, 0x00, 0x01, 0x00, 0x01, 0x00 };
// Set Navigation/Measurement Rate (CFG-RATE): set measRate to 1000msec; align to UTC time
static const uint8_t setRATE_1Hz[] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xe8, 0x03, 0x01, 0x00, 0x00, 0x00 };
// Set Navigation/Measurement Rate (CFG-RATE): set measRate to 10000msec; align to UTC time
static const uint8_t setRATE_10s[] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x00, 0x00 };
static const uint8_t len_setRATE = 12;

// Set RXM-RAWX Message Rate: once per measRate
static const uint8_t setRAWX[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x15, 0x01 };
// Set RXM-RAWX Message Rate: off
static const uint8_t setRAWXoff[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x15, 0x00 };
static const int len_setRAWX = 9;

// Set RXM-SFRBX Message Rate: once per measRate
static const uint8_t setSFRBX[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x13, 0x01 };
// Set RXM-SFRBX Message Rate: off
static const uint8_t setSFRBXoff[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x13, 0x00 };
static const int len_setSFRBX = 9;

// Set TIM-TM2 Message Rate: once per measRate
static const uint8_t setTIM[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0d, 0x03, 0x01 };
// Set TIM-TM2 Message Rate: off
static const uint8_t setTIMoff[] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0d, 0x03, 0x00 };
static const int len_setTIM = 9;

// Set port configuration - disable or enable UBX output messages on the UART
static const uint8_t setUBX[] = {
  0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00,
  0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t setUBXoff[] = {
  0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00,
  0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const int len_setUBX = 26;

// Send message in u-blox UBX format
// Calculates and appends the two checksum bytes
// Doesn't add the 0xb5 and 0x62 sync chars (these need to be included at the start of the message)
void sendUBX(const uint8_t *message, const int len) {
  int csum1 = 0; // Checksum bytes
  int csum2 = 0;
#ifdef DEBUG
    Serial.print("Sending UBX packet: 0x");
#endif
  for (int i=0; i<len; i++) { // For each byte in the message
    Serial1.write(message[i]); // Write the byte
#ifdef DEBUG // DEBUG: print the message byte in HEX format
    if (message[i] < 16) {Serial.print("0");}
    Serial.print(message[i], HEX);
    Serial.print(", 0x");
#endif
    if (i >= 2) { // Don't include the sync chars in the checksum
      csum1 = csum1 + message[i]; // Update the checksum bytes
      csum2 = csum2 + csum1;
    }
  }
  csum1 = csum1 & 0xff; // Limit checksums to 8-bits
  csum2 = csum2 & 0xff;
  Serial1.write((uint8_t)csum1); // Send the checksum bytes
  Serial1.write((uint8_t)csum2);
#ifdef DEBUG // DEBUG: print the checksum bytes in HEX format
  if (csum1 < 16) {Serial.print("0");}
  Serial.print((uint8_t)csum1, HEX);
  Serial.print(", 0x");
  if (csum2 < 16) {Serial.print("0");}
  Serial.println((uint8_t)csum2, HEX);
#endif
}

// TimerCounter3 functions to copy Serial1 receive data into SerialBuffer
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 16

// Set TC3 Interval (sec)
void setTimerInterval(float intervalS) {
  int compareValue = intervalS * CPU_HZ / TIMER_PRESCALER_DIV;
  if (compareValue > 65535) compareValue = 65535;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

// Start TC3 with a specified interval
void startTimerInterval(float intervalS) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 16
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerInterval(intervalS);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_SetPriority(TC3_IRQn, 3); // Set the TC3 interrupt priority to 3 (lowest)
  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

// TC3 Interrupt Handler
void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // copy any available Serial1 data into SerialBuffer
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    int available1 = Serial1.available(); // Check if there is any data waiting in the Serial1 RX buffer
    while (available1 > 0) { 
        SerialBuffer.store_char(Serial1.read()); // If there is, copy it into our RingBuffer
        available1--;
    }
  }
}

// NeoPixel Functions
// WB2812B blue LED has the highest forward voltage and is slightly dim at 3.3V. The red and green values are adapted accordingly (222 instead of 255).

#ifdef NeoPixel

void LED_off() // Turn NeoPixel off
{
  pixels.setPixelColor(0,0,0,0);
  pixels.show();
}

void LED_dim_white() // Set LED to dim white
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(222,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_blue() // Set LED to dim blue
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0,0,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_green() // Set LED to dim green
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0,222,0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_dim_cyan() // Set LED to dim cyan
{
  pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  pixels.setPixelColor(0, pixels.Color(0,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setBrightness(LED_Brightness); // Reset the LED brightness
}

void LED_white() // Set LED to white
{
  pixels.setPixelColor(0, pixels.Color(222,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_red() // Set LED to red
{
  pixels.setPixelColor(0, pixels.Color(222,0,0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_green() // Set LED to green
{
  pixels.setPixelColor(0, pixels.Color(0,222,0)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_blue() // Set LED to blue
{
  pixels.setPixelColor(0, pixels.Color(0,0,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_cyan() // Set LED to cyan
{
  pixels.setPixelColor(0, pixels.Color(0,222,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void LED_magenta() // Set LED to magenta
{
  pixels.setPixelColor(0, pixels.Color(222,0,255)); // Set color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

#endif

// RTC alarm interrupt
// Must be kept as short as possible. Update the alarm time in the main loop, not here.
void alarmMatch()
{
  alarmFlag = true; // Set alarm flag
}

void setup()
{
#ifdef NeoPixel
  // Initialise the NeoPixel
  pixels.begin(); // This initializes the NeoPixel library.
  delay(100); // Seems necessary to make the NeoPixel start reliably 
  pixels.setBrightness(LED_Brightness); // Initialize the LED brightness
  LED_off(); // Set NeoPixel off
#ifndef NoLED
  LED_dim_blue(); // Set NeoPixel to blue
#endif
#else
  // initialize digital pins RedLED and GreenLED as outputs.
  pinMode(RedLED, OUTPUT); // Red LED
  pinMode(GreenLED, OUTPUT); // Green LED
  digitalWrite(RedLED, LOW); // Turn Red LED off
  digitalWrite(GreenLED, LOW); // Turn Green LED off
#ifndef NoLED
  // flash red and green LEDs on reset
  for (int i=0; i <= 4; i++) {
    digitalWrite(RedLED, HIGH);
    delay(200);
    digitalWrite(RedLED, LOW);
    digitalWrite(GreenLED, HIGH);
    delay(200);
    digitalWrite(GreenLED, LOW);
  }
#endif
#endif

  // initialize swPin as an input for the stop switch
  pinMode(swPin, INPUT_PULLUP);

  Serial.begin(115200);

  delay(10000); // Allow 10 sec for user to open serial monitor (Comment this line if required)
  //while (!Serial); // OR Wait for user to run python script or open serial monitor (Comment this line as required)

  Serial.println("RAWX_Logger_4");
  Serial.println("Log GNSS RAWX data to SD card");
  Serial.println("Continuous Red indicates a problem or that logging has been stopped");

  Serial.print("Free Memory: ");
  Serial.println(freeMemory(), DEC);  // print how much RAM is available.
  
#ifndef NoLED
#ifdef NeoPixel
  LED_blue(); // Set NeoPixel to blue
#endif
#endif

  Serial.println("Initializing GNSS...");

  // u-blox M8 Init
  // 9600 is the default baud rate for u-blox M8
  GPS.begin(9600);
  // Change the NEO-M8T UART Baud rate
  // (DDC (I2C) and SPI rates can be set independently if required)
  // Allow both NMEA and UBX protocols (but not RTCM)
  // Disable autobauding
  GPS.sendCommand("$PUBX,41,1,0003,0003,115200,0*1C"); // 115200 baud
  //GPS.sendCommand("$PUBX,41,1,0003,0003,230400,0*1E"); // 230400 baud
  // Allow time for Baud rate change
  delay(1100);
  // Restart serial communications
  GPS.begin(115200); // 115200 baud
  //GPS.begin(230400); // 230400 baud
  sendUBX(setRAWXoff, len_setRAWX); // Disable RAWX messages
  delay(100);
  sendUBX(setSFRBXoff, len_setSFRBX); // Disable SFRBX messages
  delay(100);
  sendUBX(setTIMoff, len_setTIM); // Disable TIM messages
  delay(100);
  // Disable all NMEA messages except GGA and RMC
  GPS.sendCommand("$PUBX,40,GLL,0,0,0,0*5C"); // Disable GLL
  delay(100);
  GPS.sendCommand("$PUBX,40,ZDA,0,0,0,0*44"); // Disable ZDA
  delay(100);
  GPS.sendCommand("$PUBX,40,VTG,0,0,0,0*5E"); // Disable VTG
  delay(100);
  GPS.sendCommand("$PUBX,40,GSV,0,0,0,0*59"); // Disable GSV
  delay(100);
  GPS.sendCommand("$PUBX,40,GSA,0,0,0,0*4E"); // Disable GSA
  delay(100);
  GPS.sendCommand("$PUBX,40,GGA,0,1,0,0*5B");  // Enable GGA
  delay(100);
  GPS.sendCommand("$PUBX,40,RMC,0,1,0,0*46");  // Enable RMC
  delay(100); // Wait
#ifdef STATIC
  sendUBX(setNavStatic, len_setNav); // Set Static Navigation Mode (use this for the Base Logger)
#else
  // Select one mode for the mobile Rover Logger
  //sendUBX(setNavPortable, len_setNav); // Set Portable Navigation Mode
  //sendUBX(setNavPedestrian, len_setNav); // Set Pedestrian Navigation Mode
  //sendUBX(setNavAutomotive, len_setNav); // Set Automotive Navigation Mode
  //sendUBX(setNavSea, len_setNav); // Set Sea Navigation Mode
  sendUBX(setNavAir, len_setNav); // Set Airborne <1G Navigation Mode
#endif
  delay(100);
  sendUBX(setNMEA, len_setNMEA); // Set NMEA configuration (use GP prefix instead of GN otherwise parsing will fail)
  delay(100);
  sendUBX(setRATE_1Hz, len_setRATE); // Set Navigation/Measurement Rate to 1Hz
  delay(100);
  sendUBX(setGNSS, len_setGNSS); // Set GNSS - causes a reset of the M8!
  delay(3100);
  while(Serial1.available()){Serial1.read();} // Flush RX buffer so we don't confuse Adafruit_GPS with UBX acknowledgements

  Serial.println("GNSS initialized!");

#ifndef NoLED
#ifndef NeoPixel
  // flash the red LED during SD initialisation
  digitalWrite(RedLED, HIGH);
#endif
#endif

  // Initialise SD card
  Serial.println("Initializing SD card...");
  // See if the SD card is present and can be initialized
  if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
    Serial.println("Panic!! SD Card Init failed, or not present!");
    Serial.println("Waiting for reset...");
#ifndef NoLED
#ifdef NeoPixel
    LED_red(); // Set NeoPixel to red
#endif
#endif
    // don't do anything more:
    while(1);
  }
  Serial.println("SD Card initialized!");

#ifndef NoLED
#ifdef NeoPixel
  LED_dim_cyan(); // Set NeoPixel to dim cyan now that the SD card is initialised
#else
  // turn red LED off
  digitalWrite(RedLED, LOW);
#endif
#endif

  Serial.println("Waiting for GNSS fix...");

}

void loop() // run over and over again
{
  switch(loop_step) {
    case init: {
      // read data from the GNSS
      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
      // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          break; // we can fail to parse a sentence in which case we should just wait for another
    
#ifdef DEBUG
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" Quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
          Serial.print("HDOP: "); Serial.println(GPS.HDOP);
        }
#endif
  
        // read battery voltage
        vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
#ifdef DEBUG
        Serial.print("Battery(V): ");
        Serial.println(vbat, 2);
#endif
      
        // indicate GNSS fix
        if (GPS.fix) {
#ifndef NoLED
#ifdef NeoPixel
          LED_cyan(); // Set NeoPixel to cyan
#else
          digitalWrite(GreenLED, HIGH);
#endif
#endif
          // increment valfix and cap at maxvalfix
          // don't do anything fancy in terms of decrementing valfix as we want to keep logging even if the fix is lost
          valfix += 1;
          if (valfix > maxvalfix) valfix = maxvalfix;
        }
        else {
#ifndef NoLED
#ifdef NeoPixel
          LED_dim_cyan(); // Set NeoPixel to dim cyan
#else
          digitalWrite(GreenLED, LOW); // Turn green LED off
#endif
#endif
        }
  
        if (valfix == maxvalfix) { // wait until we have enough valid fixes
          
          // Set and start the RTC
          alarmFlag = false; // Make sure alarm flag is clear
          rtc.begin(); // Start the RTC
          rtc.setTime(GPS.hour, GPS.minute, GPS.seconds); // Set the time
          rtc.setDate(GPS.day, GPS.month, GPS.year); // Set the date
          rtc.setAlarmSeconds(0); // Set RTC Alarm Seconds to zero
          uint8_t nextAlarmMin = ((GPS.minute+INTERVAL)/INTERVAL)*INTERVAL; // Calculate next alarm minutes
          nextAlarmMin = nextAlarmMin % 60; // Correct hour rollover
          rtc.setAlarmMinutes(nextAlarmMin); // Set RTC Alarm Minutes
          rtc.enableAlarm(rtc.MATCH_MMSS); // Alarm Match on minutes and seconds
          rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt

          // check if voltage is > LOWBAT(V), if not then don't try to log any data
          if (vbat < LOWBAT) {
            Serial.println("Low Battery!");
            break;
          }

          // Disable the GPGGA and GPRMC messages; set the RAWX measurement rate
          GPS.sendCommand("$PUBX,40,GGA,0,0,0,0*5A");  // Disable GGA
          delay(100);
          GPS.sendCommand("$PUBX,40,RMC,0,0,0,0*47");  // Disable RMC
          delay(100);
          sendUBX(setRATE, len_setRATE); // Set Navigation/Measurement Rate
          delay(1100); // Wait
          while(Serial1.available()){Serial1.read();} // Flush RX buffer to clear UBX acknowledgement
          
          // Now that Serial1 should be idle and the buffer empty, start TC3 interrupts to copy all new data into SerialBuffer
          // Set the timer interval to 10 * 10 / 230400 = 0.000434 secs (10 bytes * 10 bits (1 start, 8 data, 1 stop) at 230400 baud)
          startTimerInterval(0.000434); 
          
          loop_step = start_rawx; // start rawx messages
        }
      }
    }
    break;

    // (Re)Start RAWX messages
    case start_rawx: {
      sendUBX(setRAWX, len_setRAWX); // Set RXM-RAWX message rate
      sendUBX(setSFRBX, len_setSFRBX); // Set RXM-SFRBX message rate
      sendUBX(setTIM, len_setTIM); // Set TIM-TM2 message rate

      bufferPointer = 0; // (Re)initialise bufferPointer

      while (SerialBuffer.available() < 30) { ; } // Wait for three UBX acknowledgements' worth of data (3 x 10 = 30 bytes)
      
      for (int x=0;x<3;x++) { // Check ten bytes three times
        for (int y=0;y<10;y++) { // Get ten bytes
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Add a character to serBuffer
          bufferPointer++; // Increment the pointer   
        }
        // Now check if these bytes were an acknowledgement
        if ((serBuffer[bufferPointer - 10] == 0xB5) and (serBuffer[bufferPointer - 9] == 0x62) and (serBuffer[bufferPointer - 8] == 0x05)) {
          // This must be an acknowledgement so simply ignore it and decrement bufferPointer by 10
          bufferPointer -= 10;
        }
      }

      loop_step = open_file; // start logging rawx data
    }
    break;

    // Open the log file
    case open_file: {
      
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
      
      // Do the divides to convert date and time to char
      char secT = RTCseconds/10 + '0';
      char secU = RTCseconds%10 + '0';
      char minT = RTCminutes/10 + '0';
      char minU = RTCminutes%10 + '0';
      char hourT = RTChours/10 + '0';
      char hourU = RTChours%10 + '0';
      char dayT = RTCday/10 +'0';
      char dayU = RTCday%10 +'0';
      char monT = RTCmonth/10 +'0';
      char monU = RTCmonth%10 +'0';
      char yearT = RTCyear/10 +'0';
      char yearU = RTCyear%10 +'0';
  
      // filename is limited to 8.3 characters so use format: YYYYMMDD/r_HHMMSS.ubx or YYYYMMDD/b_HHMMSS.ubx
      rawx_filename[2] = yearT;
      rawx_filename[3] = yearU;
      rawx_filename[4] = monT;
      rawx_filename[5] = monU;
      rawx_filename[6] = dayT;
      rawx_filename[7] = dayU;
      rawx_filename[11] = hourT;
      rawx_filename[12] = hourU;
      rawx_filename[13] = minT;
      rawx_filename[14] = minU;
      rawx_filename[15] = secT;
      rawx_filename[16] = secU;
      
      dirname[2] = yearT;
      dirname[3] = yearU;
      dirname[4] = monT;
      dirname[5] = monU;
      dirname[6] = dayT;
      dirname[7] = dayU;

      // flash red LED to indicate SD write (leave on if an error occurs)
      // or flash NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_green(); // Set the NeoPixel to green
#else
      LED_off(); // Turn NeoPixel off if NoLogLED
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off for NoLogLED
      digitalWrite(GreenLED, LOW); // Turn the green LED off for NoLogLED
#endif
#endif
#endif

     // try to create subdirectory (even if it exists already)
      sd.mkdir(dirname);
      
      // Open the rawx file for fast writing
      if (rawx_dataFile.open(rawx_filename, O_CREAT | O_WRITE | O_EXCL)) {
        Serial.print("Logging to ");
        Serial.println(rawx_filename);
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("Panic!! Error opening RAWX file!");
        Serial.println("Waiting for reset...");
#ifndef NoLED
#ifdef NeoPixel
      LED_red(); // Set the NeoPixel to red to indicate a problem
#else
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate a problem
#endif
#endif
        // don't do anything more:
        while(1);
      }

#ifdef DEBUG
      // Set the log file creation time
      if (!rawx_dataFile.timestamp(T_CREATE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file create timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_CREATE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif

      // Now that SD write is complete
      // Turn the Red LED off or set NeoPixel to dim green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_dim_green();
#endif
#else
      digitalWrite(RedLED, LOW); // turn red LED off
#endif
#endif

      bytes_written = 0; // Clear bytes_written

      ubx_state = looking_for_B5; // set ubx_state to expect B5
      ubx_length = 0; // set ubx_length to zero
          
      loop_step = write_file; // start logging rawx data
    }
    break;

    // Stuff bytes into serBuffer and write when we have reached SDpacket
    case write_file: {
      if (SerialBuffer.available()) {
        uint8_t c = SerialBuffer.read_char();
        serBuffer[bufferPointer] = c;
        bufferPointer++;
        if (bufferPointer == SDpacket) {
          bufferPointer = 0;

          // flash red LED to indicate SD write (leave on if an error occurs)
          // or flash NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
          LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
          digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
          
          numBytes = rawx_dataFile.write(&serBuffer, SDpacket);

          // Now that SD write is complete
          // Turn the Red LED off or set NeoPixel to dim green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
          LED_dim_green();
#endif
#else
          digitalWrite(RedLED, LOW); // turn red LED off
#endif
#endif
          
          //rawx_dataFile.sync(); // Sync the file system
          bytes_written += SDpacket;
#ifdef DEBUG
          if (numBytes != SDpacket) {
            Serial.print("SD write error! Write size was ");
            Serial.print(SDpacket);
            Serial.print(". ");
            Serial.print(numBytes);
            Serial.println(" were written.");
          }
#endif
//#ifdef DEBUG
//          Serial.print("SD Write: ");
//          Serial.print(SDpacket);
//          Serial.println(" Bytes");
//          Serial.print(bytes_written);
//          Serial.println(" Bytes written so far");
//#endif
        }
        // Process data bytes according to ubx_state:
        // Sync Char 1: 0xB5
        // Sync Char 2: 0x62
        // Class byte
        // ID byte
        // Length: two bytes, little endian
        // Payload: length bytes
        // Checksum: two bytes
        // Only allow a new file to be opened when a complete packet has been processed and ubx_state has returned to "looking_for_B5"
        // Or when a data error is detected (sync_lost)
        switch (ubx_state) {
          case (looking_for_B5): {
            if (c == 0xB5) { // Have we found Sync Char 1 (0xB5) when we were expecting one?
              ubx_state = looking_for_62; // Now look for Sync Char 2 (0x62)
            }
            else {
              Serial.println("Panic!! Was expecting Sync Char 0xB5 but did not receive one!");
              ubx_state = sync_lost;
            }
          }
          break;
          case (looking_for_62): {
            if (c == 0x62) { // Have we found Sync Char 2 (0x62) when we were expecting one?
              ubx_expected_checksum_A = 0; // Reset the expected checksum
              ubx_expected_checksum_B = 0;
              ubx_state = looking_for_class; // Now look for Class byte
            }
            else {
              Serial.println("Panic!! Was expecting Sync Char 0x62 but did not receive one!");
              ubx_state = sync_lost;
            }
          }
          break;
          case (looking_for_class): {
            ubx_class = c;
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = looking_for_ID; // Now look for ID byte
#ifdef DEBUG
            // Class syntax checking
            if ((ubx_class != 0x02) and (ubx_class != 0x0D)) {
              Serial.println("Panic!! Was expecting Class of 0x02 or 0x0D but did not receive one!");
              ubx_state = sync_lost;
            }
#endif
          }
          break;
          case (looking_for_ID): {
            ubx_ID = c;
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = looking_for_length_LSB; // Now look for length LSB
#ifdef DEBUG
            // ID syntax checking
            if ((ubx_class == 0x02) and ((ubx_ID != 0x15) and (ubx_ID != 0x13))) {
              Serial.println("Panic!! Was expecting ID of 0x15 or 0x13 but did not receive one!");
              ubx_state = sync_lost;
            }
            else if ((ubx_class == 0x0D) and (ubx_ID != 0x03)) {
              Serial.println("Panic!! Was expecting ID of 0x03 but did not receive one!");
              ubx_state = sync_lost;
            }
#endif
          }
          break;
          case (looking_for_length_LSB): {
            ubx_length = c; // Store the length LSB
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = looking_for_length_MSB; // Now look for length MSB
          }
          break;
          case (looking_for_length_MSB): {
            ubx_length = ubx_length + (c * 256); // Add the length MSB
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_state = processing_payload; // Now look for payload bytes (length: ubx_length)
          }
          break;
          case (processing_payload): {
            ubx_length = ubx_length - 1; // Decrement length by one
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            if (ubx_length == 0) {
              ubx_expected_checksum_A = ubx_expected_checksum_A & 0xff; // Limit checksums to 8-bits
              ubx_expected_checksum_B = ubx_expected_checksum_B & 0xff;
              ubx_state = looking_for_checksum_A; // If we have received length payload bytes, look for checksum bytes
            }
          }
          break;
          case (looking_for_checksum_A): {
            ubx_checksum_A = c;
            ubx_state = looking_for_checksum_B;
          }
          break;
          case (looking_for_checksum_B): {
            ubx_checksum_B = c;
            ubx_state = looking_for_B5; // All bytes received so go back to looking for a new Sync Char 1 unless there is a checksum error
            if ((ubx_expected_checksum_A != ubx_checksum_A) or (ubx_expected_checksum_B != ubx_checksum_B)) {
              Serial.println("Panic!! Checksum error!");
              ubx_state = sync_lost;
            }
          }
          break;
        }
      }
      else {
        // read battery voltage
        vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
      }
      // Check if the stop button has been pressed or battery is low
      // or if there has been an RTC alarm and it is time to open a new file
      if ((digitalRead(swPin) == LOW) or (vbat < LOWBAT)) {
        loop_step = close_file; // now close the file for the last time
        break;
      }
      else if ((alarmFlag == true) and (ubx_state == looking_for_B5)) {
        loop_step = new_file; // now close the file and open a new one
        break;
      }
      else if (ubx_state == sync_lost) {
        loop_step = restart_file; // Sync has been lost so stop RAWX messages and open a new file before restarting RAWX
      }
    }
    break;

    // Close the current log file and open a new one without stopping RAWX messages
    case new_file: {

        // flash red LED to indicate SD write (leave on if an error occurs)
        // or flash NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif

      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {

        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
//        Serial.print("Final SD Write: ");
//        Serial.print(bufferPointer);
//        Serial.println(" Bytes");
//        Serial.print(bytes_written);
//        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif

#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_cyan(); // Set NeoPixel to cyan
#endif
#else
      digitalWrite(RedLED, LOW); // turn red LED off
#endif
#endif

      Serial.println("File closed!");
      // An RTC alarm was detected, so set the RTC alarm time to the next INTERVAL and loop back to open_file.
      // We only receive an RTC alarm on a minute mark, so it doesn't matter that the RTC seconds will have moved on at this point.
      alarmFlag = false; // Clear the RTC alarm flag
      uint8_t rtc_mins = rtc.getMinutes(); // Read the RTC minutes
      rtc_mins = rtc_mins + INTERVAL; // Add the INTERVAL to the RTC minutes
      rtc_mins = rtc_mins % 60; // Correct for hour rollover
      rtc.setAlarmMinutes(rtc_mins); // Set next alarm time (minutes only - hours are ignored)

      loop_step = open_file; // loop round again and open a new file
      bytes_written = 0; // Clear bytes_written
    }
    break;

    // Disable RAWX messages, save any residual data and close the file for the last time
    case close_file: {
      sendUBX(setRAWXoff, len_setRAWX); // Disable messages
      sendUBX(setSFRBXoff, len_setSFRBX); // Disable messages
      sendUBX(setTIMoff, len_setTIM); // Disable messages
      int waitcount = 0;
      
      // flash red LED to indicate SD write
      // or flash NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif

      // leave 30 bytes in the serial buffer as this _should_ be the three message acknowledgements (3 x 10 bytes)
      while (waitcount < dwell) { // Wait for residual data
        while (SerialBuffer.available() > 30) { // Leave 30 bytes in the serial buffer
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
          bufferPointer++;
          if (bufferPointer == SDpacket) { // Write a full packet
            bufferPointer = 0;
            numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
            //rawx_dataFile.sync(); // Sync the file system
            bytes_written += SDpacket;
#ifdef DEBUG
            if (numBytes != SDpacket) {
              Serial.print("SD write error! Write size was ");
              Serial.print(SDpacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
//            Serial.print("SD Write: ");
//            Serial.print(SDpacket);
//            Serial.println(" Bytes");
//            Serial.print(bytes_written);
//            Serial.println(" Bytes written so far");
#endif
          }
        }
        waitcount++;
        delay(1);
      }
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Penultimate/Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
        Serial.print(bytes_written);
        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // We now have exactly 30 bytes left in the buffer. Let's check if they contain acknowledgements or residual data.
      // If they contain residual data, save it to file. This means we have probably already saved acknowledgement(s)
      // to file and there's now very little we can do about that except hope that RTKLib knows to ignore them!
      for (int x=0;x<3;x++) { // Check ten bytes three times
        for (int y=0;y<10;y++) { // Add ten bytes
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Add a character to serBuffer
          bufferPointer++; // Increment the pointer   
        }
        // Now check if these bytes were an acknowledgement
        if ((serBuffer[bufferPointer - 10] == 0xB5) and (serBuffer[bufferPointer - 9] == 0x62) and (serBuffer[bufferPointer - 8] == 0x05)) {
          // This must be an acknowledgement so simply ignore it and decrement bufferPointer by 10
          bufferPointer -= 10;
        }
      }
      // If the last 30 bytes did contain any data, write it to file now
      if (bufferPointer > 0) {
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      
      // Either the battery is low or the user pressed the stop button. So, just wait for a reset.
#ifndef NoLED
#ifdef NeoPixel
        LED_red(); // Set the NeoPixel to red
#else
        digitalWrite(RedLED, HIGH); // Turn the red LED on
#endif
#endif
      Serial.println("Waiting for reset...");
      while(1); // Wait for reset
    }
    break;

    // RAWX data lost sync so disable RAWX messages, save any residual data, close the file, open another and restart RAWX messages
    // Don't update the next RTC alarm - leave it as it is
    case restart_file: {
      sendUBX(setRAWXoff, len_setRAWX); // Disable messages
      sendUBX(setSFRBXoff, len_setSFRBX); // Disable messages
      sendUBX(setTIMoff, len_setTIM); // Disable messages
      int waitcount = 0;
      
      // flash red LED to indicate SD write
      // or flash NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_green(); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif

      // leave 30 bytes in the serial buffer as this _should_ be the three message acknowledgements (3 x 10 bytes)
      while (waitcount < dwell) { // Wait for residual data
        while (SerialBuffer.available() > 30) { // Leave 30 bytes in the serial buffer
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
          bufferPointer++;
          if (bufferPointer == SDpacket) { // Write a full packet
            bufferPointer = 0;
            numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
            //rawx_dataFile.sync(); // Sync the file system
            bytes_written += SDpacket;
#ifdef DEBUG
            if (numBytes != SDpacket) {
              Serial.print("SD write error! Write size was ");
              Serial.print(SDpacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
//            Serial.print("SD Write: ");
//            Serial.print(SDpacket);
//            Serial.println(" Bytes");
//            Serial.print(bytes_written);
//            Serial.println(" Bytes written so far");
#endif
          }
        }
        waitcount++;
        delay(1);
      }
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Penultimate/Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
        Serial.print(bytes_written);
        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // We now have exactly 30 bytes left in the buffer. Let's check if they contain acknowledgements or residual data.
      // If they contain residual data, save it to file. This means we have probably already saved acknowledgement(s)
      // to file and there's now very little we can do about that except hope that RTKLib knows to ignore them!
      for (int x=0;x<3;x++) { // Check ten bytes three times
        for (int y=0;y<10;y++) { // Add ten bytes
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Add a character to serBuffer
          bufferPointer++; // Increment the pointer   
        }
        // Now check if these bytes were an acknowledgement
        if ((serBuffer[bufferPointer - 10] == 0xB5) and (serBuffer[bufferPointer - 9] == 0x62) and (serBuffer[bufferPointer - 8] == 0x05)) {
          // This must be an acknowledgement so simply ignore it and decrement bufferPointer by 10
          bufferPointer -= 10;
        }
      }
      // If the last 30 bytes did contain any data, write it to file now
      if (bufferPointer > 0) {
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");

#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      LED_cyan(); // Set NeoPixel to cyan
#endif
#else
      digitalWrite(RedLED, LOW); // turn red LED off
#endif
#endif

      loop_step = start_rawx; // loop round again and restart rawx messages before opening a new file
    }
    break;  
  }
}
