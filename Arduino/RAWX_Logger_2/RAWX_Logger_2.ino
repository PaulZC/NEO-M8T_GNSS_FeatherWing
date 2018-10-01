// RAWX_Logger V2

// Logs RXM-RAWX, RXM-SFRBX and TIM-TM2 data from u-blox NEO-M8T GNSS to SD card

// Changes to a new log file every INTERVAL minutes

// Define how long we should log in minutes before changing to a new file
// Sensible values are: 5, 10, 15, 20, 30, 60
// Must be <= 60 (or RTC alarm code needs to be updated to match on HHMMSS)
const int INTERVAL = 15;

// Define how long we should wait in msec (approx.) for residual RAWX data before closing a log file
// For a measurement rate of 4Hz (250msec), 300msec is a sensible value. i.e. slightly more than one measurement interval
const int dwell = 300;

// The RAWX log file has a .ubx suffix instead of .bin
// The log filename starts with "r_" for the rover and "b_" for the static base

// Define if this is the static base logger
#define STATIC // Comment this line out for the mobile rover logger

// This code is written for the Adalogger M0 Feather
// https://www.adafruit.com/products/2796
// https://learn.adafruit.com/adafruit-feather-m0-adalogger

// GNSS data is provided by Paul's u-blox NEO-M8T FeatherWing
// https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing

// Solder the GRESET link so that pressing the reset button
// resets the NEO-M8T as well as the Adalogger.

// Choose a good quality SD card. Some cheap cards can't handle the write rate.
// Ensure the card is formatted as FAT32.

// You need to enlarge the serial receive buffer to avoid buffer
// overruns while data is being written to the SD card.
// For the Adafruit Feather M0 Adalogger (SAMD):
// See this post by MartinL: https://forum.arduino.cc/index.php?topic=365220.0
// Under Windows, edit: C:\Users\ ...your_user... \AppData\Local\Arduino15\packages\adafruit\hardware\samd\1.0.19\cores\arduino\RingBuffer.h
// and change: #define SERIAL_BUFFER_SIZE 164
// to:         #define SERIAL_BUFFER_SIZE 8192
// Check the reported freeMemory before and after to make sure the change has been successful
// (You should find that you've lost twice as much memory as expected! (Tx and Rx buffers both increase in size!))

// Define the GNSS configuration: GPS + Galileo + either GLONASS or BeiDou
#define GLONASS // Comment this line out to use BeiDou

// Send serial debug messages
//#define DEBUG // Comment this line out to disable debug messages

// Connect a normally-open push-to-close switch between swPin and GND.
// Press it to stop logging and close the log file.
#define swPin 15 // Digital Pin 15 (0.2" away from the GND pin on the Adalogger)

// LEDs
#define RedLED 13 // The red LED on the Adalogger is connected to Digital Pin 13
#define GreenLED 8 // The green LED on the Adalogger is connected to Digital Pin 8

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

// Include Real Time Clock support for the M0
// https://github.com/arduino-libraries/RTCZero
#include <RTCZero.h> // M0 Real Time Clock
RTCZero rtc; // Create an rtc object
volatile bool alarmFlag = false; // RTC alarm (interrupt) flag

// Count number of valid fixes before starting to log
#define maxvalfix 10 // Collect at least this many valid fixes before logging starts
int valfix = 0;

// Loop Steps
#define init        0
#define open_file   1
#define write_file  2
#define close_file  3
int loop_step = init;

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

// RTC alarm interrupt
// Must be kept as short as possible. Update the alarm time in the main loop, not here.
void alarmMatch()
{
  alarmFlag = true; // Set alarm flag
}

void setup()
{
  // initialize digital pins RedLED and GreenLED as outputs.
  pinMode(RedLED, OUTPUT); // Red LED
  pinMode(GreenLED, OUTPUT); // Green LED
  // flash red and green LEDs on reset
  for (int i=0; i <= 4; i++) {
    digitalWrite(RedLED, HIGH);
    delay(200);
    digitalWrite(RedLED, LOW);
    digitalWrite(GreenLED, HIGH);
    delay(200);
    digitalWrite(GreenLED, LOW);
  }

  // initialize swPin as an input for the stop switch
  pinMode(swPin, INPUT_PULLUP);

  delay(10000); // Allow 10 sec for user to open serial monitor (Comment this line if required)
  //while (!Serial); // OR Wait for user to run python script or open serial monitor (Comment this line as required)

  Serial.begin(115200);

  Serial.println("RAWX_Logger_2");
  Serial.println("Log GNSS RAWX data to SD card");
  Serial.println("Green LED = Initial GNSS Fix");
  Serial.println("Red LED Flash = SD Write");
  Serial.println("Continuous Red indicates a problem or that logging has been stopped");

  Serial.print("Free Memory: ");
  Serial.println(freeMemory(), DEC);  // print how much RAM is available.
  
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
  sendUBX(setNavPortable, len_setNav); // Set Portable Navigation Mode
  //sendUBX(setNavPedestrian, len_setNav); // Set Pedestrian Navigation Mode
  //sendUBX(setNavAutomotive, len_setNav); // Set Automotive Navigation Mode
  //sendUBX(setNavSea, len_setNav); // Set Sea Navigation Mode
  //sendUBX(setNavAir, len_setNav); // Set Airborne <1G Navigation Mode
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

  // flash the red LED during SD initialisation
  digitalWrite(RedLED, HIGH);

  // Initialise SD card
  Serial.println("Initializing SD card...");
  // See if the SD card is present and can be initialized
  if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
    Serial.println("Panic!! SD Card Init failed, or not present!");
    Serial.println("Waiting for reset...");
    // don't do anything more:
    while(1);
  }
  Serial.println("SD Card initialized!");

  bufferPointer = 0; // Make sure bufferPointer is initialised (should be redundant!)

  // turn red LED off
  digitalWrite(RedLED, LOW);
  
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
      
        // turn green LED on to indicate GNSS fix
        if (GPS.fix) {
          digitalWrite(GreenLED, HIGH);
          // increment valfix and cap at maxvalfix
          // don't do anything fancy in terms of decrementing valfix as we want to keep logging even if the fix is lost
          valfix += 1;
          if (valfix > maxvalfix) valfix = maxvalfix;
        }
        else {
          digitalWrite(GreenLED, LOW);
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
          
          loop_step = open_file; // start logging rawx data
        }
      }
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
      digitalWrite(RedLED, HIGH);

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

      digitalWrite(RedLED, LOW); // turn red LED off

      bytes_written = 0; // Clear bytes_written
      bufferPointer = 0; // (Re)initialise bufferPointer

      // Start RAWX messages
      sendUBX(setRAWX, len_setRAWX); // Set RXM-RAWX message rate
      sendUBX(setSFRBX, len_setSFRBX); // Set RXM-SFRBX message rate
      sendUBX(setTIM, len_setTIM); // Set TIM-TM2 message rate

      while (Serial1.available() < 30) { ; } // Wait for three UBX acknowledgements' worth of data (3 x 10 = 30 bytes)
      
      for (int x=0;x<3;x++) { // Check ten bytes three times
        for (int y=0;y<10;y++) { // Get ten bytes
          serBuffer[bufferPointer] = Serial1.read(); // Add a character to serBuffer
          bufferPointer++; // Increment the pointer   
        }
        // Now check if these bytes were an acknowledgement
        if ((serBuffer[bufferPointer - 10] == 0xB5) and (serBuffer[bufferPointer - 9] == 0x62) and (serBuffer[bufferPointer - 8] == 0x05)) {
          // This must be an acknowledgement so simply ignore it and decrement bufferPointer by 10
          bufferPointer -= 10;
        }
      }
      
      loop_step = write_file; // start logging rawx data
    }
    break;

    // Stuff bytes into serBuffer and write when we have reached SDpacket
    case write_file: {
      if (Serial1.available()) {
        serBuffer[bufferPointer] = Serial1.read();
        bufferPointer++;
        if (bufferPointer == SDpacket) {
          bufferPointer = 0;
          digitalWrite(RedLED, HIGH); // flash red LED
          numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
          //rawx_dataFile.sync(); // Sync the file system
          bytes_written += SDpacket;
          digitalWrite(RedLED, LOW);
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
      }
      else {
        // read battery voltage
        vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
        // check if the stop button has been pressed or battery is low
        // or if there has been an RTC alarm and it is time to open a new file
        if ((digitalRead(swPin) == LOW) or (vbat < LOWBAT) or (alarmFlag == true)) {
          loop_step = close_file; // now close the file
          break;
        }
      }
    }
    break;

    // Disable RAWX messages, save any residual data and close the file
    case close_file: {
      sendUBX(setRAWXoff, len_setRAWX); // Disable messages
      sendUBX(setSFRBXoff, len_setSFRBX); // Disable messages
      sendUBX(setTIMoff, len_setTIM); // Disable messages
      int waitcount = 0;
      // leave 30 bytes in the serial buffer as this _should_ be the three message acknowledgements (3 x 10 bytes)
      while (waitcount < dwell) { // Wait for residual data
        while (Serial1.available() > 30) { // Leave 30 bytes in the serial buffer
          serBuffer[bufferPointer] = Serial1.read(); // Put extra bytes into serBuffer
          bufferPointer++;
          if (bufferPointer == SDpacket) { // Write a full packet
            bufferPointer = 0;
            digitalWrite(RedLED, HIGH); // flash red LED
            numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
            //rawx_dataFile.sync(); // Sync the file system
            digitalWrite(RedLED, LOW);
            bytes_written += SDpacket;
#ifdef DEBUG
            if (numBytes != SDpacket) {
              Serial.print("SD write error! Write size was ");
              Serial.print(SDpacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
            Serial.print("SD Write: ");
            Serial.print(SDpacket);
            Serial.println(" Bytes");
            Serial.print(bytes_written);
            Serial.println(" Bytes written so far");
#endif
          }
        }
        waitcount++;
        delay(1);
      }
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
        digitalWrite(RedLED, HIGH); // flash red LED
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
        digitalWrite(RedLED, LOW);
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
          serBuffer[bufferPointer] = Serial1.read(); // Add a character to serBuffer
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
        digitalWrite(RedLED, HIGH); // flash red LED
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
        digitalWrite(RedLED, LOW);
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
      digitalWrite(RedLED, HIGH); // flash red LED
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
      digitalWrite(RedLED, LOW);
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      if (alarmFlag == true) {
        // If an RTC alarm was detected, set the RTC alarm time to the next INTERVAL and loop back to open_file.
        // We only receive an RTC alarm on a minute mark, so it doesn't matter that the RTC seconds will have moved on at this point.
        alarmFlag = false; // Clear the RTC alarm flag
        uint8_t rtc_mins = rtc.getMinutes(); // Read the RTC minutes
        rtc_mins = rtc_mins + INTERVAL; // Add the INTERVAL to the RTC minutes
        rtc_mins = rtc_mins % 60; // Correct for hour rollover
        rtc.setAlarmMinutes(rtc_mins); // Set next alarm time (minutes only - hours are ignored)
        loop_step = open_file; // loop round again and open a new file
      }
      else {
        // Either the battery is low or the user pressed the stop button. So, just wait for a reset.
        digitalWrite(RedLED, HIGH); // leave the red led on
        Serial.println("Waiting for reset...");
        while(1); // Wait for reset
      }
    }
    break;
  }
}

