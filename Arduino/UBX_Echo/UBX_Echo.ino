// UBX Echo

// Simple serial echo for the u-blox NEO-M8T GNSS FeatherWing
// https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing

// ** Changes the NEO-M8T Baud rate from 9600 to 115200 **
// (To allow e.g. RAWX logging)

// Designed to run on the Adafruit Feather M0 Adalogger
// https://www.adafruit.com/product/2796
// https://learn.adafruit.com/adafruit-feather-m0-adalogger

// Code is based on:
// https://github.com/adafruit/Adafruit_GPS/tree/master/examples/GPS_HardwareSerial_EchoTest

void setup()
{
  Serial1.begin(9600); // Start off using 9600 Baud to talk to the NEO-M8T
  Serial.begin(115200); // Use 115200 baud to talk to the PC

  // give the NEO-M8T time to power up and/or give the user time to open a serial monitor
  // by flashing the LEDs for 10 seconds
  pinMode(13, OUTPUT); // Red LED
  pinMode(8, OUTPUT); // Green LED
  // flash red and green LEDs on reset
  for (int i=0; i <= 25; i++) {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    digitalWrite(8, HIGH);
    delay(200);
    digitalWrite(8, LOW);
  }

  // Wait for user to run python script or open serial monitor
  // Comment out if you want the code to run without waiting
  //while (!Serial);

  // Change the NEO-M8T UART Baud rate to 115200
  // (DDC (I2C) and SPI rates can be set independantly if required)
  // Allow both NMEA and UBX protocols (but not RTCM)
  // Disable autobauding
  Serial1.write("$PUBX,41,1,0003,0003,115200,0*1C\r\n");

  // Allow time for Baud rate change
  delay(1000);

  // Restart serial communications at 115200 Baud
  Serial1.begin(115200);
}
     
void loop() // run over and over again
{
  if (Serial.available()) {
    digitalWrite(13, HIGH); // Flash red LED
    char c = Serial.read();
    Serial1.write(c);
    digitalWrite(13, LOW);
  }
  if (Serial1.available()) {
    digitalWrite(8, HIGH); // Flash green LED
    char c = Serial1.read();
    Serial.write(c);
    digitalWrite(8, LOW);
  }  
}

