/** ---------- TMP102 Proof Code ------------------------
This program proves out the TMP102 temperature sensor.
This will be converted to a library

Controller is Teensy++ 2.0 but will also work with Arduino
We are running at 3.3V and therefore 8 MHz

Why the TMP102? The TMP102 is two-wire- and SMBus 
interface-compatible, and is specified over a 
temperature range of –40°C to +125°C.
TINY SOT563 PACKAGE
ACCURACY: 0.5°C (–25°C to +85°C) (a bit worse up to 125C)
12-bit direct to digital conversion
Alarm output
Up to four devices on one bus.
Reported value agrees to within 0.25 deg C with Fluke 80T-150U

Copyright 2013 Systronix Inc www.systronix.com

---------------------------------------------------- **/

/** ---------- WIZnet W5100 in W812MJ module ----------

---------------------------------------------------- **/

/**
NOTES ABOUT WIRE

Wire.endTransmission() seems to be only intended for use with a master write.
Wire.requestFrom() is used to get bytes from a slave, with read().
beginTransmission() followed by read() does not work. Slave address gets sent, then nothing else. As if the read commands get ignored.
I guess reads are not a "Transmission".

So to read three bytes, do a requestFrom(address, 2, true). << insists the first param is int.  
Compiler has major whines if called as shown in the online Wire reference.

**/
 
/** ---------- REVISIONS ----------

2013 Jul 20 bboyes: adding Ethernet support on Teensy++2 and Wiznet 812 on adapter. Simple web server works!
2013 Feb 13 bboyes: adding more methods to set mode and sample temperature
2013 Feb 11 bboyes: reducing output and changing format to allow for easy import into spreadsheet
2013 Feb 08 bboyes: 13-bit extended mode is working. Simulated data used to test entire range from +150C to -55C
            SD (shutdown) and OS (one-shot conversion) bits work in low power shutdown mode
2013 Feb 04 bboyes: Arduino 1.0.3 and Teensy 1v12 (not beta). got readRegister to work and discovered some surprising things 
            about the Arduino Wire library.
2013 Jan 16 bboyes: start
--------------------------------**/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <util.h>

/** --------  Addressing --------
TMP102 base address is 0x48 (B 1001 000x) where x is R/W 
This is confusing because the address part is just the the upper 7 bits, so if considered
alone, and right-justified, that is  B 0100 1000 which is 0x48 which is how Arduino wants it.
if ADDR is GND, address is 0x48
if ADDR is VDD, address is 0x49
if ADDR is SDA, address is 0x4A
if ADDR is SCL, address is 0x4B

The two lsb of the pointer register hold the register bits, which 
are used to address one of the four directly-accessible registers.

There are four pointer addresses:
00  Temperature Register (Read Only) 12-13 bits in ms position
    bit 0 in LS byte is 1 if in 13-bit 'extended mode' (EM)
    If temp is positive (msb=0) the value is just the binary value.
    If temp is negative (msb=0) the value is in 2's complement form,
    so take the whole binary value, complement it, and add 1.
01  Configuration Register (Read/Write) 16 bits, MSB first
10  TLOW Limit Register (Read/Write)
11  THIGH Limit Register (Read/Write)

One lsb is 0.0625°C
Negative numbers are binary twos complement format.

On our BHF boards the ALERT pin is Teensy++2 module pin 32, signal PE7, Arduino pin #19
*/

const uint8_t TMP102_ADDR = 0x48;  // base address (four possible values)

#define TMP102_REG_TEMP_REG 0x00  // 16-bit temperature register, read only
#define TMP102_REG_CONF_REG 0x01  // 16-bit config register, read/write
#define TMP102_REG_TLOW_REG 0x02  // 16-bit Tlow register, read/write
#define TMP102_REG_THIGH_REG 0x03  // 16-bit Thigh register, read/write

/** --------  Configuration --------
OR these bits together appropriately, one choice from each option group
and write to the config register for the desired option
This is a a 16-bit register, bits 15..0 but bits 3..0 are not used and read as zeros
I have used the same designation as the data sheet, to be consistent,
instead of making names longer and perhaps more readable; 
for example TMP102_CFG_AL = 'AL', the Alert config bit
*/

/*
  0x60A0 is the POR default read value of the configuration register.
  12-bit resolution (but these bits 14,13 are read only)
  continuous conversions at 4Hz rate
  Alert bit = 1  (but this bit 5 is read only)
  So writing to only the bits of this default value that are writable,
  we could write 0x0080, which would still read back as 0x60A0
*/
#define TMP102_CFG_DEFAULT_RD 0x60A0
#define TMP102_CFG_DEFAULT_WR 0x0080

/* One-shot/Conversion Ready is Config bit 15
  When in shutdown mode (SD=1), setting OS starts a single conversion
  during which OS reads '0'. When conversion is complete OS = 1.
  A single conversion typically takes 26ms and a read can take
  place in less than 20ms. When using One-Shot mode,
  30 or more conversions per second are possible.
  Default is OS = 0.
*/
#define TMP102_CFG_OS 0x08000  // Start a single conversion (if in SD mode)

// Resolution is Config bits 14,13
// read-only bits, '11' = 12-bit resolution
#define TMP102_CFG_RES 0x6000  // if both bits set, 12-bit mode *default*

// Fault Queue Config bits 12,11
// how many faults generate an Alert based on T-high and T-low registers
#define TMP102_CFG_FLTQ_6 0x01800  // 6 consecutive faults 
#define TMP102_CFG_FLTQ_4 0x01000  // 4 consecutive faults 
#define TMP102_CFG_FLTQ_2 0x00800  // 2 consecutive faults 
#define TMP102_CFG_FLTQ_1 0x00000  // 1 consecutive fault *default*

// Polarity bit is Config bit 10, adjusts polarity of Alert pin output
// POL=0 means Alert is active LOW
// POL=1 means Alert is active HIGH
#define TMP102_CFG_POL 0x0400  // 0 = Alert active LOW *default*

// TM Thermostat Mode is Config bit 9
// Tells TMP102 to operate in Comparator (TM=0) or Interrupt mode (TM=1)
#define TMP102_CFG_TM 0x0200  // 0 = Comparator mode *default*

// Shutdown mode Config bit 8
// Set this bit to be low power (0.5 uA) between single conversions
// In SD mode the conversion rate bits are ignored, and you have
// to set the OS (one shot) bit to start a single conversion.
#define TMP102_CFG_SD 0x0100  // 0 = continuous conversion state *default*

// Conversion rate is Config bits 7,6
#define TMP102_CFG_RATE_8HZ 0x00C0  // 8 Hz conversion rate
#define TMP102_CFG_RATE_4HZ 0x0080  // 4 Hz conversion rate *default*
#define TMP102_CFG_RATE_1HZ 0x0040  // 1 Hz conversion rate
#define TMP102_CFG_RATE_QHZ 0x0000  // 0.25 Hz conversion rate (Q = 1/4)

// Alert is Config bit 5, read-only, it shows comparator status
// POL bit inverts the alarm polarity
#define TMP102_CFG_AL 0x0020  // I don't fully understand this bit. Page 7 in data sheet

// Extended mode is bit 4, set for 13-bit to read temps above 128C
#define TMP102_CFG_EM 0x0010  // 0 = 12-bit mode *default*

 /**
 * debug level
 * 0 = quiet, suppress everything
 * 3 = max, even more or less trivial message are emitted
 * 4 = emit debug info which checks very basic data conversion, etc
 */
 byte DEBUG = 1;

uint16_t rawtemp;
uint16_t faketemp;

uint16_t dtime;  // delay in loop

uint16_t configOptions;

boolean fake;    // if true use simulated temperature data

/**
Data for one instance of a TMP102 temp sensor.
Extended 13-bit mode is assumed (12-bit mode is only there for compatibility with older parts)
Error counters could be larger but then they waste more data in the typical case where they are zero.
Errors peg at max value for the data type: they don't roll over.
**/
struct data {
  uint8_t address;    // I2C address, only the low 7 bits matter
  uint16_t raw_temp;  // most recent
  float deg_c;        
  float deg_f;
  uint16_t t_high;
  uint16_t t_low;  
  uint16_t i2c_err_nak;  // total since startup
  uint16_t i2c_err_rd;   // total read fails - data not there when expected
};

static data tmp48;      // TMP102 at base address 0x48

// assign a MAC address for the ethernet controller.
// fill in your address here:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// assign an IP address for the controller:
IPAddress ip(192,168,1,20);
IPAddress gateway(192,168,1,1);	
IPAddress subnet(255, 255, 255, 0);


// Initialize the Ethernet server library
// with the IP address and port you want to use 
// (port 80 is default for HTTP):
EthernetServer server(80);

float temperature = 0.0;

/* ========== SETUP ========== */
void setup(void) 
{
  uint16_t raw16=0;  // place to put what we just read
  uint16_t wrt16=0;  // temp write variable
  int8_t stat = -1;
  
  delay (2000);      // give some time to open monitor window
  Serial.begin(115200);     // use max baud rate
  Serial.print("TMP102 Proof Code at 0x");
  Serial.println(TMP102_ADDR, HEX);
   
  int8_t flag = -1;  // I2C returns 0 if no error
  
  // Teensy++2 PE7/INT7/AIN1, pin 32 on module.
  // make it input so we can use it for ALERT
  pinMode(19, INPUT_PULLUP);  
  
  // join I2C bus as master
  Wire.begin();

  // start with default config
  Serial.print ("SetCFG=");
  Serial.print ((uint16_t)TMP102_CFG_DEFAULT_WR, HEX);
  Serial.print (" ");
  stat = writeRegister(TMP102_REG_CONF_REG, TMP102_CFG_DEFAULT_WR);
  if ( 0!= stat) Serial.print (" writeReg error! ");
  stat = readRegister (&raw16);
  if ( 2!= stat) Serial.print (" readReg error! ");
  Serial.print("CFG:");
  Serial.print(raw16, HEX);
  Serial.print(" ");    
  
//  // configure the TMP102
//  // POR default should be 0x60A0 (datasheet Table 7)
//  stat = writePointer(TMP102_REG_CONF_REG);
//  stat = readRegister (&raw16);
//  Serial.print("Config was:");
//  Serial.print(raw16, HEX);
//  Serial.println(" ");
  
  configOptions = 0x0;  // 
  configOptions |= TMP102_CFG_EM;  // set Extended Mode
  configOptions |= TMP102_CFG_RATE_1HZ;  // 1Hz conversion
  configOptions |= TMP102_CFG_SD;        // sleep between conversions
  
  Serial.print ("SetCFG=");
  Serial.print (configOptions, HEX);
  Serial.print (" ");
  stat = writeRegister(TMP102_REG_CONF_REG, configOptions);
  if ( 0!= stat) Serial.print (" writeReg error! ");
  stat = readRegister (&raw16);
  if ( 2!= stat) Serial.print (" readReg error! ");
  Serial.print("CFGnow:");
  Serial.print(raw16, HEX);
  Serial.print(" ");  
  
  delay(30);    // 26 msec for conversion
  stat = writePointer(TMP102_REG_CONF_REG);
  stat = readRegister (&raw16);
  Serial.print("CFG:");
  Serial.print(raw16, HEX);
  Serial.print(" ");
  
  stat = writePointer(TMP102_REG_TLOW_REG);
  stat = readRegister (&raw16);
  Serial.print("Tlo:");
  Serial.print(raw16, HEX);
  Serial.print(" ");
  
  stat = writePointer(TMP102_REG_THIGH_REG);
  stat = readRegister (&raw16);
  Serial.print("Thi:");
  Serial.print(raw16, HEX);
  Serial.print(" ");
  
  // leave the pointer set to read temperature
  stat = writePointer(TMP102_REG_TEMP_REG);
  
  // fake temp  
  fake = true;
  dtime = 0;  // fast loop
  faketemp = (uint16_t)0x4B00;  // max 13-bit value in raw 16-bit format
  
  // use real temperature data
  fake = false;      // switch to real data after full cycle of simulated
  dtime = 1000;      // msec between samples, 1000 = 1 sec, 60,000 = 1 minute
  Serial.print(" Interval is ");
  Serial.print(dtime/1000);
  Serial.print(" sec, ");

  
  Serial.println("Config Ethernet");
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  
  Serial.println("Setup Complete!");
  Serial.println(" "); 
  
  if (1 == DEBUG)
  {
    Serial.println("sec deg C");
  }
}


uint16_t good=0;
uint16_t bad=0;
uint16_t raw16;

/* ========== LOOP ========== */
void loop(void) 
{
  int16_t temp0;
  int8_t stat=-1;  // status flag
  float temp;
  
  Serial.print("@");
  Serial.print(millis()/1000);
  Serial.print(" ");
  
  if (!fake)  // get real temperature data from sensor
  {
  //  Serial.print("good:");
  //  Serial.print(good);
  //  Serial.print(" ");
    if (bad > 0) 
    {
      Serial.print(" bad:");
      Serial.print(bad);
      Serial.print(" ");
    }
  
    if (DEBUG >=3)
    {
      Serial.print("ALpin:");
      Serial.print(digitalRead(19));
      Serial.print(" ");
    }
  
    stat = writePointer(TMP102_REG_CONF_REG);
    if (DEBUG >=3)
    {
      stat = readRegister (&raw16);
      Serial.print("CFG:");
      Serial.print(raw16, HEX);
      Serial.print(" ");
    }
  
    configOptions |= TMP102_CFG_OS;        // start single conversion
    stat = writeRegister (TMP102_REG_CONF_REG, configOptions);
  
    if (DEBUG >=2)
    {  
      stat = readRegister (&raw16);
      Serial.print("CFG:");
      Serial.print(raw16, HEX);
      Serial.print(" ");
    }
    // pointer set to read temperature
    stat = writePointer(TMP102_REG_TEMP_REG); 
    // read two bytes of temperature
    stat = readRegister (&rawtemp);
    if (2==stat) good++;
    else bad++;
  } 
  else rawtemp = faketemp;    // fresh simulated value
  
// 12-bit temp mode
//  rawtemp = rawtemp>>4;      // ignore neg for now
//  Serial.print ("RawTemp:");
//  Serial.print(rawtemp, HEX);
//  Serial.print (" ");
//  
//  float temp;
//  temp = 0.0625 * (rawtemp);    
//  Serial.print (temp);
//  Serial.println ("C");

  if (DEBUG >= 2)
  {
    Serial.print ("Raw16:0x");
    if (0==(rawtemp & 0xF000)) Serial.print("0");
    Serial.print(rawtemp, HEX);
    Serial.print (" ");
  }
    
  // 13-bit temp mode  
//  if ((rawtemp & 0x8000) == 0x8000) // temp is neg, we must take 2's complement
//  {
//    //Serial.print (" (neg temp) ");
//    rawtemp = ~rawtemp;    // complement each bit
//    rawtemp++;             // add one
//    rawtemp = rawtemp>>3;      // we want ms 13 bits  
//    temp = -0.0625 * (rawtemp);  // make result negative
//  }
//  else
//  {
//    rawtemp = rawtemp>>3;      // we want ms 13 bits right-justified  
//    temp = 0.0625 * (rawtemp);  // make result negative
//  }

  temp = raw13ToC(rawtemp);
  
  temperature = temp;  // for Ethernet client
  
  if (DEBUG >= 2)
  {
    Serial.print("ms13:0x");
    Serial.print(rawtemp, HEX);
    Serial.print (" ");
  }
  
  if (fake) Serial.print (temp,4);  // no rounding of simulated data
  else Serial.print (temp,2);       // 2 dec pts good enough for real data 0.0625 deg C per count
  
  Serial.print (" C ");
  
  if (DEBUG >= 2)
  {
    Serial.print(rawtemp, DEC);
    Serial.print ("D ");
  }
  
  // test with all values of rawtemp
  if (fake) 
  {
    // faketemp raw change of 0x280 is 5 deg C
    faketemp -= 0x280;  
    // 0xE480 is min legal value
    if (temp <= -55) 
    {
      faketemp = 0x4B00;  // if min then reset to max    
      
      fake = false;      // switch to real data after full cycle of simulated
      dtime = 2000;
      Serial.println();
      Serial.print ("Changing to real data");
    }
  }
  
  Serial.println();
  
  // listen for incoming Ethernet connections:
  listenForEthernetClients();
  
  delay(dtime);
}

/**

taken from Tom Igoes BarometricPressureWebServer example

With loop delay of 2 sec, this server sees a single FF request but two Chrome requests.
With " " 1 sec FF is seen as two requests 
**/
void listenForEthernetClients() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("Got a client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          Serial.println("Sending response");
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println();
          // print the current readings, in HTML format:
          client.print ("@" + String(millis()/1000));
          client.print(" Temp= ");
          client.print(temperature);  // float
          client.print(" degrees C");
          client.println("<br />");
//          client.print("Pressure: " + String(pressure));
//          client.print(" Pa");
//          client.println("<br />");  
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    Serial.println("Closing client");
    client.stop();
  }
} 

/**
Read the most current temperature already converted and present in the TMP102 temperature registers

In continuous mode, this could be one sample interval old
In one shot mode this data is from the last-requested one shot conversion
**/
uint8_t readTempDegC (float *tempC) 
{
}

/**
Convert raw 13-bit temperature to float deg C

handles neg and positive specific to TMP102 extendend mode 13-bit temperature data

TODO instead pass a pointer to the float variable? and return error if value out of bounds
**/
float raw13ToC (uint16_t raw13)
{
  float degC;
  
  // 13-bit temp mode  
  if ((raw13 & 0x8000) == 0x8000) // temp is neg, we must take 2's complement
  {
    //Serial.print (" (neg temp) ");
    raw13 = ~raw13;    // complement each bit
    raw13++;             // add one
    raw13 = raw13>>3;      // we want ms 13 bits  
    degC = -0.0625 * (raw13);  // make result negative
  }
  else
  {
    raw13 = raw13>>3;      // we want ms 13 bits right-justified  
    degC = 0.0625 * (raw13);  // make result negative
  }
  return degC;  
}

/**
Convert deg C float to a raw 13-bit temp value in TMP102 format.
This is needed for Th and Tl registers as thermostat setpoint values

return 0 if OK, error codes if float is outside range of TMP102
**/
int8_t degCToRaw13 (uint16_t *raw13, float *tempC)
{
}



/**
Trigger a one-shot temperature conversion, wait for the new value, about 26 msec, and update 
the variable passed.

If the TMP102 is in continuous conversion mode, this places the part in One Shot mode, 
triggers the conversion, waits for the result, updates the variable, and leaves the TMP102 in one shot mode.

returns 0 if no error
**/
uint8_t getOneShotDegC (float *tempC)
{
}


/**
Set the TMP102 mode to one-shot, with low power sleep in between

mode: set to One Shot if true. 
If false, sets to continuous sampling mode at whatever sample rate was last set.

returns: 0 if successful
**/
int8_t setModeOneShot (boolean mode)
{
}

/**
Set TMP102 mode to continuous sampling at the rate given.

rate: must be one of the manifest constants such as TMP102_CFG_RATE_1HZ
if rate is not one of the four supported, it is set to the default 4 Hz

returns: 0 if successful
**/
int8_t setModeContinuous (int8_t rate)
{
}


/**
Write to a TMP102 register
Start with slave address, as in any I2C transaction.
Next byte must be the Pointer Register value, in 2 lsbs
If all you want to do is set the Pointer Register for subsequent read(s)
then this 2-byte write cycle is complete.

Data bytes in the write are optional but must always follow the Pointer Register write byte.
The last value written to the Pointer Register persists until changed.

**/

int8_t writePointer (uint8_t pointer)
{
 
  if (DEBUG >=4)
  {
    Serial.print ("adr=0x");
    Serial.print(TMP102_ADDR, HEX);
    Serial.print(" ");
    Serial.print("ptr=0x");
    Serial.print(pointer, HEX);
    Serial.print(" ");
  }

  uint8_t b;  // temp variable
  uint16_t ui;  // temp variable
  int8_t flag=-1;  // signed flag, init to error result so we know if it has changed
  byte written;  // number of bytes written

  Wire.beginTransmission(TMP102_ADDR);  // base address
  written = Wire.write(pointer);        // pointer in 2 lsb

  flag = Wire.endTransmission();    // flag is zero if no error
  
  
  if (DEBUG >=4)
  {
    Serial.print ("wrote:");
    Serial.print (written);
    Serial.print (" ");
  
    if (0 == flag)
    {
      Serial.print("TMP102 WrtP OK=");
      Serial.print((int)flag);
    }  
    else
    {
      Serial.print("Error=");
      Serial.print((int)flag);
    }
    Serial.print (" ");
  }
  
  return (flag);  // zero if no error
}

/**

Param pointer is the TMP102 register into which to write the data
data is the 16 bits to write.
returns 0 if no error, positive values for NAK errors
**/
int8_t writeRegister (uint8_t pointer, uint16_t data)
{
  if (DEBUG >=4)
  {
    Serial.print("ptr=0x");
    Serial.print(pointer, HEX);
    Serial.print(" dat=0x");
    Serial.print(data, HEX);
    Serial.print(" ");
  }

  uint8_t ub;  // temp variable
  uint16_t ui;  // temp variable
  int8_t flag=-1;  // signed flag, init to error result so we know if it has changed
  byte written;  // number of bytes written

  Wire.beginTransmission(TMP102_ADDR);  // base address
  written = Wire.write(pointer);        // pointer in 2 lsb
  
  ub = data >> 8;    // put MSB of data into lower byte
  written += Wire.write(ub);    // write MSB of data

  if (DEBUG >=4)
  {
    Serial.print("MSB=0x");
    Serial.print(ub, HEX);
    Serial.print(" ");
  }
  
  ub = data & 0x00FF;  // mask off upper byte of data ??? Is this necessary?
  written += Wire.write(ub);      // write LSB of data
  
  if (DEBUG >=4)
  {
    Serial.print("LSB=0x");
    Serial.print(ub, HEX);
    Serial.print(" ");
  }
    
  
  flag = Wire.endTransmission();    // flag is zero if no error
  if (0 != flag)
  {
    Serial.print("Error TMP102 wrtReg flag=");
    Serial.println((int)flag);
  }
  
  if (DEBUG >=4)
  {
    Serial.print ("wrote:");
    Serial.print (written);
    Serial.print (" ");
    
    if (0 == flag)
    {
      Serial.print("TMP102 WrtR OK=");
      Serial.print((int)flag);
    }  
    else
    {
      Serial.print("Error=");
      Serial.print((int)flag);
    }
    Serial.print (" ");
  }
  
  return (flag);  // zero if no error
}

/**
  Read the 16-bit register addressed by the current pointer value, store the data at the location passed
  
  return 0 if no error, positive bytes read otherwise.
*/
int8_t readRegister (uint16_t *data)
{
  uint8_t ub1;  // temp variable
  uint8_t ub2;
  uint16_t ui=0;  // temp variable
  int8_t flag=-1;  // signed flag, init to error result so we know if it has changed
  int avail1=0;
  int avail2=0;
//  
  // 
  // Wire.beginTransmission(TMP102_ADDR);  // base address
  Wire.requestFrom(TMP102_ADDR, (uint8_t)2, (uint8_t)true);
  avail1 = Wire.available();
  flag = avail1;
  ub1 = Wire.read();   // read MSB
  ub2 = Wire.read();  // LSB
//  flag = Wire.endTransmission();    // flag is zero if no error
  
  avail2 = Wire.available();
  if (DEBUG >=4)
  {
    Serial.print("avail-1/2:");
    Serial.print(avail1);
    Serial.print("/");
    Serial.print(avail2);
    Serial.print(" ");
    
    
    Serial.print("ms=0x");
    Serial.print(ub1, HEX);
    Serial.print(" ");
  }
  
  ui = ub1<<8;      // save read byte in upper byte of 16-bit temp
  
  if (DEBUG >=4)
  {
    Serial.print("ls=0x");
    Serial.print(ub2, HEX);
    Serial.print(" ");
  }
  
  ui |= ub2;        // OR in the byte read into the low byte of 16-bit temp
  if (DEBUG >=4)
  {
    Serial.print("dat=0x");
    Serial.print(ui, HEX);
    Serial.print(" ");
  }

//  // requuestFrom does not return an error
//  if (0 != flag)
//  {
//    Serial.print("Error-rdReg=");
//    Serial.println((int)flag);
//  }
//  
  *data = ui;     // copy read value into data location
  
  return (flag);  // zero if no error
}
