

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include "privateConfig.h"
#include "DataToMaker.h"

// Configurable globals and defines.  Edit to your needs. -------------------

// Debugging flags
#define _DEBUG  1   // try to push data to serial monitor (save memory by setting to 0)
//#define NOTWEET 1   // don't start networking or send tweet
#define WASHER  0   // washing machine connected, 0==no
#define DRYER   1   // dryer connected, 0==no
#define WEBMSG  0   // pull tweet strings from the web, 0==no

// Dryer settings
// Using analog 0 for the Current Transformer for Dryer
#define CTPIN_D              0
#define WAITONTIME         15000        //In milliseconds, time before setting appliace as "running" (used for Washer & Dryer)
#define WAITOFFTIME        30000        //In milliseconds, time to wait to make sure dryer is really off (avoid false positives)
#define CURRENT_THRESHOLD    20.0    //In AMPS
double CURRENT_OFFSET  =   0.0;  // Dryer offset (made a double so it can auto set)
#define NUMADCSAMPLES      4000     //Number of samples to get before calculating RMS

// Washer settings
// Using analog 9 for the Current Transformer for Washer
#define CTPIN_W            9
#define CURRENT_THRESHOLD_W      10.0    // In AMPS
double CURRENT_OFFSET_W   =     0.0;  // Washer offset in Amps
#define WAITOFFTIME_W        99000    // In MS 

//STATUS LEDS
#define DRYLED   19   //Dryer status
#define WASHLED  20   //Washer status
#define REDLED   21   //General status/error

// TODO: see if this can be pulled out
// Buffer for reading messages?
char strBuf[165];
// PString str(strBuf, sizeof(strBuf));
#define TEMP_MSG_BUFFER_SIZE 100
char tempMsgBuffer[TEMP_MSG_BUFFER_SIZE];

boolean printCurrent = true;

//Constants from DataToMaker.h
const char* myKey = MakerIFTTT_Key; // your maker key here
const char* ssid = WLAN_SSID; // your router ssid here
const char* password = WLAN_PASS; // your router password here

// declare new maker event with the name defined in privateConfig.h
DataToMaker event(myKey, MakerIFTTT_Event);

// Random strings db settings
// char stringServer[] = "YOUR_STRING_SERVER";    // Server for strings

//If using 3.3V Arduino use this
#define ADC_OFFSET 1.65                    //Resistor divider is used to bias current transformer 3.3V/2 = 1.65V
#define ADCVOLTSPERDIVISION 0.003222656    //3.3V/1024 units

//If using 5V Arduino use this
//#define ADC_OFFSET 2.5                   //Resistor divider is used to bias current transformer 5V/2 = 2.5V
//#define ADCVOLTSPERDIVISION 0.0047363    //5V / 1024 units

//These are the various states during monitoring. WAITON and WAITOFF both have delays
//that can be set with the WAITONTIME and WAITOFFTIME parameters
enum STATE {
  WAITON,    //Washer/Dryer is off, waiting for it to turn on
  RUNNING,   //Exceeded current threshold, waiting for washer/dryer to turn off
  WAITOFF    //Washer/dryer is off, sends notifications, and returns to WAITON
};

//This keeps up with the current state of the FSM, default startup setting is the first state of waiting for the dryer to turn on.
STATE dryerState = WAITON;
STATE washerState = WAITON;

// CC3000 interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3 // MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT  5 // These can be
#define ADAFRUIT_CC3000_CS   10 // any two pins
// Hardware SPI required for remaining pins.
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(
                           ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                           SPI_CLOCK_DIVIDER);

// WiFi access point credentials
//#define WLAN_SSID     "YourSSID"  // 32 characters max
//#define WLAN_PASS     "YourWLANPass" //
#define WLAN_SECURITY WLAN_SEC_WPA2 // WLAN_SEC_UNSEC/WLAN_SEC_WEP/WLAN_SEC_WPA/WLAN_SEC_WPA2



const unsigned long
dhcpTimeout     = 60L * 1000L, // Max time to wait for address from DHCP
connectTimeout  = 15L * 1000L, // Max time to wait for server connection
responseTimeout = 15L * 1000L; // Max time to wait for data from server
unsigned long
currentTime = 0L;
Adafruit_CC3000_Client client;        // For WiFi connections





// What page to grab!
// #define WEBSITE      "maker.ifttt.com"
// #define WEBPAGE      "/your/trigger/url/here/" //If you're using ifttt you add your URL here

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
// received before closing the connection.  If you know the server
// you're accessing is quick to respond, you can reduce this value.


// --------------------------------------------------------------------------


// Prototypes
int timedRead(void);
unsigned long getTime(void);
void urlEncode(
  Print      &p,       // EthernetClient, Sha1, etc.
  const char *src,     // String to be encoded
  boolean     progmem, // If true, string is in PROGMEM (else RAM)
  boolean     x2) ;

uint16_t checkFirmwareVersion(void);
void displayMACAddress(void);
bool displayConnectionDetails(void);
void listSSIDResults(void);
int sendIftttMakerEvent(void);
double takeCurrentMeasurement(int);
void monitorWasher(void);
void monitorDryer(void);
String GetAMessage(char);
String strFromMySQL(String);
int hang(const);
char *append_str(char , char);
char *append_ul(char , unsigned long );


/*=====================================================
   Setup
  =====================================================*/
void setup(void) {

  uint32_t ip = 0L, t;

  Serial.begin(115200);

  Serial.print(F("Hello! Initializing CC3000..."));
  if (!cc3000.begin()) hang(F("failed. Check your wiring?"));

  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong firmware version!"));
    for (;;);
  }

  displayMACAddress();

  /* Optional: Get the SSID list (not available in 'tiny' mode) */
#ifndef CC3000_TINY_DRIVER
  listSSIDResults();
#endif

  /* Delete any old connection data on the module */
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while (1);
  }

  /* Optional: Set a static IP address instead of using DHCP.
     Note that the setStaticIPAddress function will save its state
     in the CC3000's internal non-volatile memory and the details
     will be used the next time the CC3000 connects to a network.
     This means you only need to call the function once and the
     CC3000 will remember the connection details.  To switch back
     to using DHCP, call the setDHCP() function (again only needs
     to be called once).
  */
  uint32_t ipAddress = cc3000.IP2U32(192, 168, 1, 19);
  uint32_t netMask = cc3000.IP2U32(255, 255, 255, 0);
  uint32_t defaultGateway = cc3000.IP2U32(192, 168, 1, 1);
  uint32_t dns = cc3000.IP2U32(8, 8, 4, 4);
  if (!cc3000.setStaticIPAddress(ipAddress, netMask, defaultGateway, dns)) {
    Serial.println(F("Failed to set static IP!"));
    while (1);
  }



  /* Display the IP address DNS, Gateway, etc. */
  //  while (! displayConnectionDetails()) {
  //    delay(1000);
  //  }
  //Send a message (used for testing the event without a connected appliance)
  int tempInt = sendIftttMakerEvent();

  // Get initial time from time server (make a few tries if needed)
  // for (uint8_t i = 0; (i < 5) && !(currentTime = getTime()); delay(5000L), i++);







  delay(1000);

  // Configure LEDs
  pinMode(DRYLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(WASHLED, OUTPUT);

  // Turn on the red LED during setup
  digitalWrite(REDLED, HIGH);

  if (_DEBUG) {
    Serial.println("Washer Dryer Tweeter");
    Serial.println("==========================");

  }

  //Initialize random seed using unused analog 1
  randomSeed(analogRead(1));

  // Instead of having the offset hard coded, I've made it zero itself
  // on boot. This removed variance in the offset that seemed to come
  // from different power sources for the Arduino.
  //
  // Pull current on startup to zero the measurements
  double theMeasurement = 0;
  if (WASHER == 1) {
    // Pull washer current measurement
    for (int i = 0; i < 3; i++) {
      theMeasurement += takeCurrentMeasurement(CTPIN_W);
    }
    //
    CURRENT_OFFSET_W = 0 - (theMeasurement / 3);

    // display the calculation
    if (_DEBUG) {
      String tempString = "Washer offset: ";
      tempString += CURRENT_OFFSET_W;
      Serial.println (tempString);
      delay(5000);
    }
  }
  theMeasurement = 0; // zero the measurement variable

  if (DRYER == 1) {
    // Pull dryer current measurement
    for (int i = 0; i < 3; i++) {
      theMeasurement += takeCurrentMeasurement(CTPIN_D);
    }

    CURRENT_OFFSET = 0 - (theMeasurement / 3);

    // display the calculation
    if (_DEBUG) {
      String tempString = "Dryer offset: ";
      tempString += CURRENT_OFFSET;
      Serial.println (tempString);
      delay(5000);
    }
  }
  // Setup successful, turn out red LED, turn on the wash and dry LEDs
  digitalWrite(REDLED, LOW);
  digitalWrite(WASHLED, HIGH);
  digitalWrite(DRYLED, HIGH);

}

uint8_t countdown = 23; // Countdown to next time server query (once per ~24hr)
uint32_t ip;


/* -------------------------------------------------------



  ---------------------------------------------------------
*/

void loop() {

  unsigned long t = millis();


  if (WASHER)
  {
    monitorWasher();
  }

  if (DRYER)
  {
    monitorDryer();
  }




  //  int theResult = sendIftttMakerEvent();



  //  // Delay remainder of 1 hour (takes tweeting time into account)
  //  Serial.println("Waiting ~1 hour...");
  //  delay((60L * 60L * 1000L) - (millis() - t));
  //  currentTime += 60L * 60L * 1000L;

  //  if (!countdown) {       // 24 hours elapsed?
  //    if ((t = getTime())) { // Synchronize time if server contacted
  //      currentTime = t;
  //      countdown   = 23;   // and reset counter
  //    }
  //  } else countdown--;


  // Returns true on success, false on error

  //unsigned long startTime = millis();


}


/*--------------------------------------
   Helper functions



   ------------------------------------- */

uint16_t checkFirmwareVersion(void) {
  uint8_t  major, minor;
  uint16_t version = 0;

#ifndef CC3000_TINY_DRIVER
  if (!cc3000.getFirmwareVersion(&major, &minor)) {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
  } else {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = ((uint16_t)major << 8) | minor;
  }
#endif
  return version;
}


// For URL-encoding functions below
static const char PROGMEM hexChar[] = "0123456789ABCDEF";

// URL-encoding output function for Print class.
// Input from RAM or PROGMEM (flash).  Double-encoding is a weird special
// case for Oauth (encoded strings get encoded a second time).
void urlEncode(
  Print      & p,      // EthernetClient, Sha1, etc.
  const char *src,     // String to be encoded
  boolean     progmem, // If true, string is in PROGMEM (else RAM)
  boolean     x2)      // If true, "double encode" parenthesis
{
  uint8_t c;

  while ((c = (progmem ? pgm_read_byte(src) : *src))) {
    if (((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) ||
        ((c >= '0') && (c <= '9')) || strchr_P(PSTR("-_.~"), c)) {
      p.write(c);
    } else {
      if (x2) p.print("%25");
      else   p.write('%');
      p.write(pgm_read_byte(&hexChar[c >> 4]));
      p.write(pgm_read_byte(&hexChar[c & 15]));
    }
    src++;
  }
}

// Returns would-be length of encoded string, without actually encoding
int encodedLength(char *src) {
  uint8_t c;
  int     len = 0;

  while ((c = *src++)) {
    len += (((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) ||
            ((c >= '0') && (c <= '9')) || strchr_P(PSTR("-_.~"), c)) ? 1 : 3;
  }

  return len;
}

// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getTime(void) {

  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;

  Serial.print(F("Locating time server..."));

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if (cc3000.getHostByName("pool.ntp.org", &ip)) {
    static const char PROGMEM
    timeReqA[] = { 227,  0,  6, 236 },
                 timeReqB[] = {  49, 78, 49,  52 };

    Serial.println(F("found\r\nConnecting to time server..."));
    startTime = millis();
    do {
      client = cc3000.connectUDP(ip, 123);
    } while ((!client.connected()) &&
             ((millis() - startTime) < connectTimeout));

    if (client.connected()) {
      Serial.print(F("connected!\r\nIssuing request..."));

      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      client.write(buf, sizeof(buf));

      Serial.print(F("OK\r\nAwaiting response..."));
      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while ((!client.available()) &&
             ((millis() - startTime) < responseTimeout));
      if (client.available()) {
        client.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
             (unsigned long)buf[43]) - 2208988800UL;
        Serial.println(F("success!"));
      }
      client.close();
    }
  }
  if (!t) Serial.println(F("error"));
  return t;
}
/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];

  if (!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}


/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if (!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

/**************************************************************************/
/*!
    @brief  Begins an SSID scan and prints out all the visible networks
*/
/**************************************************************************/

void listSSIDResults(void)
{
  uint32_t index;
  uint8_t valid, rssi, sec;
  char ssidname[33];

  if (!cc3000.startSSIDscan(&index)) {
    Serial.println(F("SSID scan failed!"));
    return;
  }

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);

    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}

int sendIftttMakerEvent(void) {
  // string to send to Maker Event
  // move this to the function to receive from calling function (or as global from the get string function)
  String myValue1 = "Dryer is done";
  String myRequestBody = "{ \"value1\" : \"" + myValue1 + "\"}";


  Serial.println("Entered maker event function");
  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);

  /* NOTE: Secure connections are not available in 'Tiny' mode!
     By default connectToAP will retry indefinitely, however you can pass an
     optional maximum number of retries (greater than zero) as the fourth parameter.

     ALSO NOTE: By default connectToAP will retry forever until it can connect to
     the access point.  This means if the access point doesn't exist the call
     will _never_ return!  You can however put in an optional maximum retry count
     by passing a 4th parameter to the connectToAP function below.  This should
     be a number of retries to make before giving up, for example 5 would retry
     5 times and then fail if a connection couldn't be made.
  */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while (1);
  }

  Serial.println(F("Connected!"));


  // Try looking up the website's IP address
  Serial.print(WEBSITE);
  Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println("Couldn't resolve!");
    }
    delay(500);
  }
  cc3000.printIPdotsRev(ip);
  Serial.println("");

  // Optional: Do a ping test on the website

  Serial.print(F("\n\rPinging ")); cc3000.printIPdotsRev(ip); Serial.print("...");
  int replies = cc3000.ping(ip, 5);
  Serial.print(replies); Serial.println(F(" replies"));

  Serial.println("before connection appempt");

  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
  if (www.connected()) {
    Serial.println("connected, making request");

    // Testing to find the right combo
    // Serial.println("Here's the request I'm going to make");
    // Serial.println((char*)post_rqst);

    //    www.fastrprint(postRequestString.c_str());

    //www.fastrprint((char*)post_rqst);

    Serial.println("Sending...");
    www.fastrprint(F("POST "));
    www.fastrprint(WEBPAGE);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: "));
    www.fastrprint(WEBSITE);
    www.fastrprint(F("Content-Type: application/json\r\n"));
    www.fastrprint(F("Content-Length: 58\r\n"));
    www.fastrprint(F("Connection: close\r\n\r\n"));
    www.fastrprint(F("{\"value1\":\"930\",\"value2\":\"950\",\"value3\":\"hello, world!\"}\r\n"));
    //www.fastrprint(myValue1.c_str());
    //    www.println();
  } else {
    Serial.println("Connection failed");
    return;
  }
  Serial.println("after connection appempt");

  Serial.println(F("-------------------------------------"));

  /* Read data until either the connection is closed, or the idle timeout is reached. */
  unsigned long lastRead = millis();
  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
    while (www.available()) {
      char c = www.read();
      Serial.print(c);
      lastRead = millis();
    }
  }
  www.close();
  Serial.println(F("-------------------------------------"));

  /* You need to make sure to clean up after yourself or the CC3000 can freak out */
  /* the next time your try to connect ... */
  Serial.println(F("\n\nDisconnecting"));
  cc3000.disconnect();
  return 1;
}


/*--------------------------------------------------------------



  -------------------------------------------------------------*/
double takeCurrentMeasurement(int channel)
{
  static double VDOffset = 1.65;

  //Equation of the line calibration values
  //double factorA = 15.2; //factorA = CT reduction factor / rsens
  //double factorA = 25.7488;
  double factorA = 33.0113;
  //double factorA = 29.03225;
  double Ioffset =  -0.04;
  double theCurrentOffset = 0.0;

  if (channel == CTPIN_D) { // check if washer or dryer
    theCurrentOffset = CURRENT_OFFSET; // use dryer offset
  }
  else {
    theCurrentOffset = CURRENT_OFFSET_W; // use washer offset
  }

  //Used for calculating real, apparent power, Irms and Vrms.
  double sumI = 0.0;

  int sum1i = 0;
  double sumVadc = 0.0;

  double Vadc, Vsens, Isens, Imains, sqI, Irms;

  for (int i = 0; i < NUMADCSAMPLES; i++)
  {
    int val = 0;
    val = analogRead(channel);
    Vadc = val * ADCVOLTSPERDIVISION;
    Vsens = Vadc - VDOffset;
    //Current transformer scale to find Imains
    Imains = Vsens;
    //Calculates Voltage divider offset.
    sum1i++;
    sumVadc = sumVadc + Vadc;
    if (sum1i >= 1000) {
      VDOffset = sumVadc / sum1i;
      sum1i = 0;
      sumVadc = 0.0;
    }

    //Root-mean-square method current
    //1) square current values
    sqI = Imains * Imains;
    //2) sum
    sumI = sumI + sqI;
  }
  Irms = factorA * sqrt(sumI / NUMADCSAMPLES) + theCurrentOffset;
  sumI = 0.0;
  return Irms;
}

void monitorWasher()
{
  // TODO: what's the loop count for?
  static unsigned long wLoopCount = 0;
  static unsigned long wStartTime;
  static unsigned long wRunStartTime;
  static boolean wWaitCheck;
  double wCurrent;

  wLoopCount++;
  switch (washerState)
  {
    case WAITON:
      wCurrent = takeCurrentMeasurement(CTPIN_W);
      if (printCurrent)
      {
        Serial.print("Washer Current: ");
        Serial.print(wCurrent);
        Serial.print(" Threshold: ");
        Serial.println(CURRENT_THRESHOLD_W);
      }
      if (wCurrent > CURRENT_THRESHOLD_W)
      {
        if (!wWaitCheck)
        {
          wStartTime = millis();
          wWaitCheck = true;
          Serial.print("Started watching washer in WAITON. wStartTime: ");
          Serial.println(wStartTime);
        }
        else
        {
          if ( (millis() - wStartTime  ) >= WAITONTIME)
          {
            Serial.println("Changing washer state to RUNNING");
            washerState = RUNNING;
            wWaitCheck = false;
            wRunStartTime = millis();
          }
        }
      }
      else
      {
        if (wWaitCheck)
        {
          Serial.println("False alarm, not enough time elapsed to advance washer to running");
        }
        wWaitCheck = false;
      }
      break;

    case RUNNING:
      wCurrent = takeCurrentMeasurement(CTPIN_W);
      digitalWrite(WASHLED, !digitalRead(WASHLED));
      if (printCurrent) // TODO: I'm repeating this block over and over, should make it into a function
      {
        Serial.print("Washer Current: ");
        Serial.print(wCurrent);
        Serial.print(" Threshold: ");
        Serial.println(CURRENT_THRESHOLD_W);
      }
      if (wCurrent < CURRENT_THRESHOLD_W)
      {
        Serial.println("Changing washer state to WAITOFF");
        washerState = WAITOFF;
      }
      break;

    case WAITOFF:
      if (!wWaitCheck)
      {
        wStartTime = millis();
        wWaitCheck = true;
        if (printCurrent)
        {
          Serial.print("Washer Current: ");
          Serial.print(wCurrent);
          Serial.print(" Threshold: ");
          Serial.println(CURRENT_THRESHOLD_W);
          Serial.print("Entered washwer WAITOFF. wStartTime: ");
          Serial.println(wStartTime);
        }
        // setting the RED LED to solid on while waiting to send off signal.
        // TODO: Check for conflict with dryer use of red led
        digitalWrite(REDLED, HIGH);
      }

      wCurrent = takeCurrentMeasurement(CTPIN_W);

      if (wCurrent > CURRENT_THRESHOLD_W)
      {
        if (printCurrent)
        {
          Serial.print("Washer Current: ");
          Serial.print(wCurrent);
          Serial.print(" Threshold: ");
          Serial.println(CURRENT_THRESHOLD_W);
          Serial.println("False Alarm, washer not off long enough, back to RUNNING we go!");
        }
        wWaitCheck = false;
        // TODO: need to make it so the false alarm light goes off when done,
        //       but keep false alarm on one from turning off light for other.
        digitalWrite(REDLED, LOW);

        washerState = RUNNING;
      }
      else
      {
        if ( (millis() - wStartTime) >= WAITOFFTIME_W)
        {
          Serial.println("Washer cycle complete");
          String theMessage = GetAMessage('w'); // call GetAMessage with 'w' for washer message

          // Cycle time
          // I don't really care about this info, commenting out
          // Uncomment if you want to know how long the cycle ran
          //        str.print( F(" Wash Time: " ));
          //        double totalWashRun = (double) (millis() - wRunStartTime) / 1000 / 3600;
          //        str.print(totalWashRun);
          //        if(totalWashRun < 0)
          //          str.print("hr");
          //        else
          //          str.print("hrs");


          Serial.println("Posting to Maker Channel");
          Serial.println("Posting the following: ");
          Serial.println(theMessage);

          // TODO: insert maker channel call

          Serial.println("Resetting washer state back to WAITON");
          washerState = WAITON;
          wWaitCheck = false;
          // TODO: setting RED to low will turn off even if
          //       washer is in wait off. Need logic to
          //       account for the potential conflict.
          digitalWrite(REDLED, LOW);
          digitalWrite(WASHLED, HIGH);
        }
      }
  }

} // monitorWasher

void monitorDryer()
{
  static unsigned long loopCount = 0;
  static unsigned long startTime;
  static unsigned long runStartTime;
  static boolean waitCheck;
  double current;

  loopCount++;
  switch (dryerState)
  {
    case WAITON:
      current = takeCurrentMeasurement(CTPIN_D);
      if (printCurrent)
      {
        String tempString = "Dryer Current: ";
        tempString = tempString + current  + " Dryer Threshold: " + CURRENT_THRESHOLD;
        Serial.println(tempString);
      }
      if (current > CURRENT_THRESHOLD)
      {
        if (!waitCheck)
        {
          startTime = millis();
          waitCheck = true;
          String tempString = "Started watching dryer in WAITON. startTime: ";
          tempString += startTime;
          Serial.println(tempString);
        }
        else
        {
          if ( (millis() - startTime  ) >= WAITONTIME)
          {
            Serial.println("Changing dryer state to RUNNING");
            dryerState = RUNNING;
            waitCheck = false;
            runStartTime = millis();
          }
        }
      }
      else
      {
        if (waitCheck)
        {
          Serial.println("False alarm, not enough time elapsed to advance to running");
        }
        waitCheck = false;
      }
      break;

    case RUNNING:
      current = takeCurrentMeasurement(CTPIN_D);
      digitalWrite(DRYLED, !digitalRead(DRYLED));
      if (printCurrent)
      {
        String tempString = "Dryer Current: ";
        tempString = tempString + current + " Threshold: " + CURRENT_THRESHOLD;
        Serial.println(tempString);
      }
      if (current < CURRENT_THRESHOLD)
      {
        Serial.println("Changing dryer state to WAITOFF");
        dryerState = WAITOFF;
      }
      break;

    case WAITOFF:
      if (!waitCheck)
      {
        startTime = millis();
        waitCheck = true;
        String tempString = "Entered WAITOFF. startTime: ";
        tempString += startTime;
        Serial.println(tempString);

        // setting the RED LED to solid on while waiting to send off signal.
        digitalWrite(REDLED, HIGH);
      }

      current = takeCurrentMeasurement(CTPIN_D);

      if (current > CURRENT_THRESHOLD)
      {
        Serial.println("False Alarm, dryer not off long enough, back to RUNNING we go!");
        waitCheck = false;
        dryerState = RUNNING;
      }
      else
      {
        if ( (millis() - startTime) >= WAITOFFTIME)
        {
          Serial.println("Dryer cycle complete");
          String theMessage = GetAMessage('d');

          // How long was the cycle?
          // I don't really care, so commeting out
          // Uncomment if you want this info in the tweet
          //        str.print( F(" Run Time: " ));
          //        double totalRun = (double) (millis() - runStartTime) / 1000 / 3600;
          //        str.print(totalRun);
          //        if(totalRun < 0)
          //          str.print("hr");
          //        else
          //          str.print("hrs");


          Serial.println("Posting to Maker channel");
          Serial.println("Posting the following: ");
          Serial.println(theMessage);
          Serial.println("But I won't be posting the message until I can move the string over");

          // Use IFTTT instead of Twitter now.
          int theResult = sendIftttMakerEvent();



          Serial.println ("Resetting state back to WAITON");
          dryerState = WAITON;
          waitCheck = false;
          digitalWrite(DRYLED, HIGH);
          digitalWrite(REDLED, LOW);
        }
      }
  }

}// end monitorDryer



/*-----------------------------------


  pulls a message from a web page


   ---------------------------------*/
String GetAMessage(char theType)
{
  if (!WEBMSG) { // Pull message from web is off

    if (theType == 'w') { // washer is done
      return "Wash is done.";
    }
    else { // dryer is done
      return "Dryer is done.";
    }

  }

  if (WEBMSG) { // Pull message from web

    if (theType == 'w') { // washer is done
      //return "Wash is done.";
      String myTempString = strFromMySQL("washer");
      char myTempChar [myTempString.length()];
      myTempString.toCharArray(myTempChar, myTempString.length());
      return myTempChar;

    }
    else { // dryer is done
      //return "Dryer is done.";
      String myTempString = strFromMySQL("dryer");
      char myTempChar [myTempString.length()];
      myTempString.toCharArray(myTempChar, myTempString.length());
      return myTempChar;
    }
  }
}// end getAMessage

/*-----------------------------------

  strFromMySQL
  Function to pull a random string from from mySQL for quote
  Requires setting up mySQL db and hosting the required pages
  See http://www.little.org/blog/?p=2229

   ---------------------------------*/
String strFromMySQL(String theDevice)
{
  String mySqlString = "";
  if (client.connect(stringServer, 80)) {
    // Set the path to the location of your washer/dryer db strings
    client.println("GET " + stringServerPath + theDevice + ".php HTTP/1.1"); // dryer quote page
    client.print("Host: ");
    client.print(stringServer);
    client.print("\n");
    client.println("Connection: close");
    client.println();

  }
  else {
    Serial.println(" connection failed");
  }
  //  while (client.connected()) {
  //    if (client.available()) {
  //      if (finder.find("<") ) {
  //        char c;
  //        for (int i = 0; i < 200; i++) {
  //          c = client.read();
  //          // did we find the terminating string?
  //          if (c == '>') {
  //            break;
  //          }
  //          mySqlString += c;
  //        }
  //      }
  //    }
  //  }

  client.stop();
  client.flush();
  return mySqlString;
} // end strFromMySQL() //
// On error, print PROGMEM string to serial monitor and stop
int hang(const __FlashStringHelper * str) {
  Serial.println(str);
  for (;;);
  return 1;
}


// helper functions for constructing the POST data
// append a string or int to a buffer, return the resulting end of string

char *append_str(char *here, char *s) {
  while (*here++ = *s++)
    ;
  return here - 1;
}

char *append_ul(char *here, unsigned long u) {
  char buf[20];       // we "just know" this is big enough

  return append_str(here, ultoa(u, buf, 10));
}


//
