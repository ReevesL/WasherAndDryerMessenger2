// rename this file to "privateConfig.h" and add your personal variables
// this file was separated out to simplify my check ins to Git

// WiFi access point credentials
#define WLAN_SSID     "Your_SSID"  // 32 characters max
#define WLAN_PASS     "your_password"

// What page to grab!
#define WEBSITE      "maker.ifttt.com"
#define WEBPAGE      "/trigger/your_event/with/key/your_key"

// strings for constructing request
#define MakerIFTTT_Event "your_event"
#define MakerIFTTT_Key "your_key"

// Random strings db settings
char stringServer[] = "YOUR_STRING_SERVER";    // Server for strings
String stringServerPath = "/yourPathHere/";
