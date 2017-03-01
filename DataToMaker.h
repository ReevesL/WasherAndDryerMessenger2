#include <Arduino.h>
#include <Adafruit_CC3000.h>

// CC3000 interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3 // MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT  5 // These can be
#define ADAFRUIT_CC3000_CS   10 // any two pins
// Hardware SPI required for remaining pins.
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11


#ifndef DataToMaker_h
class DataToMaker
{
  public:
    DataToMaker(const char*, String); // constructor
    bool connect();
    bool setValue(int, String);
    void sendToMaker();
    void post();

  protected: // it is protected because the subclass needs access
    //to max distance!

  private:
    void compileData();
    Adafruit_CC3000_Client client;
    const char* privateKey;
    String event;
    String value1, value2, value3 = "";
    bool dataAvailable;
    String postData;
    uint32_t ip = 0L, t;
    Adafruit_CC3000 cc3000 = Adafruit_CC3000(
                               ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                               SPI_CLOCK_DIVIDER);

};

DataToMaker::DataToMaker(const char* _privateKey, String _event)
{
  privateKey = _privateKey;
  event = _event;
}

bool DataToMaker::connect()
{
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

  if (cc3000.connectTCP(ip, 80))
    return true;
  else return false;
}

void DataToMaker::post()
{
  compileData();
  client.fastrprint("POST /trigger/");
  client.fastrprint(event.c_str());
  client.fastrprint("/with/key/");
  client.fastrprint(privateKey);
  client.fastrprint(" HTTP/1.1\r\n");
  client.fastrprint("Host: maker.ifttt.com\r\n");
  client.fastrprint("User-Agent: Arduino/1.0\r\n");
  client.fastrprint("Connection: close\r\n");
  if (dataAvailable)
  { // append json values if available
    client.fastrprint("Content-Type: application/json\r\n");
    client.fastrprint("Content-Length: ");
    client.fastrprint((char*)postData.length());
    client.fastrprint("\r\n");
    client.fastrprint("\r\n\r\n");
    client.fastrprint(postData.c_str());
  }
  else
    client.println();
}

bool DataToMaker::setValue(int valueToSet, String value)
{
  switch (valueToSet)
  {
    case 1:
      value1 = value;
      break;
    case 2:
      value2 = value;
      break;
    case 3:
      value3 = value;
      break;
    default:
      return false;
      break;
  }
  return true;
}

void DataToMaker::compileData()
{
  if (value1 != "" || value2 != "" || value3 != "")
  {
    dataAvailable = true;
    bool valueEntered = false;
    postData = "{";
    if (value1 != "")
    {
      postData.concat("\"value1\":\"");
      postData.concat(value1);
      valueEntered = true;
    }
    if (value2 != "")
    {
      if (valueEntered)postData.concat("\",");
      postData.concat("\"value2\":\"");
      postData.concat(value2);
      valueEntered = true;
    }
    if (value3 != "")
    {
      if (valueEntered)postData.concat("\",");
      postData.concat("\"value3\":\"");
      postData.concat(value3);
    }
    postData.concat("\"}");
  }
  else dataAvailable = false;
}
#endif
