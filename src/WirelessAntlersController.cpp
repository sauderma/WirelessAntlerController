// **********************************************************************************
// Code for Radio City Music Hall Wireless Antlers Controller module
// **********************************************************************************
// Copyright 2021 Radio City Music Hall
// Contact: Michael Sauder, michael.sauder@msg.com

// **********************************************************************************
// Changelog:
// 
// **********************************************************************************

#include <RFM69.h>         // https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>     // https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     // https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>      // https://github.com/lowpowerlab/spiflash
#include <SafeString.h>    // https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
#include <SafeStringReader.h> // https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
#include <DebugLog.h>

#define NODEID       1
#define NETWORKID    150
#define GATEWAY1     1
#define GATEWAY2     2
#define GATEWAY3     3
#define BROADCASTID  0
#define FREQUENCY     RF69_915MHZ
#define FREQUENCY_EXACT 905500000
#define ENCRYPTKEY  "rcmhprodrcmhprod" //16-bytes or ""/0/null for no encryption
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*****************************************************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
#define FLASH_ID      0xEF30  //ID for the 4Mbit Winbond W25X40CL flash chip
//*****************************************************************************************************************************
//#define BR_300KBPS         //run radio at max rate of 300kbps!
//*****************************************************************************************************************************
#define SERIAL_BAUD 115200
#define BLINKPERIOD 250
//*****************************************************************************************************************************

#define ANTLER_PIN   6 //PWM pin for controlling antler LEDs

#define DEBUG_MODE  //uncomment to enable debug comments
#define VERSION 1   // Version of code programmed

int currentState; // What is the current state of this module?

SPIFlash flash(SS_FLASHMEM, FLASH_ID);

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char input = 0;
long lastPeriod = -1;

createSafeStringReader(sfReader, 80, '\n'); // Create reader up to 80 characters for strings coming in via 
                                            // serial with new line character as the delimiter.

// struct for packets being sent to antler hats
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  nodeState; // What state should node go into?
  byte  antlerState; // What state should the actual antlers to be in?
  long sleepTime; // In milliseconds. Used if we want to overwrite pre-defined states
} ToAntlersPayload;
//ToAntlersPayload antlersPayload;

// struct for packets being sent to controllers
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  nodeState; // What state Hat node is currently in
  byte  antlerState; // What state the antlers are currently in
  float vcc; // VCC read from battery monitor
  int   temperature; // Temperature of the radio
} ToControllersPayload;
ToControllersPayload controllersPayload;

//*****************************************************************************
// Setup                                                                      *
//*****************************************************************************

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(1000);

  SafeString::setOutput(Serial); // enable error messages and debug() output to be sent to Serial
  sfReader.connect(Serial); // connect reader to serial
  sfReader.echoOn(); // not sure

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY);

  radio.setFrequency(FREQUENCY_EXACT);

  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif

  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); // Only for RFM69HW/HCW. Damage may occur if enabled on non-HW/HCW nodes.
  #endif

  Serial.print("Starting node: ");
  Serial.println(NODEID);

  if (flash.initialize())
    LOG_INFO("SPI Flash Init OK!");
  else
    LOG_WARN("SPI Flash Init FAIL!");

  LOG_INFO("Listening at 915 Mhz...");
  LOG_INFO(FREQUENCY_EXACT);
  LOG_INFO(NETWORKID);
  LOG_INFO(ENCRYPTKEY);

#ifdef BR_300KBPS
  radio.writeReg(0x03, 0x00);  //REG_BITRATEMSB: 300kbps (0x006B, see DS p20)
  radio.writeReg(0x04, 0x6B);  //REG_BITRATELSB: 300kbps (0x006B, see DS p20)
  radio.writeReg(0x19, 0x40);  //REG_RXBW: 500kHz
  radio.writeReg(0x1A, 0x80);  //REG_AFCBW: 500kHz
  radio.writeReg(0x05, 0x13);  //REG_FDEVMSB: 300khz (0x1333)
  radio.writeReg(0x06, 0x33);  //REG_FDEVLSB: 300khz (0x1333)
  radio.writeReg(0x29, 240);   //set REG_RSSITHRESH to -120dBm
#endif
}

//*****************************************************************************
// The Functions                                                              *
//*****************************************************************************

//*************************************
// Radio send function                *
//*************************************

void sendAntlerPayload(ToAntlersPayload payload, byte node = 0)
{
  radio.send(node, (const void*)(&payload), sizeof(payload), false);
  LOG_INFO("Antler payload sent");

  //radio.send(0, "hi", 2);
}

//*************************************
// Parse serial data                  *
//*************************************

bool parseSerialData (SafeString& msg) {
  ToAntlersPayload antlerPayload; // Struct to populate
  antlerPayload.nodeId = NODEID; // First payload value is the node ID of this controller

  int tempInt;
  byte tempByte;

  int toNode; // Node we're going to be sending to
  cSF(sfKey, 30); // temp SafeString to receive keys, max field len is 2;
  //cSF(sfValue, 20); // temp SafeString to receive values, max field len is 10;
  char delimKey[] = ":,"; // keys separated from values by :
  //char delimValue[] = ","; // key/value pairs are separated by commas
  //bool returnEmptyFields = true; // return empty field for ,,
  int idx = 2;  // Index holding our location in the message.
                // Skip the first two characters, which should be a "!!"

  msg.removeBefore(2);

  msg.nextToken(sfKey, delimKey); // get the Node ID header, stick it into "sfKey"
  LOG_INFO("sfKey Node header is: "); LOG_INFO(sfKey);
  if (!sfKey.equals("ND")){  // Check the first field is "ND" (node ID) otherwise something went wrong.
    LOG_WARN("ND didn't pass");
    return false;
  }
  msg.nextToken(sfKey, delimKey); //Get the Node ID value
  sfKey.toInt(toNode); // First value is the node we're sending to, save for later.
  LOG_INFO("sfkey Node value is:"); LOG_INFO(toNode);

  msg.nextToken(sfKey, delimKey); // get Version header
  LOG_INFO("sfKey Version header is: "); LOG_INFO(sfKey);
  if (!sfKey.equals("VR")) {
    LOG_WARN("VS didn't pass");
    return false;
  }
  msg.nextToken(sfKey, delimKey); // Get Version value
  LOG_INFO("sfKey Version value is: "); LOG_INFO(sfKey);
  //sfKey.toInt(antlerPayload.version);
  LOG_WARN(antlerPayload.version);
  sfKey.toInt(tempInt);
  antlerPayload.version = byte(tempInt);


  msg.nextToken(sfKey, delimKey); // get Node state header
  LOG_INFO("sfKey Node State header is: "); LOG_INFO(sfKey);
  if (!sfKey.equals("NS")) {
    LOG_WARN("NS didn't pass");
    return false;
  }
  msg.nextToken(sfKey, delimKey); // get Node state value
  LOG_INFO("sfKey Node State value is: "); LOG_INFO(sfKey);
  //sfKey.toInt(antlerPayload.nodeState);
  LOG_WARN(antlerPayload.nodeState);
  sfKey.toInt(tempInt);
  antlerPayload.nodeState = byte(tempInt);

  msg.nextToken(sfKey, delimKey); // get Antler state header
  LOG_INFO("sfKey Antler State header is: "); LOG_INFO(sfKey);
  if (!sfKey.equals("AS")) {
    LOG_WARN("AS didn't pass");
    return false;
  }
  msg.nextToken(sfKey, delimKey); // get Antler state value
  LOG_INFO("sfKey Antler State header is: "); LOG_INFO(sfKey);
  //sfKey.toInt(antlerPayload.antlerState);
  LOG_WARN(antlerPayload.antlerState);
  sfKey.toInt(tempInt);
  antlerPayload.antlerState = byte(tempInt);

  msg.nextToken(sfKey, delimKey); // get node sleep time header
  LOG_INFO("sfKey Time header is: "); LOG_INFO(sfKey);
  if (!sfKey.equals("SL")) {
    LOG_WARN("SL didn't pass");
    return false;
  }
  msg.nextToken(sfKey, delimKey); // get node sleep time value
  LOG_INFO("sfKey Time value is: "); LOG_INFO(sfKey);
  sfKey.toLong(antlerPayload.sleepTime);
  LOG_WARN(antlerPayload.sleepTime);

  // Send all the above stuff over to the nodes
  sendAntlerPayload(antlerPayload, toNode);

  return true;

}


//*************************************
// Blink function                     *
//*************************************

void blinkLED(int blinkTime, int blinkNumber)
{

}

//*****************************************************************************
// The Loop                                                                   *
//*****************************************************************************

void loop(){

  // sfReader is our serial port handler, check for incoming serial data
  if (sfReader.read()) {
    sfReader.trim(); // remove any leading/trailing white space
    if (sfReader.startsWith("!!")) {
        parseSerialData(sfReader); // Process incoming data string
      }
      else {
        LOG_WARN("Incoming serial packet did not start with !!");
      }
    }
  
  // Check for new RF data
  if (radio.receiveDone()) {

    if (radio.ACKRequested()) {
      radio.sendACK();
      LOG_INFO("Radio ACK sent");
    }
    
    // Check for a new OTA sketch. If so, update will be applied and unit restarted.
    // Disabled for the controllers.
    // CheckForWirelessHEX(radio, flash, false);

  //  #ifdef DEBUG_MODE
  //     Serial.print("Got [");
  //     Serial.print(radio.SENDERID);
  //     Serial.print(':');
  //     Serial.print(radio.DATALEN);
  //     Serial.print("] > ");
  //     for (byte i = 0; i < radio.DATALEN; i++)
  //       Serial.print((char)radio.DATA[i], HEX);
  //     Serial.println();
  //   #endif

    // Check if valid packet. In future perhaps add checking for different payload versions
//    if (radio.DATALEN != sizeof(ToControllersPayload)) {
//      #ifdef DEBUG_MODE
//        Serial.print("Invalid payload received, not matching Payload struct!");
//      #endif
//    }
//    else
//    {
      controllersPayload = *(ToControllersPayload*)radio.DATA; // We'll hope radio.DATA actually contains our struct and not something else

      //Send the data straight out the serial, we don't actually need to do anything with it internally
      Serial.print("##");
      Serial.print("ID:");Serial.print(controllersPayload.nodeId);Serial.print(",");      // Node ID
      Serial.print("VR:");Serial.print(controllersPayload.version);Serial.print(",");     // Payload version
      Serial.print("NS:");Serial.print(controllersPayload.nodeState);Serial.print(",");       // Node state
      Serial.print("AS:");Serial.print(controllersPayload.antlerState);Serial.print(","); // Antler state
      Serial.print("VC:");Serial.print(controllersPayload.vcc);Serial.print(",");         // Battery voltage
      Serial.print("TP:");Serial.println(controllersPayload.temperature); // Radio temperature
    
    //} // close valid payload
  } // close radio.receiveDone()
} // close loop()