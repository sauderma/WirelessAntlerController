// **********************************************************************************
// This sketch is an example of how wireless programming can be achieved with a Moteino
// that was loaded with a custom 1k bootloader (DualOptiboot) that is capable of loading
// a new sketch from an external SPI flash chip
// The sketch includes logic to receive the new sketch 'over-the-air' and store it in
// the FLASH chip, then restart the Moteino so the bootloader can continue the job of
// actually reflashing the internal flash memory from the external FLASH memory chip flash image
// The handshake protocol that receives the sketch wirelessly by means of the RFM69 radio
// is handled by the SPIFLash/RFM69_OTA library, which also relies on the RFM69 library
// These libraries and custom 1k Optiboot bootloader are at: http://github.com/lowpowerlab
// **********************************************************************************
// Copyright Felix Rusu 2020, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
//#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID       3  // node ID used for this unit
#define NETWORKID    150
#define GATEWAY1     1
#define GATEWAY2     2
#define GATEWAY3     3
#define BROADCASTID  0
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
#define FREQUENCY     RF69_915MHZ // choose this one
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

byte currentState; // What is the current state of this module?

SPIFlash flash(SS_FLASHMEM, FLASH_ID);

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char input = 0;
long lastPeriod = -1;

// struct for EEPROM config
struct configuration {
  byte frequency; // What family are we working in? Basically always going to be 915Mhz in RCMH.
  long frequency_exact; // The exact frequency we're operating at.
  byte isHW;
  byte nodeID;    // 8bit address (up to 255)
  byte networkID; // 8bit address (up to 255)
  byte gatewayID; // 8bit address (up to 255)
  char encryptionKey[16];
  byte state;     // Just in case we want to save a state setting.
  byte codeversion; // What version code we're using
} CONFIG;

// struct for packets being sent to antler hats
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  state; // What state are we being told to go into?
  bool  antlerState; // What state do we want the actual antlers to be in?
  bool  antlerStateUse; // Should we pay attention to the incoming Antler state?
  long  sleepTime; // In milliseconds. Used if we want to overwrite pre-defined states
  bool  sleepTimeUse; // Should we pay attention to the incoming sleep time?
} ToAntlersPayload;
ToAntlersPayload antlersPayload;

// struct for packets being sent to controllers
typedef struct {
  byte  nodeId; // Sender node ID
  byte  version; // What version payload
  byte  state; // What state Hat node is currently in
  bool  antlerState; // What state the antlers are currently in
  float vcc; // VCC read from battery monitor
  int   rssi; // RSSI signal from last received transmission
  int   temperature; // Temperature of the radio
} ToControllersPayload;
ToControllersPayload controllersPayload;

  void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

//*************************************
// Setup                              *
//*************************************

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  //EEPROM.setMaxAllowedWrites(10000);
  //EEPROM.readBlock(0, CONFIG); pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(1000);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY); //OPTIONAL

  //nodeID=CONFIG.nodeID;
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

if (CONFIG.isHW) {
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
}

  Serial.print("Start node ");
  Serial.println(NODEID);

  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL!");

  Serial.println("Listening at 915 Mhz...");
  Serial.println(FREQUENCY_EXACT);
  Serial.println(NETWORKID);
  Serial.println(ENCRYPTKEY);

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

//*************************************
// Blink function                     *
//*************************************

void blinkLED(int blinkTime, int blinkNumber)
{
  // This is terrible don't do this, it blocks the whole thing.

  // int counter = 0;
  // while (counter <= blinkNumber) {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(blinkTime);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(blinkTime);
  //   counter++;
  // }
}

void sendAntlerPayload(byte hatState, bool antlerState, bool antlerStateUse, long sleepTime, bool sleepTimeUse, byte node = 0)
{
  antlersPayload.nodeId = NODEID;
  antlersPayload.version = VERSION;
  antlersPayload.state = hatState;
  antlersPayload.antlerState = antlerState;
  antlersPayload.antlerStateUse = antlerStateUse;
  antlersPayload.sleepTime = sleepTime;
  antlersPayload.sleepTimeUse = sleepTimeUse;

//  if (radio.send(255, (const void*)(&antlersPayload), sizeof(antlersPayload), false))
//    Serial.println("Sent payload");
//  else Serial.println("Send failed for some reason");

  //radio.send(255, (const void*)(&antlersPayload), sizeof(antlersPayload), false);
  //  antlersPayload.nodeId = NODEID;
  
  radio.send(node, (const void*)(&antlersPayload), sizeof(antlersPayload), false);
    //Serial.println("Send succeeded");
  //else Serial.println("Send failed");

  // if (radio.canSend()) {
  //   Serial.println("Radio ready to send");
  // }

  // if (radio.sendWithRetry(HATID, "Hi", 2)) {//target node Id, message as string or byte array, message length
  //   Serial.println("Send worked");
  //   }
  // else {
  //   Serial.println("Send failed");
  // }
  Serial.println(antlersPayload.version);
  Serial.println(antlersPayload.state);
  Serial.println(antlersPayload.antlerState);
  Serial.println(antlersPayload.antlerStateUse);
  Serial.println(antlersPayload.sleepTime);
  Serial.println(antlersPayload.sleepTimeUse);
}


//*************************************
// Loop                               *
//*************************************

void loop(){

    // Handle serial input
    if (Serial.available() > 0) {
      input = Serial.parseInt();
      //int intInput = input - '0'

      if (input >= 1 && input <= 9) { //0-9
        Serial.print("\nSending state "); Serial.println(input);
        sendAntlerPayload((byte)input, 0, 0, 0, 0);
      } 
    }  // close if Serial.available()
  
  // Check for existing RF data
  if (radio.receiveDone()) {

    if (radio.ACKRequested()) {
      radio.sendACK();
      #ifdef DEBUG_MODE
        Serial.print(" - ACK sent");
      #endif
    }
    
    // Check for a new OTA sketch. If so, update will be applied and unit restarted.
    CheckForWirelessHEX(radio, flash, false);

   #ifdef DEBUG_MODE
      Serial.print("Got [");
      Serial.print(radio.SENDERID);
      Serial.print(':');
      Serial.print(radio.DATALEN);
      Serial.print("] > ");
      for (byte i = 0; i < radio.DATALEN; i++)
        Serial.print((char)radio.DATA[i], HEX);
      Serial.println();
    #endif

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
      Serial.print("ID:");Serial.println(controllersPayload.nodeId);      // Node IS
      Serial.print("VS:");Serial.println(controllersPayload.version);     // Payload version
      Serial.print("ST:");Serial.println(controllersPayload.state);       // Node state
      Serial.print("AS:");Serial.println(controllersPayload.antlerState); // Antler state
      Serial.print("VC:");Serial.println(controllersPayload.vcc);         // Battery voltage
      Serial.print("RS:");Serial.println(controllersPayload.rssi);        // RSSI strength
      Serial.print("TP:");Serial.println(controllersPayload.temperature); // Radio temperature
    
    //} // close valid payload
  } // close radio.receiveDone()
} // close loop()

