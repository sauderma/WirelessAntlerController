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
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID       1  // node ID used for this unit
#define NETWORKID    150
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
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
#define VERSION 1    // Version of code programmed

int currentState; // What is the current state of this module?

SPIFlash flash(SS_FLASHMEM, FLASH_ID);

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char input = 0;
long lastPeriod = -1;

typedef struct {
  byte  receivedNodeId; // node received payload from
  byte  receivedVersion; // What version payload did we receive?
  int   receivedState; // What state are we being told to go into?
  bool  receivedAntlerState; // Used if we want to overwrite pre-defined states
  bool  receivedAntlerStateUse; // Should we pay attention to the incoming Antler state?
  int   receivedSleepTime; // In milliseconds. Used if we want to overwrite pre-defined states
  bool  receivedSleepTimeUse; // Should we pay attention to the incoming sleep time?
} ToAntlersPayload;
ToAntlersPayload incomingData;

typedef struct {
  byte  outgoingNodeId; // Node sending to
  byte  outgoingVersion; // Version payload we're sending
  int   outoingState; // What state we are currently in
  bool  outgoingAntlerState; // What state the antlers are currently in
  float outgoingVCC; // VCC read from battery monitor
  int   outgoingRSSI; // RSSI signal from last received transmission
  int   outoingTemperature; // Temperature of the radio
} ToControllersPayload;
ToControllersPayload outgoingData;

//*************************************
// Setup                              *
//*************************************

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(1000);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY); //OPTIONAL

#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
#endif

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif

  Serial.print("Start node ");
  Serial.println(NODEID);

  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL!");

  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  Serial.flush();

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
  int counter = 0;
  while (counter <= blinkNumber) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(blinkTime);
    counter++;
  }
}

//*************************************
// Loop                               *
//*************************************

void loop(){

  #ifdef DEBUG_MODE
    // Handle serial input (to allow basic DEBUGGING of FLASH chip)
    // ie: display first 256 bytes in FLASH, erase chip, write bytes at first 10 positions, etc
    if (Serial.available() > 0) {
      input = Serial.read();
      if (input == 'd') { //d=dump first page
        Serial.println("Flash content:");
        int counter = 0;
        while(counter<=256) {
          Serial.print(flash.readByte(counter++), HEX);
          Serial.print('.');
        }
        
        Serial.println();
      }
      else if (input == 'D') { //d=dump higher memory
        Serial.println("Flash content:");
        uint16_t counter = 4090; //dump the memory between the first 4K and second 4K sectors

        while(counter<=4200) {
          Serial.print(flash.readByte(counter++), HEX);
          Serial.print('.');
        }        
        Serial.println();
      }
      else if (input == 'e') {
        Serial.print("Erasing Flash chip ... ");
        flash.chipErase();
        while(flash.busy());
        Serial.println("DONE");
      }
      else if (input == 'i') {
        Serial.print("DeviceID: ");
        Serial.println(flash.readDeviceId(), HEX);
      }
      else if (input == 'r') {
        Serial.print("Rebooting");
        resetUsingWatchdog(true);
      }
      else if (input == 'R') {
        Serial.print("RFM69 registers:");
        radio.readAllRegs();
      }
      else if (input >= 48 && input <= 57) { //0-9
        Serial.print("\nWriteByte("); Serial.print(input); Serial.print(")");
        flash.writeByte(input-48, millis()%2 ? 0xaa : 0xbb);
      } 
    }    // close if Serial.available()
  #endif // close #ifdef DEBUG_MODE
  
  // Check for existing RF data
  if (radio.receiveDone()) {

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

    // Check for a new OTA sketch. If so, update will be applied and unit restarted.
    CheckForWirelessHEX(radio, flash, false);
    
    // Check if valid packet. In future perhaps add checking for different payload versions
    if (radio.DATALEN != sizeof(ToAntlersPayload)) {
      #ifdef DEBBUG_MODE
        Serial.print("Invalid payload received, not matching Payload struct!");
      #endif
    }
    else
    {
      incomingData = *(ToAntlersPayload*)radio.DATA; // We'll hope radio.DATA actually contains our struct and not something else

      #ifdef DEBUG_MODE
        Serial.println("Received data from node: ");
        Serial.println(incomingData.receivedNodeId);
        Serial.println("Received data is payload version: ");
        Serial.println(incomingData.receivedVersion);
        Serial.println("Received state is: ");
        Serial.println(incomingData.receivedState);
        Serial.println("Received antler state is: ");
        Serial.println(incomingData.receivedAntlerState);
        Serial.println("Received sleep time is: ");
        Serial.println(incomingData.receivedSleepTime);
      #endif

      // And now for the real meat, what should we be doing right now?

      if (incomingData.receivedState != currentState) {
        switch (incomingData.receivedState) {
          case 0:
            // This is our deep sleep state
            currentState = 0;
            Serial.println("Entered state 0");
            break;
          case 1:
            // This is our wake up and pay some attention state
            currentState = 1;
            Serial.println("Entered state 1");
            break;
          case 2:
            // This is our standby to do stuff really soon state. 
            currentState = 2;
            Serial.println("Entered state 2");
            break;
          case 3:
            // This is our GO GO GO state
            currentState = 3;
            Serial.println("Entered state 3");
            break;
          case 4:
            // This is the state where we experiment with keeping the antlers on while sleeping the MCU and radio
            break;
          case 5:
            // This is a TBD state
            break;
          case 6:
            // Another TBD state. Perhaps for custom timings and antler states?
            break;
          case 9:
            // This is our programming state. Major power hog. Programming takes time, not sure if want to auto sleep
            currentState = 9;
            Serial.println("Entered state 9");
            break;
        } // Close state switch
      } // close state IF

      // Check if sender wanted an ACK
      if (radio.ACKRequested())
      {
        radio.sendACK();
        #ifdef DEBUG_MODE
          Serial.print(" - ACK sent");
        #endif
      }
    
    } // close valid payload
  } // close radio.receiveDone()
} // close loop()



