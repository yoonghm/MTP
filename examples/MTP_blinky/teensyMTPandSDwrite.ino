/*
 * 
 * Modified version of MTP Blinky that allows you to switch between MTP and SD writing with the same sketch.
 * By Jared Reabow May 2020, written for teensy 3.5/3.6
 * 
 * 
  */
#define USBMODE 0
#define SDLOGGINGMODE 1
#define NOSDDETECTED 3
#define SWITCHPIN 23

#include "SdFat.h"
SdFatSdioEX sdEx;
File file;
  #include <MTP.h>
  MTPStorage_SD storage;
  MTPD          mtpd(&storage);
  bool checkSDInstalled(int type){
    if(type == 0){
      file.close();
      if (!SD.begin()) {
        Serial.print("MTP failed");
        return false;
      } else {
        return true;
      }
    } else {
      file.close();
      if (!sdEx.begin()) {
        Serial.print("SdFatSdioEX begin() failed");
        return false;
      } else {
        return true;
      }
    }
    // make sdEx the current volume.
    //sdEx.chvol();
  }
  void enableUSB(){
      mtpd.loop();
  }



volatile int  status = 0;
volatile bool sdfound = 0;
volatile bool mode = USBMODE;
volatile int  count = 1;
volatile bool currentSwitchPosition;
volatile bool previousSwitchPosition;

int counter = 0;
String outputString = "";

void rtc_seconds_isr() {//activates each second
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Program cycles: ");Serial.print(counter);Serial.print(" ");
    Serial.print(" ");Serial.println(outputString);
    counter = 0;
    digitalWrite(LED_BUILTIN, LOW);
}




void setup() {

///seconds timer interrupt
    RTC_IER |= 0x10;  // Enable seconds IRQ from RTC peripheral
    NVIC_ENABLE_IRQ(IRQ_RTC_SECOND); // Enable seconds IRS function in NVIC
/// end of second timer interrupt

//enable 5v output
  PORTE_PCR6 = PORT_PCR_MUX(1);
  GPIOE_PDDR |= (1<<6);
  GPIOE_PSOR = (1<<6); // turn on USB host power  
///end of enabled 5v output

  Serial.begin(2000000);
  pinMode(LED_BUILTIN, OUTPUT);
  if (checkSDInstalled(0)==true) {
    sdfound = true;
  } else {
    sdfound = false;
  }
  
  pinMode(SWITCHPIN,HIGH);
  

}


void loop() {
  counter++;
  viewSDonUSB();
}


void viewSDonUSB(){
  currentSwitchPosition = digitalRead(SWITCHPIN);
  if(sdfound == true){ // if an SD card is found and working
    if(currentSwitchPosition != previousSwitchPosition){
      previousSwitchPosition = currentSwitchPosition;
      if(currentSwitchPosition == HIGH){
        Serial.println("USB MODE ENABLED");
        checkSDInstalled(USBMODE);
        mode = USBMODE;
      } else {
        Serial.println("SD LOGGING MODE ENABLED");
        checkSDInstalled(SDLOGGINGMODE);
        mode = SDLOGGINGMODE;
      }
    }
  } else { 
    outputString = ("NO SD CARD DETECTED");
    mode = NOSDDETECTED;
  } 

  if(mode != NOSDDETECTED){ 
      if(mode == USBMODE){ // if the hardware pin for SD view is enabled
        outputString = ("usb storage enabled, system will run slower");
        enableUSB(); //448473 cycles no pin check//
        digitalWriteFast(LED_BUILTIN,HIGH);
      } else if(mode == SDLOGGINGMODE){ 
        outputString = ("HARDWARE SD TO PC DISABLED"); digitalWriteFast(LED_BUILTIN,LOW);
        runTest();
      } 
  }
}


void runTest() {
  if(sdEx.exists("TeensyDemo.txt")){
    Serial.println("FILE FOUND");
    delay(1000);
  } else {
    Serial.println("FILE NOT FOUND");
    delay(1000);
  }
  /*String fileString = String(millis());
  fileString += ".txt";
  char fileNameStore[20];
  fileString.toCharArray(fileNameStore, 20);
  Serial.print("FILE NAME ");Serial.println(fileString);*/
  if (!file.open("TeensyDemo.txt", O_WRITE | O_CREAT | O_AT_END)) {
        Serial.println("SD CARD FILE NOT OPENED");
  } else {
    Serial.println("SD CARD FILE OPENED");
  }
  String outputString = String(millis());
  outputString += "-By JR- Teensy MTP & SD demo";
  if (!file.println(outputString)) {
        Serial.println("write failed");
  } else {
    Serial.println("write success");
    delay(1000);
  }
  file.close();
}
  
