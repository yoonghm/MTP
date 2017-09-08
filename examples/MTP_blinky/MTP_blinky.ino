/*
  This example demonstrates MTP with blinky using systick interrupt.

  This example tests MTP and SdFat
*/

#include <MTP.h>

MTPStorage_SD storage;
MTPD          mtpd(&storage);


volatile int  status = 0;
volatile bool sdfound = 0;
volatile int  count = 1;

void rtc_seconds_isr() {
  if (count-- == 0) {
    digitalWrite(LED_BUILTIN, status);
    Serial.println("I should be commented out");
    status = !status;
    if (sdfound)
      count = 2;
    else
      count = 1;
  }
}

void setup() {
  Serial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);

  RTC_IER |= 0x10;  // Enable seconds IRQ from RTC peripheral
  NVIC_ENABLE_IRQ(IRQ_RTC_SECOND); // Enable seconds IRS function in NVIC
}

void loop() {
  if (SD.begin()) {
    sdfound = true;
    mtpd.loop();
  }
  else {
    sdfound = false;
  }
}
