#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_PIN 33

/*
 * Runtime configurables and variables
 */
DmxNowDmxGadget gadget("Candle", DMXGADGET_BOARD_XIAO_ESP32S3);

/*
 * Main functions
 */
void setup() { 
  gadget.setup();
}

void loop() {
  gadget.loop();

  analogWrite(LED_PIN, gadget.getValue(gadget.dmxAddress.value()));
  delay(25);
}