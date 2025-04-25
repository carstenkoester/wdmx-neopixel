#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_PIN        1
#define BOARD_TYPE     DMXGADGET_BOARD_XIAO_ESP32S3

/*
 * Runtime configurables and variables
 */
DmxNowDmxGadget gadget("Candle", BOARD_TYPE);

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