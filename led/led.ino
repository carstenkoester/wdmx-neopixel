#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_PIN 33

/*
 * Runtime configurables and variables
 */
dmxGadget gadget("LED Candle", 0);

/*
 * Main functions
 */
void setup() { 
  gadget.setup();
  gadget.config.advertise();
}

void loop() {
  gadget.loop();

  analogWrite(LED_PIN, gadget.receiver.getValue(gadget.dmxAddress.value()));
  delay(25);
}