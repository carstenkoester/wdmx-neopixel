#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_POWER 33
#define LED_R 27
#define LED_G 12
#define LED_B 13

/*
 * Runtime configurables and variables
 */
dmxGadget gadget("RGBW", 0);

struct dmxData {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} dmx_data;

/*
 * Main functions
 */
void setup() { 
  gadget.setup();
  gadget.config.advertise();

  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_POWER, HIGH);
}

void loop() {
  gadget.loop();

  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);
  analogWrite(LED_R, dmx_data.red);
  analogWrite(LED_G, dmx_data.green);
  analogWrite(LED_B, dmx_data.blue);
  delay(25);
}