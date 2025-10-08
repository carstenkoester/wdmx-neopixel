#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_R 19
#define LED_G 22
#define LED_B 21

/*
 * Runtime configurables and variables
 */
rf24DmxGadget gadget("RGB", DMXGADGET_BOARD_HUZZAH32_RF24, 0);

struct dmxData {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} dmx_data;

/*
 * Main functions
 */
void setup() { 
  gadget.setup({});

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
}

void loop() {
  gadget.loop();

  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);
  analogWrite(LED_R, dmx_data.red);
  analogWrite(LED_G, dmx_data.green);
  analogWrite(LED_B, dmx_data.blue);
  delay(25);
}