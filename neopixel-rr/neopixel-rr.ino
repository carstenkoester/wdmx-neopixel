// - compare sACN with output on laptop (maybe run Capture on laptop or some sACN analyzer on laptop). Just to find out if the issue is receive side or WiFi side.
//   May want to create separate class with receive thread on CPU 0.

#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_COUNT   100

/*
 * Runtime configurables and variables
 */
rf24DmxGadget gadget("NeoRR24", DMXGADGET_BOARD_HUZZAH32_PROPMAKER_RF24, LED_COUNT);
// DmxNowDmxGadget gadget("NeoRRNow", DMXGADGET_BOARD_HUZZAH32_PROPMAKER_RF24, LED_COUNT);
// sACNDmxGadget gadget("NeoRRSACN", DMXGADGET_BOARD_HUZZAH32_PROPMAKER_RF24, LED_COUNT);

struct dmxData {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};
BLEUIntConfigItem numGroups("LED Groups", 4);

void setup() {
//  gadget.setup(std::vector<BLEConfigItem*> {&numGroups});
  gadget.setup({&numGroups});
}

unsigned long lastMillis = 0;

void loop() {
  dmxData dmx_data[LED_COUNT * sizeof(dmxData)];
  unsigned int num_groups = numGroups.value();

  if (millis() > lastMillis+25) {
  gadget.loop();
  gadget.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);

  for(unsigned int i=0; i<LED_COUNT; i++) {
    gadget.strip.setPixelColor(i, dmx_data[i % num_groups].red, dmx_data[i % num_groups].green, dmx_data[i % num_groups].blue);
  }
  gadget.strip.show();
  lastMillis=millis();
  }
  // delay(25);
}