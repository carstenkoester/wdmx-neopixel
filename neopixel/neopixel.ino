#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_COUNT                     100  // How many LED pixels are attached to the Arduino?

/*
 * Runtime configurables and variables
 */
dmxGadget gadget("Neopixel", LED_COUNT);

struct dmxData {
  uint8_t intensity;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t macro;
} dmx_data;

uint16_t firstPixelHue = 0;

void setup() {
  gadget.setup();
  gadget.config.advertise();
}

inline uint8_t pixelIntensity(uint8_t value)
{
  return ((value * dmx_data.intensity)/255);
}

void loop() {
  gadget.loop();

  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);
  if (dmx_data.macro < 128) {
    for(unsigned int i=0; i<gadget.strip.numPixels(); i++) {
      gadget.strip.setPixelColor(i, pixelIntensity(dmx_data.red), pixelIntensity(dmx_data.green), pixelIntensity(dmx_data.blue));
    }
    gadget.strip.show();
    delay(25);
  } else {
    gadget.strip.rainbow(firstPixelHue, 1, 255, dmx_data.intensity, false);
    gadget.strip.show();
    firstPixelHue += 256; // This is a 16-bit unsigned integer so will wrap at 65535
    delay(255-dmx_data.macro);
  }
}