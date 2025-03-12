#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_COUNT                     sizeof(PIXEL_SECTIONS)/sizeof(PIXEL_SECTIONS[0])

// For each pixel, indicate which section it is in. Indicating 0 means this pixel in no section at all.
const unsigned int PIXEL_SECTIONS[] = {
   1,  1,  1,  1,  1,  1,  1,  1,  1,  0,        // Pixels 1-9 are in section 0
   2,  2,  2,  2,  2,  2,  2,  2,  2,  0,        // Pixels 11-19 are in section 1
   3,  3,  3,  3,  3,  3,  3,  3,  3,  0,        // Pixels 21-19 are in section 2
   4,  4,  4,  4,  0,  0,  5,  5,  5,  5,        // Pixels 31-34 are in section 3. Pixels 37-40 are in section 4.
   0,  6,  6,  6,  6,  6,  6,  6,  6,  6,        // Pixels 42-50 are in section 5
   0,  7,  7,  7,  7,  7,  7,  7,  7,  7,        // Pixels 52-60 are in section 6
   0,  8,  8,  8,  8,  8,  8,  8,  8,  8,        // Pixels 62-70 are in section 7
   0,  0,  0,  0,  9,  9,  9,  9,  9,  9,        // Pixels 75-80 are in section 8
   0, 10, 10, 10, 10, 10, 10
};

/*
 * Runtime configurables and variables
 */
dmxGadget gadget("NeoMulti", LED_COUNT);

struct dmxData {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};
unsigned int num_sections = 0;

void setup() {
  gadget.setup();
  gadget.config.advertise();

  for(unsigned int i=0; i<LED_COUNT; i++) {
    if (PIXEL_SECTIONS[i] > num_sections) {
      num_sections = PIXEL_SECTIONS[i];
    }
  }
  Serial.printf("Have %d LEDs in %d sections\n", LED_COUNT, num_sections);
}

// inline uint8_t pixelIntensity(unsigned int i, uint8_t value)
// {
//   return ((value * dmx_data[PIXEL_SECTIONS[i]].intensity)/255);
// }

void loop() {
  dmxData dmx_data[num_sections];

  gadget.loop();
  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);

  for(unsigned int i=0; i<LED_COUNT; i++) {
    if (PIXEL_SECTIONS[i] > 0) {
      gadget.strip.setPixelColor(i, dmx_data[PIXEL_SECTIONS[i]-1].red, dmx_data[PIXEL_SECTIONS[i]-1].green, dmx_data[PIXEL_SECTIONS[i]-1].blue);
    } else {
      gadget.strip.setPixelColor(i, 0, 0, 0);
    }
  }
  gadget.strip.show();
  delay(25);
}