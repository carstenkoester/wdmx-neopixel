#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_COUNT                     sizeof(PIXEL_CELLS)/sizeof(PIXEL_CELLS[0])

// For each pixel, indicate which cell it is in. Indicating 0 means this pixel in no cell at all.
const unsigned int PIXEL_CELLS[] = {
   1,  1,  1,  1,  1,  1,  1,  1,  1,  0,        // Pixels 1-9 are in cell 1
   2,  2,  2,  2,  2,  2,  2,  2,  2,  0,        // Pixels 11-19 are in cell 2
   3,  3,  3,  3,  3,  3,  3,  3,  3,  0,        // Pixels 21-19 are in cell 3
   4,  4,  4,  4,  0,  0,  5,  5,  5,  5,        // Pixels 31-34 are in cell 4. Pixels 37-40 are in cell 5.
   0,  0,  6,  6,  6,  6,  6,  6,  6,  6,        // Pixels 43-51 are in cell 6
   6,  0,  7,  7,  7,  7,  7,  7,  7,  7,        // Pixels 53-61 are in cell 7
   7,  0,  8,  8,  8,  8,  8,  8,  8,  8,        // Pixels 63-71 are in cell 8
   8,  0,  0,  0,  0,  9,  9,  9,  9,  9,        // Pixels 76-81 are in cell 9
   9, 10, 10, 10, 10, 10, 10                     // Pixels 82-87 are in cell 10
};

/*
 * Runtime configurables and variables
 */
dmxGadget gadget("NeoMC", LED_COUNT);

struct dmxData {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};
unsigned int num_cells = sizeof(PIXEL_CELLS)/sizeof(PIXEL_CELLS[0]);

void setup() {
  gadget.setup();
  gadget.config.advertise();

  for(unsigned int i=0; i<LED_COUNT; i++) {
    if (PIXEL_CELLS[i] > num_cells) {
      num_cells = PIXEL_CELLS[i];
    }
  }
  Serial.printf("Have %d LEDs in %d cells\n", LED_COUNT, num_cells);
}

void loop() {
  dmxData dmx_data[num_cells];

  gadget.loop();
  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);

  for(unsigned int i=0; i<LED_COUNT; i++) {
    if (PIXEL_CELLS[i] > 0) {
      gadget.strip.setPixelColor(i, dmx_data[PIXEL_CELLS[i]-1].red, dmx_data[PIXEL_CELLS[i]-1].green, dmx_data[PIXEL_CELLS[i]-1].blue);
    } else {
      gadget.strip.setPixelColor(i, 0, 0, 0);
    }
  }
  gadget.strip.show();
  delay(25);
}