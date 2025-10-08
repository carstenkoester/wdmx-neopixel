#include <dmxGadget.h>
#include "esp_wifi.h"

/*
 * Compile-time configurables
 */
const dmxgadget_board_t  BOARD               = DMXGADGET_BOARD_SPARKLE_MOTION_MINI;
// const uint8_t            LED_PINS[3]         = {19, 22, 21};  // Sparkle Motion
const uint8_t            LED_PINS[2]         = {32, 33};      // Sparkle Motion Mini

#define                  PWM_FREQ              100
#define                  PWM_RESOLUTION_BITS   16


BLEUIntConfigItem pwm_correction_gamma("Gamma", 22);  // Gamma multiplied by ten, as integer value. E.g. value "22" is a gamma of 2.2

/*
 * Runtime configurables and variables
 */
DmxNowDmxGadget gadget("RGBNow", BOARD, 0);

// Pre-create a dimming curve
uint16_t incandescent_curve[256];
uint8_t dmx_data[sizeof(LED_PINS)/sizeof(LED_PINS[0])];

/*
 * Main functions
 */
void setup() {
  gadget.setup({&pwm_correction_gamma});

  for (int i=0; i<(sizeof(LED_PINS)/sizeof(LED_PINS[0])); i++) {
    ledcAttach(LED_PINS[i], PWM_FREQ, PWM_RESOLUTION_BITS);
  }
  
  Serial.printf("Using Gamma value of %f\n", ((double)pwm_correction_gamma.value()/10));
  for (int i = 0; i < 256; i++) {
    double linear_in = (double)i / 255.0;
    double corrected_out = pow(linear_in, ((double)pwm_correction_gamma.value()/10)) * 65535.0;
    incandescent_curve[i] = (uint16_t)round(corrected_out);
    Serial.printf("%03d->%05d ", i, incandescent_curve[i]);
    if ((i+1) % 8 == 0) {
      Serial.printf("\n");
    }
  }
}

void loop() {
  gadget.loop();

  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);
  for (int i=0; i<(sizeof(LED_PINS)/sizeof(LED_PINS[0])); i++) {
    ledcWrite(LED_PINS[i], incandescent_curve[dmx_data[i]]);
  }
  delay(25);
}