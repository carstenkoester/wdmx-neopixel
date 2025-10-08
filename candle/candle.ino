#include <dmxGadget.h>
#include "esp_wifi.h"

/*
 * Compile-time configurables
 */
#define LED_CH1               32

#define PWM_FREQ              100
#define PWM_RESOLUTION_BITS   16

BLEUIntConfigItem pwm_correction_gamma("Gamma", 22);  // Gamma multiplied by ten, as integer value. E.g. value "22" is a gamma of 2.2

/*
 * Runtime configurables and variables
 */
DmxNowDmxGadget gadget("CandleNow", DMXGADGET_BOARD_SPARKLE_MOTION_MINI, 0);

// Pre-create a dimming curve
uint16_t incandescent_curve[256];

/*
 * Main functions
 */
void setup() { 
  gadget.setup({&pwm_correction_gamma});

  ledcAttach(LED_CH1, PWM_FREQ, PWM_RESOLUTION_BITS);

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

  ledcWrite(LED_CH1, incandescent_curve[gadget.receiver.getValue(gadget.dmxAddress.value())]);
  delay(25);
}