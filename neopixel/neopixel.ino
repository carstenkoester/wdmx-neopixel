#include <dmxGadget.h>

/*
 * Compile-time configurables
 */
#define LED_COUNT                     100  // How many LED pixels are attached to the Arduino?

/*
 * Runtime configurables
 */

dmxGadget gadget("RGBStrip2", 100);

struct dmxData {
  uint8_t intensity;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t macro;
} dmx_data;

unsigned long outputLoopCount = 0;
uint16_t firstPixelHue = 0;

void setup() { 
  gadget.setup();
  gadget.config.advertise();
}

void loop() {
  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);

  if (dmx_data.macro < 128) {
    for(unsigned int i=0; i<gadget.strip.numPixels(); i++) {
      gadget.strip.setPixelColor(i, ((dmx_data.red * dmx_data.intensity)/255), ((dmx_data.green * dmx_data.intensity)/255), ((dmx_data.blue * dmx_data.intensity)/255));
    }
    gadget.strip.show();
    delay(25);
  } else {
    gadget.strip.rainbow(firstPixelHue, 1, 255, dmx_data.intensity, false);
    gadget.strip.show();
    firstPixelHue += 256; // This is a 16-bit unsigned integer so will wrap at 65535
    delay(255-dmx_data.macro);
  }

  outputLoopCount++;

  // Print a status on every 40th iteration (ie. every 40*25msec = 1000msec = 1 sec).
  if (outputLoopCount % 40 == 0) {
    float voltageLevel = (analogRead(BAT_VOLT_PIN) / MAX_ANALOG_VAL) * 2 * 1.1 * 3.3; // calculate voltage level
    float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
    unsigned int rxCount = gadget.receiver.rxCount();
    unsigned int rxErrors = gadget.receiver.rxErrors();
    Serial.printf("Addr %d, Uptime: %d, RxCount: %d (avg %f/sec), errCount %d, Bat Voltage: %fV Percent: %.2f%%, BLE active %d, connected: %d\n", gadget.dmxAddress.value(), millis()/1000, rxCount, ((float)rxCount/millis()*1000), rxErrors, voltageLevel, (batteryFraction * 100), gadget.config.active(), BLE.connected());

    if ((millis() > 300000) && (gadget.config.active())) {
      Serial.printf("Disabling Bluetooth configuration\n");
      gadget.config.end();
    }
  }
}