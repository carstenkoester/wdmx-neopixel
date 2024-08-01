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
//  gadget.receiver.capture = true;
}

inline uint8_t getPixelIntensity(uint8_t value)
{
  return ((value * dmx_data.intensity)/255);
}

void loop() {
  gadget.loop();

  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);
  if (dmx_data.macro < 128) {
    for(unsigned int i=0; i<gadget.strip.numPixels(); i++) {
      gadget.strip.setPixelColor(i, getPixelIntensity(dmx_data.red), getPixelIntensity(dmx_data.green), getPixelIntensity(dmx_data.blue));
    }
    gadget.strip.show();
    delay(25);
  } else {
    gadget.strip.rainbow(firstPixelHue, 1, 255, dmx_data.intensity, false);
    gadget.strip.show();
    firstPixelHue += 256; // This is a 16-bit unsigned integer so will wrap at 65535
    delay(255-dmx_data.macro);
  }

  if (gadget.receiver.capture && gadget.receiver.debugBuffer.isFull()) {
    Serial.println("Debug buffer is full");
    gadget.receiver.capture = false;

    WirelessDMXReceiver::wdmxReceiveBuffer buf;
    unsigned int i=0;
    Serial.println("popping...");
    while (gadget.receiver.debugBuffer.pop(buf)) {
      i++;
      Serial.printf("Pkt %04d Magic %02x Payload %02x (%d) HighestChannel %04x (%d), Data ", i, buf.magic, buf.payloadID, buf.payloadID, buf.highestChannelID, buf.highestChannelID);
      for (int j = 0; j < sizeof(buf.dmxData); j++) {
        Serial.printf("%02x ", buf.dmxData[j]);
      }
      Serial.printf("\n");
    }
    Serial.println("Done popping...");
  }
}