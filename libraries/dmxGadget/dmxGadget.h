#ifndef dmxGadget_h
#define dmxGadget_h

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <WirelessDMXReceiver.h>
#include <BLEConfig.h>
#include <Battery18650Stats.h>

#define WDT_TIMEOUT                    60   // 60 seconds watchdog timeout

#define RF24_PIN_CE                    25   // GPIO connected to nRF24L01 CE pin (module pin 3)
#define RF24_PIN_CSN                    4   // GPIO connected to nRF24L01 CSN pin (module pin 4)

#define LED_POWER                      33   // Power pin that needs to be pulled HIGH. Applicable to Arduino PropMaker.
#define LED_PIN                        14   // GPIO connected to Neopixel LED Data
#define LED_CONFIG   NEO_RGB + NEO_KHZ800   // NEO_GRB / NEO_RGB / NEO_RGBW

#define BAT_VOLT_PIN                  A13   // Battery voltage measure pin
#define STATUS_LED_PIN                 13   // Status indicator LED pin
#define MAX_ANALOG_VAL             4095.0
#define MAX_BATTERY_VOLTAGE           4.2   // Max LiPoly voltage of a 3.7 battery is 4.2

#define ADC_PIN                        35

class dmxGadget
{
  public:
    dmxGadget(char* name, unsigned int led_count, unsigned int defaultDmxAddress=1);
    void setup();
    void loop();

    WirelessDMXReceiver receiver;
    BLEConfig config;
    Adafruit_NeoPixel strip;
    Battery18650Stats battery;

    BLEUIntConfigItem dmxAddress;

    unsigned int bleConfigDisableSeconds = 30;

  private:
    unsigned long outputLoopCount = 0;
    unsigned long lastStatusMillis = 0;
};
#endif