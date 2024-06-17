#include <dmxGadget.h>

dmxGadget::dmxGadget(char* name, unsigned int led_count, unsigned int defaultDmxAddress):
  receiver(RF24_PIN_CE, RF24_PIN_CSN, STATUS_LED_PIN),
  config(name),
  strip(led_count, LED_PIN, LED_CONFIG),
  battery(ADC_PIN),
  dmxAddress("dmx_address", defaultDmxAddress)
{
};

void dmxGadget::setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting up!");

  // Enable WDT
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = 0x03,    // Bitmask of all cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&twdt_config);

  // Bring up the status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  // Start config
  config.begin();
  config.addItem(dmxAddress);
  //config.advertise();

  // Start the receiver
  receiver.debug = true;
  receiver.begin();

  // Start the strip
  if (strip.numPixels() > 0) {
    strip.begin();
    strip.show();
  } else {
    Serial.println("NeoPixel strip is zero length, not initializing");
  }

  // Power PropMaker wing NeoPixel circuit
  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_POWER, HIGH);
}

void dmxGadget::loop() {
  outputLoopCount++;

  unsigned long currentTime = millis();
  if (currentTime > (lastStatusMillis + 1000)) {
    unsigned int rxCount = receiver.rxCount();
    unsigned int rxErrors = receiver.rxErrors();
    Serial.printf("Addr %d, Uptime: %d, RxCount: %d (avg %.2f/sec), errCount %d, loop iterations %d (avg %.2f/sec), Bat Voltage: %.2fV (%d%%), BLE active %d, connected: %d\n", dmxAddress.value(), currentTime/1000, rxCount, ((float)rxCount/currentTime*1000), rxErrors, outputLoopCount, ((float)outputLoopCount/currentTime*1000), battery.getBatteryVolts(), battery.getBatteryChargeLevel(), config.active(), BLE.connected());
    lastStatusMillis = currentTime;
  }

  if ((millis() > (bleConfigDisableSeconds * 1000)) && (config.active())) {
    Serial.printf("Disabling Bluetooth configuration\n");
    config.end();
  }
}