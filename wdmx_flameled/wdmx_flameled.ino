#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>

#include <WirelessDMXReceiver.h>

/*
 * Compile-time configurables
 */
#define VERBOSE                             // Enable serial logging

#define WDT_TIMEOUT                    60   // 60 seconds watchdog timeout
#define RF24_PIN_CE                    25   // GPIO connected to nRF24L01 CE pin (module pin 3)
#define RF24_PIN_CSN                    4   // GPIO connected to nRF24L01 CSN pin (module pin 4)

#define LED_PIN                        33   // Pin that the LED is on

#define BAT_VOLT_PIN                  A13   // Battery voltage measure pin
#define STATUS_LED_PIN                 13   // Status indicator LED pin
#define MAX_ANALOG_VAL             4095.0
#define MAX_BATTERY_VOLTAGE           4.2   // Max LiPoly voltage of a 3.7 battery is 4.2

/*
 * Runtime configurables
 */
int dmx_address = 501;                        // DMX  address, 1-512

/*
 * Runtime variables
 */
WirelessDMXReceiver receiver(RF24_PIN_CE, RF24_PIN_CSN, STATUS_LED_PIN);

void setup() { 
  #ifdef VERBOSE
  Serial.begin(115200);
  #endif

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  
  delay(100);
  #ifdef VERBOSE
  Serial.println("Starting up!");
  #endif

  // Turn off LED while scanning
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(STATUS_LED_PIN, OUTPUT); // (Re)-set status LED for blinking

  // Bring up the status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  receiver.debug = true;
  receiver.begin();
}

void loop() {
  unsigned long outputLoopCount = 0;
  for(;;) {
    analogWrite(LED_PIN, receiver.dmxBuffer[dmx_address-1]);
    // analogWrite(LED_PIN, 255);
    outputLoopCount++;
    delay(25);

    // Print a status on every 40th iteration (ie. every 40*25msec = 1000msec = 1 sec).
    if (outputLoopCount % 40 == 0) {
      float voltageLevel = (analogRead(BAT_VOLT_PIN) / MAX_ANALOG_VAL) * 2 * 1.1 * 3.3; // calculate voltage level
      float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
      unsigned int rxCount = receiver.rxCount();
      unsigned int rxErrors = receiver.rxErrors();
      Serial.printf("RxCount: %d (avg %f/sec), errCount %d, Bat Voltage: %fV Percent: %.2f%%\n", rxCount, ((float)rxCount/millis()*1000), rxErrors, voltageLevel, (batteryFraction * 100));
    }
  }
}