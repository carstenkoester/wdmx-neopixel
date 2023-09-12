// Radio code from https://juskihackery.wordpress.com/2021/01/31/how-the-cheap-wireless-dmx-boards-use-the-nrf24l01-protocol/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>

/*
 * Compile-time configurables
 */
#define VERBOSE                             // Enable serial logging

#define DMX_BUFSIZE                   512   // Total number of channels in a DMX universe
#define WDMX_PAYLOAD_SIZE              32   // Payload size in the NRF24L01 protocol
#define WDMX_HEADER_SIZE                4   // Header size in the NRF24L01 protocol
#define WDMX_MAGIC                    128   // Magic number expected in byte 0 of every receive packet
#define WDT_TIMEOUT                    60   // 60 seconds watchdog timeout

#define RF24_PIN_CE                    25   // GPIO connected to nRF24L01 CE pin (module pin 3)
#define RF24_PIN_CSN                    4   // GPIO connected to nRF24L01 CSN pin (module pin 4)
#define RF24_PIN_IRQ                   21   // GPIO connected to nRF24L01 IRQ pin (module pin 8)
#define LED_POWER                      33   // Power pin that needs to be pulled HIGH. Applicable to Arduino PropMaker.
#define LED_PIN                        14   // GPIO connected to Neopixel LED Data
#define LED_COUNT                     150   // How many LED pixels are attached to the Arduino?
#define LED_CONFIG   NEO_RGB + NEO_KHZ800   // NEO_GRB / NEO_RGB / NEO_RGBW
#define BAT_VOLT_PIN                  A13   // Battery voltage measure pin
#define STATUS_LED_PIN                 13   // Status indicator LED pin

#define MAX_ANALOG_VAL             4095.0
#define MAX_BATTERY_VOLTAGE           4.2   // Max LiPoly voltage of a 3.7 battery is 4.2

/*
 * Runtime configurables
 */
int dmx_start = 0;           // DMX start address, zero-based
uint16_t dmx_channels = 25;  // Number of DMX clannels to use. If we hav more LEDs than channels, wrap over.

/*
 * Runtime variables
 */
struct wdmxReceiveBuffer {
  uint8_t magic; // Always WDMX_MAGIC
  uint8_t payloadID;
  uint16_t highestChannelID; // Highest channel ID in the universe (not necessarily in this packet). Basically, highestChannelID + 1 = numChannels.
  uint8_t dmxData[WDMX_PAYLOAD_SIZE-WDMX_HEADER_SIZE];
};

TaskHandle_t neopixelOutputTask;

RF24 radio(RF24_PIN_CE, RF24_PIN_CSN);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, LED_CONFIG);

static unsigned long lastPayloadTime = 0;
static unsigned int rxCount = 0;
static uint8_t dmxBuf[DMX_BUFSIZE];

static unsigned long lastRxCount;


uint64_t getAddress(int unitID, int channelID) {
  union wdmxAddress {
    struct {
      uint8_t channel;      // Channel ID
      uint8_t unitID;       // Unit ID
      uint8_t notChannel;   // Bitwise complement (bitwise NOT) of the Channel ID
      uint8_t notUnitID;    // Bitwise complement (bitwise NOT) of the unit ID
      uint8_t sum;          // Algebraic sum of Unit ID and Channel ID
    } structured;
    uint64_t uint64;
  } address;

  address.structured.channel = channelID;
  address.structured.unitID = unitID;
  address.structured.notChannel = ~channelID;
  address.structured.notUnitID = ~unitID;
  address.structured.sum = channelID + unitID;

  return address.uint64;
}

bool doScan(int unitID) {
  /* Unit IDs
      1: Red
      2: Green
      4: Blue
      5: Purple
      7: Cyan    
   */
  wdmxReceiveBuffer rxBuf;

  for (int rfCH = 0; rfCH < 126; rfCH++) {  
    delay(1);
    if (rfCH % 16 == 0) {
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); // Blink status LED while scanning - this will flash quickly
    }

    radio.flush_rx();
    radio.openReadingPipe(0, getAddress(unitID, rfCH));
    radio.startListening();
    radio.setChannel(rfCH);
    #ifdef VERBOSE
    Serial.printf("Trying channel %d (%d), unit ID %d\n", radio.getChannel(), rfCH, unitID);
    #endif

    unsigned long started_waiting_at = micros(); // timeout setup
    bool timeout = false; 
    while (!radio.available()) {                             // While nothing is received
      if (micros() - started_waiting_at > 10000) {           // If waited longer than 10ms, indicate timeout and exit while loop
         timeout = true;
         break;
      }     
    }

    if (!timeout) {  
      radio.read(&rxBuf, sizeof(rxBuf));
      if (rxBuf.magic == WDMX_MAGIC) {
        #ifdef VERBOSE
        Serial.printf("Found a transmitter on channel %d, unit ID %d\n", rfCH, unitID);
        #endif
        return(true);
      }
    }
  }
  return(false);
}

void clearDmx() {
  memset(&dmxBuf, 0x00, sizeof(dmxBuf)); // Clear DMX buffer  
}

void neopixelOutputLoop(void * parameters) {
  for(;;) {
    unsigned long t1 = millis();
    for(int i=0; i<strip.numPixels(); i++) {
      int startAddress = dmx_start+((i%dmx_channels)*3);
      strip.setPixelColor(i, (uint32_t) (dmxBuf[startAddress] << 16 | dmxBuf[startAddress+1] << 8 | dmxBuf[startAddress+2]));
    }
    strip.show();
    delay(25);
  }
}

void rxPacket() {
  wdmxReceiveBuffer rxBuf;

  while (radio.available()) {
    /*
     * Read DMX values from radio.
     */
  
    radio.read(&rxBuf, sizeof(rxBuf));
    if (rxBuf.magic != WDMX_MAGIC) {
      // Received packet with unexpected magic number. Ignore.
      continue;
    }

    lastPayloadTime = millis();

    rxCount++;

    int dmxChanStart = rxBuf.payloadID * sizeof(rxBuf.dmxData);

    // put payload into dmx buffer. If data goes beyond 512 channels, wrap over.
    memcpy(&dmxBuf[dmxChanStart], &rxBuf.dmxData, min(sizeof(rxBuf.dmxData), sizeof(dmxBuf)-dmxChanStart));
    if (dmxChanStart+sizeof(rxBuf.dmxData) > sizeof(dmxBuf)) {
      memcpy(&dmxBuf, &rxBuf.dmxData[sizeof(dmxBuf)-dmxChanStart], dmxChanStart+sizeof(rxBuf.dmxData)-sizeof(dmxBuf));
    }
  }

  // Pulse status LED when we're receiving
  if ((rxCount / 256) % 2) {
    analogWrite(STATUS_LED_PIN, rxCount % 256);
  } else {
    analogWrite(STATUS_LED_PIN, 255-(rxCount % 256));
  }
}

void setup() { 
  #ifdef VERBOSE
  Serial.begin(115200);
  #endif

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               //add current thread to WDT watch  
  
  if (!radio.begin()){
    #ifdef VERBOSE
    Serial.println("ERROR: failed to start radio");
    #endif
  }
  delay(100);
  #ifdef VERBOSE
  Serial.println("Starting up!");
  #endif
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(false);
  radio.setPayloadSize(WDMX_PAYLOAD_SIZE);

  pinMode(STATUS_LED_PIN, OUTPUT); // (Re)-set status LED for blinking

  bool gotLock;

  for (;;) {
    for (int unitID = 1; unitID < 8; unitID++) {
      Serial.printf("Scanning for Unit ID %d\n", unitID);
      gotLock = doScan(unitID);
      if (gotLock) {
        lastPayloadTime = millis();
        break;
      }
    }
    if (gotLock) {
      break;
    }
  }

  Serial.printf("Got lock\n");
  radio.maskIRQ(1,1,0);
  attachInterrupt(RF24_PIN_IRQ, rxPacket, FALLING);

  // NeoPixel setup
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP

  // Power PropMaker wing NeoPixel circuit
  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_POWER, HIGH);

  clearDmx();

  // Bring up the status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  // Start the output task
  xTaskCreatePinnedToCore(
    neopixelOutputLoop,   /* Function to implement the task */
    "NeopixelOutputTask", /* Name of the task */
    10000,                /* Stack size in words */
    NULL,                 /* Task input parameter */
    0,                    /* Priority of the task */
    &neopixelOutputTask,  /* Task handle. */
    0                     /* Core where the task should run */
  );
}

void loop() {
  if (rxCount > lastRxCount) {
    lastRxCount = rxCount;
    esp_task_wdt_reset();
  }

  int rawValue = analogRead(BAT_VOLT_PIN);
  float voltageLevel = (rawValue / MAX_ANALOG_VAL) * 2 * 1.1 * 3.3; // calculate voltage level
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  Serial.printf("RxCount: %d (avg %d/sec), Bat Raw: %d Voltage: %fV Percent: %.2f%%\n", rxCount, (rxCount/millis()*1000), rawValue, voltageLevel, (batteryFraction * 100));

  delay(1000);
}