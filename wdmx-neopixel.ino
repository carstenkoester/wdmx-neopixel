// Radio code from https://juskihackery.wordpress.com/2021/01/31/how-the-cheap-wireless-dmx-boards-use-the-nrf24l01-protocol/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

#define DMX_BUFSIZE 512      // Total number of channels in a DMX universe
#define WDMX_PAYLOAD_SIZE 32 // Payload size in the NRF24L01 protocol
#define WDMX_HEADER_SIZE 4   // Header size in the NRF24L01 protocol
#define WDMX_MAGIC 128       // Magic number expected in byte 0 of every receive packet

const float MAX_ANALOG_VAL = 4095.0;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2


#define VERBOSE

#define RF24_PIN_CE    25    // GPIO connected to nRF24L01 CE pin (module pin 3)
#define RF24_PIN_CSN    4    // GPIO connected to nRF24L01 CE pin (module pin 4)
#define LED_POWER      33    // Power pin that needs to be pulled HIGH. Applicable to Arduino PropMaker.
#define LED_PIN        14    // GPIO connected to LED Data
#define LED_COUNT      150   // How many LED pixels are attached to the Arduino?
#define LED_CONFIG     NEO_RGB + NEO_KHZ800
#define BAT_VOLT_PIN   A13
#define STATUS_LED_PIN 13

static bool gotLock = false; // Global variable tracking if we have locked an RF channel


struct wdmxReceiveBuffer {
  uint8_t magic; // Always WDMX_MAGIC
  uint8_t payloadID;
  uint16_t highestChannelID; // Highest channel ID in the universe (not necessarily in this packet). Basically, highestChannelID + 1 = numChannels.
  uint8_t dmxData[WDMX_PAYLOAD_SIZE-WDMX_HEADER_SIZE];
};

RF24 radio(RF24_PIN_CE, RF24_PIN_CSN);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, LED_CONFIG);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

/* Unit IDs
    1: Red
    2: Green
    4: Blue
    5: Purple
    7: Cyan    
 */

static const uint32_t WAIT_MASKS[19] = {0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff, 0x01ff, 0x03ff, 0x07ff, 0x0fff, 0x1fff, 0x3fff, 0x7fff, 0xffff, 0x1ffff, 0x3ffff, 0x7ffff};

int dmx_start = 225; // DMX start address, zero-based
uint16_t dmx_channels = 25; // Number of DMX clannels to use. If we hav more LEDs than channels, wrap over.

static unsigned long lastPayloadTime = 0;
static unsigned int scanIterations = 0;
static unsigned int rxCount = 0;
static uint8_t dmxBuf[DMX_BUFSIZE];

void setup() { 
  #ifdef VERBOSE
  Serial.begin(115200);
  #endif
  
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
  radio.setAutoAck(false);
  radio.setPayloadSize(WDMX_PAYLOAD_SIZE);
  radio.setChannel(0);

  // NeoPixel setup
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP

  // Power PropMaker wing NeoPixel circuit
  pinMode(LED_POWER, OUTPUT);
  
  digitalWrite(LED_POWER, HIGH); // Turn on power pin for PropMaker NeoPixel output

  clearDmx();

  // Bring up the status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(LED_POWER, HIGH); // Turn on power pin for PropMaker NeoPixel output
}

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

void doScan(int unitID) {
  wdmxReceiveBuffer rxBuf;

  for (int rfCH = 0; rfCH < 126; rfCH++) {  
    delay(1); 
    radio.flush_rx();
    radio.openReadingPipe(0, getAddress(unitID, rfCH));
    radio.startListening();
    radio.setChannel(rfCH);
    #ifdef VERBOSE
    Serial.printf("Iteration %d, trying channel %d (%d), unit ID %d\n", scanIterations, radio.getChannel(), rfCH, unitID);
    #endif

    unsigned long started_waiting_at = micros(); // timeout setup
    bool timeout = false; 
    while (!radio.available()) {                             // While nothing is received
      if (micros() - started_waiting_at > 10000) {           // If waited longer than 10ms, indicate timeout and exit while loop
         timeout = true;
         break;
      }     
    }

    if (!timeout ){  
      radio.read(&rxBuf, sizeof(rxBuf));
      if (rxBuf.magic == WDMX_MAGIC) {
        gotLock = true;
        scanIterations = 0;
        #ifdef VERBOSE
        Serial.printf("Found a transmitter on channel %d, unit ID %d\n", rfCH, unitID);
        #endif
        break;
      }
    }
  }
}

void clearDmx() {
  memset(&dmxBuf, 0x00, sizeof(dmxBuf)); // Clear DMX buffer  
  showPixels();
}

inline void showPixels() {
  for(int i=0; i<strip.numPixels(); i++) {
    int startAddress = dmx_start+((i%dmx_channels)*3);
    strip.setPixelColor(i, (uint32_t) (dmxBuf[startAddress] << 16 | dmxBuf[startAddress+1] << 8 | dmxBuf[startAddress+2]));
  }
  strip.show();
}

void loop() {
  wdmxReceiveBuffer rxBuf;

  if (!gotLock) {
    // Bring up the status LED
    digitalWrite(LED_POWER, HIGH); // Turn on power pin for PropMaker NeoPixel output

    // Tbd: Should we clear the DMX buffer when we lost the transmitter? Should we make this configurable?
    clearDmx();

    for (int unitID = 1; unitID < 8; unitID++) {
      Serial.printf("Scanning for Unit ID %d\n", unitID);
      doScan(unitID);
      if (gotLock) {
        lastPayloadTime = millis();
        break;
      }
    }

    scanIterations++;
  }

  while (radio.available()) {
    /*
     * Read DMX values from radio.
     *
     * Note that buffers received aren't necessarily in numerical order, ie. we don't always receive the payload with channels 1-28, then 29-56, then 57-84 etc. We may be receiving
     * 29-56, then 1-28. then 57-84. In order to avoid unexpected behavior, we will wait for a complete DMX universe (having received every buffer at least one) before processing the
     * DMX buffer.
     */
    uint32_t waitMask = 0xffffffff;
  
//    while ((waitMask != 0x00000000) && (millis() - lastPayloadTime <= 1000)) {
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

//      // Glean how many packets we were expecting. Reset the "wait mask" for any packets that we weren't expecting in the first place.
//      waitMask &= WAIT_MASKS[(rxBuf.highestChannelID/sizeof(rxBuf.dmxData))];

//      // Reset the wait mask for the packet we just received
//      waitMask &= ~(1 << rxBuf.payloadID);
//    }

    /*
     * Output to DMX
     */
    showPixels();

    // Remove me
    if (rxCount % 1000 == 0) {
      int rawValue = analogRead(BAT_VOLT_PIN);
      float voltageLevel = (rawValue / MAX_ANALOG_VAL) * 2 * 1.1 * 3.3; // calculate voltage level
      float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  
      Serial.printf("RxCount: %d Bat Raw: %d Voltage: %fV Percent: %.2f%%\n", rxCount, rawValue, voltageLevel, (batteryFraction * 100));
    }

    // Blink status LED when we're receiving
    if (rxCount % 100 == 0) {
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
  }

  if (millis() - lastPayloadTime > 5000) {
    gotLock = false;
    Serial.println ("Not received a payload for > 5s");
  }

  if ((lastPayloadTime != 0) && (millis() - lastPayloadTime > 600000) || (lastPayloadTime == 0 && scanIterations > 10)) {
    Serial.println("Not received a payload for > 60 seconds, restarting");
    ESP.restart();
  }
} //end of main loop