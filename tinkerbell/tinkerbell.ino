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
#define LED_POWER                      33   // Power pin that needs to be pulled HIGH. Applicable to Arduino PropMaker.
#define LED_PIN                        14   // GPIO connected to Neopixel LED Data
#define LED_COUNT                     100   // How many LED pixels are attached to the Arduino?
#define LED_CONFIG   NEO_RGB + NEO_KHZ800   // NEO_GRB / NEO_RGB / NEO_RGBW
#define BAT_VOLT_PIN                  A13   // Battery voltage measure pin
#define STATUS_LED_PIN                 13   // Status indicator LED pin
#define MAX_ANALOG_VAL             4095.0
#define MAX_BATTERY_VOLTAGE           4.2   // Max LiPoly voltage of a 3.7 battery is 4.2

enum UnitID {
  RED = 1,
  GREEN = 2,
  BLUE = 4,
  PURPLE = 5,
  CYAN = 7
};
#define UNIT_ID                      BLUE

#define BOUNCE_DWELL_MIN 25
#define BOUNCE_DWELL_MAX 200

/*
 * Runtime configurables
 */
int dmx_start = 490;           // DMX start address 1-512

/*
 * Structs and forward declarations
 */
struct wdmxReceiveBuffer {
  uint8_t magic; // Always WDMX_MAGIC
  uint8_t payloadID;
  uint16_t highestChannelID; // Highest channel ID in the universe (not necessarily in this packet). Basically, highestChannelID + 1 = numChannels.
  uint8_t dmxData[WDMX_PAYLOAD_SIZE-WDMX_HEADER_SIZE];
};

TaskHandle_t neopixelOutputTask;
void neopixelOutputLoop(void * parameters);

struct dmxData {
  uint8_t intensity;
  uint8_t mode;
  uint8_t start;
  uint8_t end;
  uint8_t bounce;
  uint8_t speed;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

enum dmxMode {
  MODE_FORWARD,
  MODE_FORWARD_BOUNCE,
  MODE_REVERSE,
  MODE_REVERSE_BOUNCE,
  MODE_UNINITIALIZED
};

/*
 * Runtime variables
 */
RF24 radio(RF24_PIN_CE, RF24_PIN_CSN);

static unsigned int rxCount = 0;
static unsigned int rxErrCount = 0;
static uint8_t dmxBuf[DMX_BUFSIZE];


/*
 * Helper function to convert a (Unit ID, Channel ID) tuple into the
 * RF24 header structure that is used for this tuple.
 *
 * Ref https://juskihackery.wordpress.com/2021/01/31/how-the-cheap-wireless-dmx-boards-use-the-nrf24l01-protocol/
 * for a description of these values.
 *
 */
uint64_t getAddress(UnitID unitID, int channelID) {
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

/*
 * Given a Unit ID, probe all channels to see if we're receiving data for this unit ID
 */
bool doScan(UnitID unitID) {
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
  //radio.setPALevel(RF24_PA_MAX);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(false);
  radio.setPayloadSize(WDMX_PAYLOAD_SIZE);

  pinMode(STATUS_LED_PIN, OUTPUT); // (Re)-set status LED for blinking

  bool gotLock;

  for (;;) {
    Serial.printf("Scanning for Unit ID %d\n", UNIT_ID);
    gotLock = doScan(UNIT_ID);
    if (gotLock) {
      break;
    }
  }

  Serial.printf("Got lock\n");

  // Power PropMaker wing NeoPixel circuit
  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_POWER, HIGH);

  // Clear the DMX buffer
  memset(&dmxBuf, 0x00, sizeof(dmxBuf)); // Clear DMX buffer

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
  wdmxReceiveBuffer rxBuf;

  while (radio.available()) {
    /*
     * Read DMX values from radio.
     */

    radio.read(&rxBuf, sizeof(rxBuf));
    if (rxBuf.magic != WDMX_MAGIC) {
      // Received packet with unexpected magic number. Ignore.
      rxErrCount++;
      continue;
    }

    rxCount++;
    esp_task_wdt_reset();

    int dmxChanStart = rxBuf.payloadID * sizeof(rxBuf.dmxData);

    // put payload into dmx buffer. If data goes beyond 512 channels, wrap over.
    memcpy(&dmxBuf[dmxChanStart], &rxBuf.dmxData, min(sizeof(rxBuf.dmxData), sizeof(dmxBuf)-dmxChanStart));
    if (dmxChanStart+sizeof(rxBuf.dmxData) > sizeof(dmxBuf)) {
      memcpy(&dmxBuf, &rxBuf.dmxData[sizeof(dmxBuf)-dmxChanStart], dmxChanStart+sizeof(rxBuf.dmxData)-sizeof(dmxBuf));
    }

    // Pulse status LED when we're receiving
    if ((rxCount / 1024) % 2) {
      analogWrite(STATUS_LED_PIN, (rxCount % 1024)/4);
    } else {
      analogWrite(STATUS_LED_PIN, 255-((rxCount % 1024)/4));
    }
  }
}

void neopixelOutputLoop(void * parameters) {
  Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, LED_CONFIG);
  volatile dmxData *dmx_data;
  unsigned int highest_pixel = strip.numPixels()-1;

  int current_position = 0;
  int target_position = 0;
  uint8_t previous_intensity = 0;
  uint8_t current_intensity = 0;
  dmxMode previous_mode = MODE_UNINITIALIZED;
  dmxMode current_mode = MODE_UNINITIALIZED;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  int start;
  int end;
  int bounce;

  enum State {
    RUNNING,
    BOUNCING,
    OFF
  } state;
  state = RUNNING;

  // NeoPixel setup
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels before starting loop

  unsigned long output_loop_count = 0;
  dmx_data = (dmxData*) &dmxBuf[dmx_start-1];

  for(;;) {
    switch (dmx_data->mode) {
      case 0 ... 63:
        current_mode = MODE_FORWARD;
        break;
      case 64 ... 127:
        current_mode = MODE_FORWARD_BOUNCE;
        break;
      case 128 ... 191:
        current_mode = MODE_REVERSE;
        break;
      case 192 ... 255:
        current_mode = MODE_REVERSE_BOUNCE;
        break;
    }
    start = ((strip.numPixels()-1) * dmx_data->start) / 255;
    end = ((strip.numPixels()-1) * dmx_data->end) / 255;
    bounce = ((strip.numPixels()-1) * dmx_data->bounce) / 255;
    current_intensity = dmx_data->intensity;

    if (((previous_intensity == 0) && (current_intensity != 0)) || (previous_mode != current_mode)) {
      // We have shifted from zero intensity to non-zero intensity
      if ((current_mode == MODE_FORWARD) || (current_mode == MODE_FORWARD_BOUNCE)) {
        current_position = start;
        target_position = end;
      } else {
        current_position = end;
        target_position = start;
      }
      state = RUNNING;
    }

    uint8_t red = ((current_intensity * dmx_data->red) / 255);
    uint8_t green = ((current_intensity * dmx_data->green) / 255);
    uint8_t blue = ((current_intensity * dmx_data->blue) / 255);
  
    for(unsigned int i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, 0, 0, 0);
    }
    if ((state == RUNNING) || (state == BOUNCING)) {
      strip.setPixelColor(current_position, red, green, blue);
    }
    strip.show();
    // Serial.printf("Loop %d position %d target %d previous_intensity %d current_intensity %d start %d end %d mode %d state %d\n", output_loop_count, current_position, target_position, previous_intensity, current_intensity, start, end, current_mode, state);


    if (current_position == target_position) {
      if ((current_mode == MODE_FORWARD) || (current_mode == MODE_REVERSE)) {
        state = OFF;
      } else {
        state = BOUNCING;
        delay((rand() % (BOUNCE_DWELL_MAX - BOUNCE_DWELL_MIN + 1)) + BOUNCE_DWELL_MIN);
        if (current_mode == MODE_FORWARD_BOUNCE) {
          target_position = (rand() % (end - bounce + 1)) + bounce;
        } else {
          target_position = (rand() % (bounce - start + 1)) + start;
        }
      }
    } else if (current_position < target_position) {
      current_position++;
    } else if (current_position > target_position) {
      current_position--;
    }

    delay(1000-(dmx_data->speed*1000/255));

    output_loop_count++;
    previous_intensity = current_intensity;
    previous_mode = current_mode;

    // Print a status on every 40th iteration (ie. every 40*25msec = 1000msec = 1 sec).
    if (output_loop_count % 40 == 0) {
      float voltageLevel = (analogRead(BAT_VOLT_PIN) / MAX_ANALOG_VAL) * 2 * 1.1 * 3.3; // calculate voltage level
      float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
      Serial.printf("RxCount: %d (avg %f/sec), errCount %d, Bat Voltage: %fV Percent: %.2f%%\n", rxCount, ((float)rxCount/millis()*1000), rxErrCount, voltageLevel, (batteryFraction * 100));
    }
  }
}