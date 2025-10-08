#include <dmxGadget.h>
#include "esp_wifi.h"

#include "hal/gpio_ll.h"

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

/*
 * Compile-time configurables
 */
#define PWM_FREQ              4000
#define PWM_RESOLUTION_BITS   8

// GPIO pins that are used. Set these to match your board
const int NUM_ROWS = 6;
const int ROW_GPIO[NUM_ROWS] = {27, 33, 15, 32, 14, 26};

const int NUM_COLUMNS = 3;
const int COLUMN_GPIO[NUM_COLUMNS] = {25, 4, 21};

const int TILT_GPIO = 13;

// FLAME_PROBABLILITY defines a probability, in percent, that a row is "on"
// during a any given cycle.
const int FLAME_PROBABILITY[NUM_ROWS] = {98, 85, 55, 20, 10, 5};

// Amount of time that the output matrix shows an active dot (or column of dots)
// before cycling to the next time
const int DISPLAY_DWELL = 2;  // msec

// Timeout before we black out the bulb
const int DMX_TIMEOUT = 3;    //

/*
 * Runtime configurables and variables
 */
DmxNowDmxGadget gadget("Flame", DMXGADGET_BOARD_HUZZAH32_NO_STATUS, 0);
BLEUIntConfigItem pwm_correction_gamma("Gamma", 22);  // Gamma multiplied by ten, as integer value. E.g. value "22" is a gamma of 2.2

// Internal variables used to store output matrix state.
uint8_t currentMatrix[NUM_ROWS][NUM_COLUMNS], outputMatrix[NUM_ROWS][NUM_COLUMNS], targetMatrix[NUM_ROWS][NUM_COLUMNS] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
};
bool blackout = false;
unsigned int lastRefreshMillis = 0; // Timestamp of last flame refresh

// Output task
TaskHandle_t _flameOutput;

// Dimming curve used to translate linear DMX512 intensities to PWM output
uint16_t incandescent_curve[256];

// Data structure for received DMX data
struct dmxData {
  uint8_t intensity;
  uint8_t fadeSpeed;
  uint8_t flameRefreshSpeed;
} dmx_data;

unsigned long loopCount = 0;

/*
 * Subroutines
 */
void flameOutputTask(void*)
{
  // Run as infinite loop; rotate over all output columns and display any one column for DISPLAY_DWELL
  // milliseconds before moving to the next. If we do this fast enough, it'll (almost) give the illusion
  // of a permanently lit flame.
  bool tiltedUp;

  while (true)
  {
    tiltedUp = digitalRead(TILT_GPIO);
    for (int column = 0; column < NUM_COLUMNS; column++)
    {
      if (blackout) {
        digitalWrite(COLUMN_GPIO[column], LOW);
        delay(DISPLAY_DWELL);
        continue;
      }
      
      // Set the row outputs "in the blind" -- all columns are currently turned off.
      for (int row = 0; row < NUM_ROWS; row++)
      {
        // Adjust for tilt sensor
        ledcWrite(ROW_GPIO[(tiltedUp ? row : NUM_ROWS-1-row)], incandescent_curve[outputMatrix[row][column]]);
      }

      // Once the output is set for all rows in this column, display the output for a brief period of time.
      digitalWrite(COLUMN_GPIO[column], HIGH);
      delay(DISPLAY_DWELL);
      digitalWrite(COLUMN_GPIO[column], LOW);
    }
  }
}


/*
 * Main functions
 */
void setup() { 
  // Set the Column pins as digital output pins
  for (int column = 0; column < NUM_COLUMNS; column++) {
    pinMode(COLUMN_GPIO[column], OUTPUT);
    digitalWrite(COLUMN_GPIO[column], LOW);
  }

  // We use PWM for the rows. Note that we need to invert the GPIO pin for this board.
  for (int ledChannel = 0; ledChannel < NUM_ROWS; ledChannel++) {
    ledcAttach(ROW_GPIO[ledChannel], PWM_FREQ, PWM_RESOLUTION_BITS);
    GPIO.func_out_sel_cfg[ROW_GPIO[ledChannel]].inv_sel = 1;
    ledcWrite(ROW_GPIO[ledChannel], 0);
  }

  // Set the tilt switch as input
  pinMode(TILT_GPIO, INPUT_PULLUP);

  // DMXGadget setup
  gadget.setup({&pwm_correction_gamma});

  // Initialize the incandescent table based on gamma curve
  Serial.printf("Using Gamma value of %f\n", ((double)pwm_correction_gamma.value()/10));
  for (int i = 0; i < 256; i++) {
    double linear_in = (double)i / 255.0;
    double corrected_out = pow(linear_in, ((double)pwm_correction_gamma.value()/10)) * 256.0;
    incandescent_curve[i] = (uint16_t)round(corrected_out);
    Serial.printf("%03d->%05d ", i, incandescent_curve[i]);
    if ((i+1) % 8 == 0) {
      Serial.printf("\n");
    }
  }

  // Start the output task
  xTaskCreatePinnedToCore(
    flameOutputTask,        // Function to implement the task
    "Flame Output",         // Name of the task
    10000,                  // Stack size in words
    0,                      // Task input parameter
    5,                      // Priority of the task
    &_flameOutput,          // Task handle.
    1                       // Core where the task should run
  );
}


void loop() {
  gadget.loop();
  gadget.receiver.getValues(gadget.dmxAddress.value(), sizeof(dmx_data), &dmx_data);

  // Calculate loop delay and timings.
  //
  // Fade speed is how long we want any single transient state during a fade to rest. It is in units
  // of 25msec, where the maximum value (255 or 100%) means 25 msec, and the mininum value (0) means
  // 6.375 seconds. From this, we calculate the loop delay.
  //
  // Refresh interval is how often we build a new target picture for a flame. The user is meant to
  // indicate it in a value where 0 means every 10 seconds, and 255 (100%) means the fastest possible
  // way, which is at "Fade speed" (or at loop delay). We'll translate this into an internal variable
  // counted as "every n-th loop iteration".
  //
  // Step size the level of intensity adjustment made to go from one flame state to the next. This
  // is calculated by assuming an intensity delta of 256 (if a pixel was off on the current state
  // but is on in the next, or vice versa) and breaking that up into equal steps based on the
  // regresh interval.
  // 
  int loopDelay = (256*25) - (dmx_data.fadeSpeed * 25);
  int refreshInterval = ((10000 - (10000 * dmx_data.flameRefreshSpeed / 255)) / loopDelay) + 1;

  int _stepSize = 256/refreshInterval;
  if (_stepSize == 0) {
    _stepSize = 1;
  }

  DEBUG_PRINT("Loop iter %d delay %d step %d refresh interval %d time time %d (rate %d)\n", loopCount, loopDelay, _stepSize, refreshInterval, refreshInterval*loopDelay, 1000/(refreshInterval*loopDelay));

  /*
   * Check if we have a current DMX signal. If not, turn off.
   */
  if (millis() > gadget.last_receive_millis() + DMX_TIMEOUT*1000) {
    blackout = true;
    delay(loopDelay);
    return;
  }

  /*
   * If our intensity is at zero, just ensure all pins remain low. Otherwise, the bulb may flicker
   * and glow a tiny bit even when at zero.
   */
  if (dmx_data.intensity == 0) {
    blackout = true;
    delay(loopDelay);
    return;
  }

  blackout = false;

  if (loopCount % refreshInterval == 0)
  {
    DEBUG_PRINT("Refreshing on loop %d, every %d, time delta %d\n", loopCount, refreshInterval, millis()-lastRefreshMillis);
    /*
    * Generate a new flame matrix based on the flame probability values.
    */
    for (int column = 0; column < NUM_COLUMNS; column++)
    {
      for (int row = 0; row < NUM_ROWS; row++)
      {
        bool isOn = (FLAME_PROBABILITY[row] > random(100));

        targetMatrix[row][column] = (isOn ? 255 : 0);
      }
    }
    lastRefreshMillis = millis();
  }

  /*
   * Fade to the new matrix over FADE_STEPS steps
   */
  for (int row = 0; row < NUM_ROWS; row++)
  {
    for (int column = 0; column < NUM_COLUMNS; column++)
    {
      uint8_t currentValue = currentMatrix[row][column];
      uint8_t targetValue = targetMatrix[row][column];
      uint8_t newValue;

      DEBUG_PRINT(" | curr %03d next %03d out %03d", currentMatrix[row][column], targetMatrix[row][column], outputMatrix[row][column]);

      if (targetValue > currentValue)
      {
        // Going up
        if (currentValue > 255-_stepSize) {
          newValue = 255;
          DEBUG_PRINT("new %03d >> MAX", newValue);
        } else {
          newValue = currentValue + _stepSize;
          DEBUG_PRINT("new %03d + STEP", newValue);
        }
      } else {
        // Going down
        if (currentValue < _stepSize) {
          newValue = 0;
          DEBUG_PRINT("new %03d << MIN", newValue);
        } else {
          newValue = currentValue - _stepSize;
          DEBUG_PRINT("new %03d - STEP", newValue);
        }
      }

      currentMatrix[row][column] = newValue;
      outputMatrix[row][column] = newValue * dmx_data.intensity / 255;
    }
    DEBUG_PRINT("\n");
  }
  DEBUG_PRINT("\n");

  delay(loopDelay);
  loopCount++;
}