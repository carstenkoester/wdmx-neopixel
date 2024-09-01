#include <dmxGadget.h>

/*
 * Runtime configurables and variables
 */
dmxGadget gadget("Capture", 0);

void setup() {
  gadget.setup();
  gadget.config.advertise();
  gadget.receiver.startCapture();
}

void loop() {
  gadget.loop();

  if (gadget.receiver.isCaptureBufferFull()) {
    gadget.receiver.stopCapture();
    gadget.receiver.printCapture();
  }
}