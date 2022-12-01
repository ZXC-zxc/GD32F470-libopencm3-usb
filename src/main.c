#include "msc_usb.h"

extern void usbLoop(void);
extern void usbInit(void);
extern void usbPoll(void);

void firmware_usbLoop(void) {
  usbInit();
  for (;;) {
    usbPoll();
  }
}

int main(void) {
  usb_gpio_init();
  usbLoop();
  // mscLoop();
  // firmware_usbLoop();
  return 0;
}