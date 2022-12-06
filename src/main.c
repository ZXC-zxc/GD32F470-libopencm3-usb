#include "msc_usb.h"

extern void usbLoop(void);
extern void usbInit(void);
extern void usbPoll(void);
extern int led_test(void);

void firmware_usbLoop(void) {
  usbInit();
  for (;;) {
    usbPoll();
  }
}

int main(void) {
  // led_test();
  usb_gpio_init();
  // usbLoop();
  // mscLoop();
  firmware_usbLoop();
  return 0;
}
