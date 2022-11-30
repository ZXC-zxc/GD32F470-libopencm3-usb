
/*
#include "gd32f427r_start.h"
#include "gd32f4xx.h"
#include "systick.h"


int main(void)
{
    gd_eval_led_init(LED1);
    
    systick_config();
    
    while(1) {
        gd_eval_led_on(LED1);
        delay_1ms(500);
        gd_eval_led_off(LED1);
        delay_1ms(500);
    }
}*/

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

static void gpio_setup(void) {
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
}

extern int msc_main(void);

int main(void) {
  int i;

  //   clock_setup();

  gpio_setup();

  msc_main();

  while (1) {
    gpio_toggle(GPIOC, GPIO6);
    for (i = 0; i < 3000000; i++) {
      __asm__("nop");
    }
  }
  return 0;
}