/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013 Weston Schmidt <weston_schmidt@alumni.purdue.edu>
 * Copyright (C) 2013 Pavol Rusnak <stick@gk2.sk>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

// clang-format off
#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/msc.h>
#include "ramdisk.h"

#include "drv_usb_core.h"
#include "drv_usb_hw.h"
// #include "usbd_msc_core.h"

// clang-format on

static const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0110,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5741,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor msc_endp[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 0,
    },
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 0,
    }};

static const struct usb_interface_descriptor msc_iface[] = {
    {.bLength = USB_DT_INTERFACE_SIZE,
     .bDescriptorType = USB_DT_INTERFACE,
     .bInterfaceNumber = 0,
     .bAlternateSetting = 0,
     .bNumEndpoints = 2,
     .bInterfaceClass = USB_CLASS_MSC,
     .bInterfaceSubClass = USB_MSC_SUBCLASS_SCSI,
     .bInterfaceProtocol = USB_MSC_PROTOCOL_BBB,
     .iInterface = 0,
     .endpoint = msc_endp,
     .extra = NULL,
     .extralen = 0}};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = msc_iface,
}};

static const struct usb_config_descriptor config_descr = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,
    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Black Sphere Technologies",
    "MSC Demo",
    "DEMO",
};

static usbd_device *msc_dev;
/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

usb_core_driver msc_udisk;
unsigned char SRAM[40 * 1024];

// void gd32_usb_core_init(void) {
//   /* configure the GPIO */
//   usb_gpio_config();
//   /* configure the USB peripheral clock */
//   usb_rcu_config();
//   /* initialize the USB timer */
//   usb_timer_init();

//   usbd_core_init(&msc_udisk,
// #ifdef USE_USB_FS
//                  USB_CORE_ENUM_FS,
// #elif defined(USE_USB_HS)
//                  USB_CORE_ENUM_HS,
// #endif
//                  &msc_desc, &msc_class);
//   usb_intr_config();
// }

/*usb gpio config*/
void usb_gpio_init(void) {
  rcu_periph_clock_enable(RCU_SYSCFG);
  rcu_periph_clock_enable(RCU_GPIOA);
  /* USBFS_DM(PA11) and USBFS_DP(PA12) GPIO pin configuration */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11 | GPIO_PIN_12);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,
                          GPIO_PIN_11 | GPIO_PIN_12);
  gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_11 | GPIO_PIN_12);
}

int led_test(void) {
  /* configure systick */
  //   systick_config();

  /* enable the LEDs GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOE);
  rcu_periph_clock_enable(RCU_GPIOF);

  /* configure LED1 GPIO port */
  gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
  gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
  /* reset LED1 GPIO pin */
  gpio_bit_reset(GPIOE, GPIO_PIN_2);

  /* enable the LED2 GPIO clock */
  /* configure LED2 GPIO port */
  gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_3);
  gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
  /* reset LED2 GPIO pin */
  gpio_bit_reset(GPIOE, GPIO_PIN_3);

  /* enable the LED3 GPIO clock */
  /* configure LED3 GPIO port */
  gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10);
  gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
  /* reset LED3 GPIO pin */
  gpio_bit_reset(GPIOF, GPIO_PIN_10);

  while (1) {
    gpio_bit_set(GPIOF, GPIO_PIN_10);
    /* turn on LED1, turn off LED3 */
    gpio_bit_set(GPIOE, GPIO_PIN_2);
    gpio_bit_reset(GPIOF, GPIO_PIN_10);
    // delay_1ms(1000);

    /* turn on LED2, turn off LED1 */
    gpio_bit_set(GPIOE, GPIO_PIN_3);
    gpio_bit_reset(GPIOE, GPIO_PIN_2);
    // delay_1ms(1000);

    /* turn on LED3, turn off LED2 */
    gpio_bit_set(GPIOF, GPIO_PIN_10);
    gpio_bit_reset(GPIOE, GPIO_PIN_3);
    // delay_1ms(1000);
  }
}

int mscLoop(void) {
  msc_dev = usbd_init(&otgfs_usb_driver, &dev_descr, &config_descr, usb_strings,
                      3, usbd_control_buffer, sizeof(usbd_control_buffer));
  ramdisk_init();
  usb_msc_init(msc_dev, 0x82, 64, 0x01, 64, "VendorID", "ProductID", "0.00",
               ramdisk_blocks(), ramdisk_read, ramdisk_write);
  for (;;) {
    usbd_poll(msc_dev);
  }
}
