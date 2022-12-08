#include "msc_usb.h"
#include "mi2c.h"
#include "oled.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "gd32f4xx.h"

#define I2CX_OWN_ADDRESS7 0x20

extern void usbLoop(void);
extern void usbInit(void);
extern void usbPoll(void);
extern int led_test(void);
extern void spiLoop(void);

static void i2c_rcu_config(void) {
  /* enable GPIOB clock */
  rcu_periph_clock_enable(GPIO_MI2C_PORT);
  /* enable I2C0 clock */
  rcu_periph_clock_enable(RCU_I2C0);
}

static void i2c_gpio_config(void) {
  /* I2C0 GPIO ports */
  /* connect PB6 to I2C0_SCL */
  gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_6);
  /* connect PB7 to I2C0_SDA */
  gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_7);

  /* configure I2C0 GPIO */
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
}

static void i2c_config(void) {
  /* configure I2C clock */
  i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
  /* configure I2C address */
  i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS,
                       I2CX_OWN_ADDRESS7);
  /* enable I2C0 */
  i2c_enable(I2C0);
  /* enable acknowledge */
  i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

// mi2c driver loop
void mi2cLoop(void) {
  i2c_gpio_config();
  i2c_rcu_config();
  i2c_config();
  for (;;) {
  }
}
// firmware hid + winusb
void firmware_usbLoop(void) {
  usbInit();
  for (;;) {
    usbPoll();
  }
}

// static void _spi_init(uint32_t spi_periph, spi_parameter_struct *spi_struct)
// {
//   uint32_t reg = 0U;
//   reg = SPI_CTL0(spi_periph);
//   reg &= SPI_INIT_MASK;

//   /* select SPI as master or slave */
//   reg |= spi_struct->device_mode;
//   /* select SPI transfer mode */
//   reg |= spi_struct->trans_mode;
//   /* select SPI frame size */
//   reg |= spi_struct->frame_size;
//   /* select SPI nss use hardware or software */
//   reg |= spi_struct->nss;
//   /* select SPI LSB or MSB */
//   reg |= spi_struct->endian;
//   /* select SPI polarity and phase */
//   reg |= spi_struct->clock_polarity_phase;
//   /* select SPI prescale to adjust transmit speed */
//   reg |= spi_struct->prescale;

//   /* write to SPI_CTL0 register */
//   SPI_CTL0(spi_periph) = (uint32_t)reg;

//   SPI_I2SCTL(spi_periph) &= (uint32_t)(~SPI_I2SCTL_I2SSEL);
// }

extern void gd_spi_enable(uint32_t spi);
static spi_master_init(void) {
  // enable SPI 1 for OLED display
  // gd32f470 : PI1-SPI1_SCK、 PI2-SPI1_MISO、PI3-SPI1_MOSI
  // rcc_periph_clock_enable(RCC_GPIOI);
  // rcc_periph_clock_enable(RCC_SPI1);

  // gpio_set_af(GPIOB, GPIO_AF5, GPIO1 | GPIO2 | GPIO3);
  // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1 | GPIO2 |
  // GPIO3);
  // // gpio_set_output_options(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
  // //                         GPIO1 | GPIO2 | GPIO3);

  // gpio_mode_setup(GPIOI, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
  // gpio_set_output_options(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO0);

  // 使用gd driver
  rcu_periph_clock_enable(RCU_GPIOI);
  rcu_periph_clock_enable(RCU_SPI1);

  /* configure SPI1 GPIO */
  gpio_af_set(GPIOI, GPIO_AF_5, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  gpio_mode_set(GPIOI, GPIO_MODE_AF, GPIO_PUPD_NONE,
                GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  gpio_output_options_set(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                          GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

  /* set SPI1_NSS as GPIO*/
  gpio_mode_set(GPIOI, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
  gpio_output_options_set(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);

  spi_parameter_struct spi_init_struct;

  /* configure SPI1 parameter */
  spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
  spi_init_struct.nss = SPI_NSS_SOFT;
  spi_init_struct.prescale = SPI_PSC_64;
  spi_init_struct.endian = SPI_ENDIAN_MSB;
  spi_init(OLED_SPI1, &spi_init_struct);

  //	spi_disable_crc(SPI1);
  // spi_init_master(
  //     OLED_SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
  //     SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1,
  //     SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  // spi_enable_ss_output(SPI1);
  OLED_NSS_HIGH;
  //	spi_enable_software_slave_management(SPI1);
  //	spi_set_nss_high(SPI1);
  //	spi_clear_mode_fault(SPI1);
  spi_enable(OLED_SPI1);
}

uint32_t send_n = 0, receive_n = 0;
uint8_t spi1_send_array[10] = {0xAC, 0xA2, 0xA3, 0xA4, 0xA5,
                               0xA6, 0xA7, 0xA8, 0xA9, 0xAF};
int main(void) {
  // led_test();
  // usb_gpio_init();
  // usbLoop();
  // mscLoop();
  // firmware_usbLoop();

  spi_master_init();

  spiLoop();

  return 0;
}
