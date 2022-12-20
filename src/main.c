#include "msc_usb.h"
#include "mi2c.h"
#include "oled.h"
#include "si2c.h"
#include "usart.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include "gd32f4xx.h"
#include "gd32f470i_eval.h"

#define I2CX_OWN_ADDRESS7 0x20

extern void usbLoop(void);
extern void usbInit(void);
extern void usbPoll(void);
extern int led_test(void);
extern void spiLoop(void);
void usartLoop(void);

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
  rcu_periph_clock_enable(RCU_GPIOH);
  rcu_periph_clock_enable(RCU_SPI1);

  /* configure OLED DC RST PIN*/
  gpio_mode_set(OLED_DC_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OLED_DC_PIN);
  gpio_mode_set(OLED_RST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OLED_RST_PIN);

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
  //	spi_enable_software_slave_manage  ment(SPI1);
  //	spi_set_nss_high(SPI1);
  //	spi_clear_mode_fault(SPI1);
  spi_enable(OLED_SPI1);
}

#define I2C0_SLAVE_ADDRESS7 0x82
#define I2C1_SLAVE_ADDRESS7 0x72

uint8_t i2c_buffer_transmitter[16];
uint8_t i2c_buffer_receiver[16];
volatile uint8_t *i2c_txbuffer;
volatile uint8_t *i2c_rxbuffer;
volatile uint16_t i2c_nbytes;
volatile ErrStatus status;
volatile ErrStatus state = ERROR;

/*!
    \brief      memory compare function
    \param[in]  src : source data
    \param[in]  dst : destination data
    \param[in]  length : the compare data length
    \param[out] none
    \retval     ErrStatus : ERROR or SUCCESS
*/
ErrStatus memory_compare(uint8_t *src, uint8_t *dst, uint16_t length) {
  while (length--) {
    if (*src++ != *dst++) {
      return ERROR;
    }
  }
  return SUCCESS;
}

void mi2c0_init(void) {
  /* enable GPIOB clock */
  rcu_periph_clock_enable(RCU_GPIOB);
  /* enable I2C0 clock */
  rcu_periph_clock_enable(RCU_I2C0);
  /* I2C0 and I2C1 GPIO ports */
  /* connect PB6 to I2C0_SCL */
  gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_6);
  /* connect PB7 to I2C0_SDA */
  gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_7);

  /* configure GPIO pins of I2C0 */
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

  /* I2C clock configure */
  i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
  /* I2C address configure */
  i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS,
                       I2C0_SLAVE_ADDRESS7);
  /* enable I2C0 */
  i2c_enable(I2C0);
  /* enable acknowledge */
  i2c_ack_config(I2C0, I2C_ACK_ENABLE);
  // i2c_clock_config(I2C1, 100000, I2C_DTCY_2);

  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
  nvic_irq_enable(I2C0_EV_IRQn, 0, 3);
  nvic_irq_enable(I2C0_ER_IRQn, 0, 2);
}

void si2c_init(void) {
  rcu_periph_clock_enable(RCU_I2C1);
  /* connect PB10 to I2C1_SCL */
  gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_10);
  /* connect PB11 to I2C1_SDA */
  gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_11);

  /* configure GPIO pins of I2C1 */
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

  i2c_clock_config(I2C1, 100000, I2C_DTCY_2);
  /* I2C address configure */
  i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS,
                       I2C1_SLAVE_ADDRESS7);
  /* enable I2C1 */
  i2c_enable(I2C1);
  /* enable acknowledge */
  i2c_ack_config(I2C1, I2C_ACK_ENABLE);

  nvic_irq_enable(I2C1_EV_IRQn, 0, 4);
  nvic_irq_enable(I2C1_ER_IRQn, 0, 1);
}

void si2c_libopencm3_init(void) {
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_GPIOB);

  i2c_reset(I2C1);
  gpio_set_af(GPIO_SI2C_PORT, GPIO_AF4, GPIO_SI2C_SCL | GPIO_SI2C_SDA);

  gpio_set_output_options(GPIO_SI2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  gpio_mode_setup(GPIO_SI2C_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                  GPIO_SI2C_SCL | GPIO_SI2C_SDA);

  i2c_set_fast_mode(I2C1);
  i2c_set_speed(I2C1, i2c_speed_fm_400k, 30);
  /* I2C address configure */
  i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS,
                       I2C1_SLAVE_ADDRESS7);

  // /*	//HSI is at 2Mhz */
  // // i2c_set_standard_mode(I2C1);
  // i2c_set_speed(I2C1, i2c_speed_sm_100k, 30);
  // /*	//addressing mode*/
  // i2c_set_own_7bit_slave_address(I2C1, I2C1_SLAVE_ADDRESS7);

  /* enable I2C1 */
  i2c_enable(I2C1);
  // i2c_peripheral_disable(I2C1);

  /* enable acknowledge */
  i2c_ack_config(I2C1, I2C_ACK_ENABLE);

  nvic_irq_enable(I2C1_EV_IRQn, 0, 4);
  nvic_irq_enable(I2C1_ER_IRQn, 0, 1);
}

void si2cLoop(void) {
  for (uint8_t i = 0; i < 16; i++) {
    i2c_buffer_transmitter[i] = i + 0xA0;
    i2c_buffer_receiver[i] = 0xFF;
  }
  /* initialize i2c_txbuffer, i2c_rxbuffer, i2c_nbytes and status */
  i2c_txbuffer = i2c_buffer_transmitter;
  i2c_rxbuffer = i2c_buffer_receiver;
  i2c_nbytes = 16;
  status = ERROR;

  /* enable the I2C0 interrupt */
  i2c_interrupt_enable(I2C0, I2C_INT_ERR);
  i2c_interrupt_enable(I2C0, I2C_INT_EV);
  i2c_interrupt_enable(I2C0, I2C_INT_BUF);

  // /* enable the I2C1 interrupt */
  // i2c_interrupt_enable(I2C1, I2C_INT_ERR);
  // i2c_interrupt_enable(I2C1, I2C_INT_EV);
  // i2c_interrupt_enable(I2C1, I2C_INT_BUF);

  /* the master waits until the I2C bus is idle */
  while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
    ;
  /* the master sends a start condition to I2C bus */
  i2c_start_on_bus(I2C0);

  while (i2c_nbytes > 0)
    ;
  while (SUCCESS != status)
    ;
  /* if the transfer is successfully completed, LED1 and LED2 is on */
  state = memory_compare(i2c_buffer_transmitter, i2c_buffer_receiver, 16);
  if (SUCCESS == state) {
    /* if success, LED1 and LED2 are on */
    while (1)
      ;
  }
  while (1) {
  }
}

int main(void) {
  // led_test();
  // usb_gpio_init();
  // usbLoop();
  // mscLoop();
  // firmware_usbLoop();
  // spi_master_init();
  // oledInit();
  // spiLoop();
  // mi2c0_init();
  // // si2c_init();
  // si2c_libopencm3_init();
  // si2cLoop();

  usartLoop();
  return 0;
}

// gd32 drv test loop
#define ARRAYNUM(arr_nanme) (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE (ARRAYNUM(transmitter_buffer) - 1)

uint8_t transmitter_buffer[] = "\n\ra usart half-duplex test example!\n\r";
uint8_t receiver_buffer0[TRANSMIT_SIZE];
uint8_t receiver_buffer1[TRANSMIT_SIZE];
uint8_t transfersize = TRANSMIT_SIZE;
extern volatile uint8_t recvBuf[64];
__IO uint8_t txcount0 = 0;
__IO uint16_t rxcount0 = 0;
__IO uint8_t txcount1 = 0;
__IO uint16_t rxcount1 = 0;
// ErrStatus state = ERROR;

void usart1_isr(void) {
  // if ((RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)) &&
  //     (RESET != usart_flag_get(USART1, USART_FLAG_RBNE))) {
  if (RESET != usart_flag_get(USART1, USART_FLAG_RBNE)) {
    /* receive data */
    // receiver_buffer1[rxcount0++] = usart_data_receive(USART1);
    ble_read_byte(&receiver_buffer1[rxcount0++]);
  }
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usartLoop(void) {
  /* enable USART and GPIOA clock */
  rcu_periph_clock_enable(RCU_GPIOA);
#if 0 
 /*configure USART0*/
  rcu_periph_clock_enable(RCU_USART0);
  gpio_af_set(GPIOA, GPIO_AF_7,
              GPIO_PIN_9 | GPIO_PIN_10);  // USART0 PIN9 -> TX, PIN10 -> RX

  /* configure USART0 TX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                GPIO_PIN_9 | GPIO_PIN_10);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                          GPIO_PIN_9 | GPIO_PIN_10);
  /* USART0 baudrate configuration */
  usart_baudrate_set(USART0, 115200);
  /* configure USART0 transmitter */
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  /* configure USART0 receiver */
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  /* enable USART0 */
  gd32usart_enable(USART0);
// #if 0
  /*configure USART1*/
  rcu_periph_clock_enable(RCU_USART1);
  /* configure the USART1 TX pin and RX pin*/
  gpio_af_set(GPIOA, GPIO_AF_7,
              GPIO_PIN_2 | GPIO_PIN_3);  // USART1 PIN2 -> TX, PIN3 -> RX

  /* configure USART1 TX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2 | GPIO_PIN_3);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                          GPIO_PIN_2 | GPIO_PIN_3);
  /* USART interrupt configuration */
  nvic_irq_enable(USART1_IRQn, 0, 0);
  /* USART1 baudrate configuration */
  usart_baudrate_set(USART1, 115200);
  /* configure USART1 transmitter */
  usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
  /* configure USART1 receiver */
  usart_receive_config(USART1, USART_RECEIVE_ENABLE);
  /* enable USART1 */
  gd32usart_enable(USART1);
  usart_interrupt_enable(USART1, USART_INT_RBNE);
#else
  // libopencm3 init usart1
  ble_usart_init();
#endif

  for (uint8_t i = 0; i < TRANSMIT_SIZE; i++) {
    receiver_buffer0[i] = 0xFF;
    receiver_buffer1[i] = 0xFF;
  }

  /* USART0 transmit and USART1 receive */
  // while (transfersize--) {
  //   // /* wait until end of transmit */
  //   // while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
  //   //   ;
  //   // usart_data_transmit(USART0, transmitter_buffer[txcount0++]);

  //   // ble_usart_send(transmitter_buffer[txcount0++], 1);
  //   /* wait until end of transmit */
  //   while (RESET == usart_flag_get(USART1, USART_FLAG_TBE))
  //     ;
  //   usart_data_transmit(USART1, transmitter_buffer[txcount1++]);
  // }

  /* 开发板需要短接 PA2 == PA3*/
  /* 断点停住后查看recvBuf中值是否与transmitter_buffer相同*/
  ble_usart_send(transmitter_buffer, sizeof(transmitter_buffer));
  /* compare the received data with the send ones */
  state =
      memory_compare(transmitter_buffer, recvBuf, sizeof(transmitter_buffer));
  if (SUCCESS == state) {
  } else {
    while (1) {
      ;
    }
  }
  // /* USART1 transmit and USART0 receive */
  // transfersize = TRANSMIT_SIZE;
  // while (transfersize--) {
  //   /* wait until end of transmit */
  //   while (RESET == usart_flag_get(USART1, USART_FLAG_TBE))
  //     ;
  //   usart_data_transmit(USART1, transmitter_buffer[txcount1++]);

  //   while (RESET == usart_flag_get(USART0, USART_FLAG_RBNE))
  //     ;
  //   /* store the received byte in the receiver_buffer0 */
  //   receiver_buffer0[rxcount1++] = usart_data_receive(USART0);
  // }

  // /* compare the received data with the send ones */
  // state = memory_compare(receiver_buffer0, receiver_buffer1, TRANSMIT_SIZE);
  // if (SUCCESS == state) {
  // } else {
  //   while (1) {
  //     ;
  //   }
  // }
  // while (1) {
  //   ;
  // }
}
