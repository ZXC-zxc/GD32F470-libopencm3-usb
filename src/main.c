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

extern void usbLoop(void);
extern void usbInit(void);
extern void usbPoll(void);
extern int led_test(void);
extern void spiLoop(void);
extern void firmware_usbLoop(void);
void usartLoop(void);

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

// #define TEST_RECV
#define SE_I2C_ADDRESS7 0x20  // 0x72
#define SI2C_ADDR 0x48        // 0x90 /* 使用gd32 driver */

volatile uint8_t mi2c_se_transBuff[16];

#define I2C0_SLAVE_ADDRESS7 0x82
#define I2C1_SLAVE_ADDRESS7 SI2C_ADDR  // 0x72

// combus io level
#define SET_COMBUS_HIGH() (gpio_set(GPIO_CMBUS_PORT, GPIO_SI2C_CMBUS))
#define SET_COMBUS_LOW() (gpio_clear(GPIO_CMBUS_PORT, GPIO_SI2C_CMBUS))

uint8_t i2c_buffer_transmitter[32];
uint8_t i2c_buffer_receiver[32];
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

static void mi2c0_init_common(void) {
  /* enable GPIOB clock */
  rcu_periph_clock_enable(RCU_GPIOB);
  /* enable I2C0 clock */
  rcu_periph_clock_enable(RCU_I2C0);
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

  /* configure I2C clock */
  i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
  // /* configure I2C address for slave mode*/
  // i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS,
  //                      I2C0_SLAVE_ADDRESS7);
  /* enable I2C0 */
  i2c_enable(I2C0);
  /* enable acknowledge */
  i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void vMI2CDRV_Init_ex(void) {
  rcc_periph_clock_enable(RCC_I2C1);
  rcc_periph_clock_enable(RCC_GPIOB);

  gpio_set_output_options(GPIO_MI2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  gpio_set_af(GPIO_MI2C_PORT, GPIO_AF4, GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  gpio_mode_setup(GPIO_MI2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_MI2C_SCL | GPIO_MI2C_SDA);
  i2c_reset(MI2CX);  // void i2c_deinit(uint32_t i2c_periph);
  delay_ms(100);
  i2c_peripheral_disable(MI2CX);  // void i2c_disable(uint32_t i2c_periph);

  // 100k
  i2c_set_speed(MI2CX, i2c_speed_sm_100k,
                30);             // i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
  i2c_peripheral_enable(MI2CX);  // void i2c_enable(uint32_t i2c_periph);
  delay_ms(100);
}

void mi2c0_init_irq(void) {
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

void si2c1_init_irq(void) {
  /* enable GPIOB clock */
  rcu_periph_clock_enable(RCU_GPIOB);
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

  i2c_clock_config(BLE_SI2C, 100000, I2C_DTCY_2);
  /* I2C address configure */
  i2c_mode_addr_config(BLE_SI2C, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS,
                       I2C1_SLAVE_ADDRESS7);
  /* enable I2C1 */
  i2c_enable(BLE_SI2C);
  /* enable acknowledge */
  i2c_ack_config(BLE_SI2C, I2C_ACK_ENABLE);

  nvic_irq_enable(I2C1_EV_IRQn, 0, 4);
  nvic_irq_enable(I2C1_ER_IRQn, 0, 1);
}

void si2c_libopencm3_init(void) {
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_GPIOB);

  i2c_reset(BLE_SI2C);

  gpio_set_output_options(GPIO_SI2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  gpio_mode_setup(GPIO_SI2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  gpio_set_af(GPIO_SI2C_PORT, GPIO_AF4, GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  i2c_peripheral_disable(BLE_SI2C);
  /*	//HSI is at 2Mhz */
  i2c_set_fast_mode(BLE_SI2C);
  i2c_set_speed(BLE_SI2C, i2c_speed_fm_400k, 30);
  /*	//addressing mode*/
  i2c_set_own_7bit_slave_address(BLE_SI2C, SI2C_ADDR);
  i2c_enable_ack(BLE_SI2C);

  // use interrupt
  i2c_enable_interrupt(BLE_SI2C,
                       I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

  // /* enable the I2C1 interrupt */
  // i2c_interrupt_enable(BLE_SI2C, I2C_INT_ERR);
  // i2c_interrupt_enable(BLE_SI2C, I2C_INT_EV);
  // i2c_interrupt_enable(BLE_SI2C, I2C_INT_BUF);

  // I2C_CR1(I2C2) |= I2C_CR1_NOSTRETCH;
  i2c_peripheral_enable(BLE_SI2C);
  // set NVIC
  nvic_set_priority(NVIC_I2C2_EV_IRQ, 0);
  nvic_enable_irq(NVIC_I2C2_EV_IRQ);
  i2c_enable_ack(BLE_SI2C);
}

void comBus_init(void) {
  rcc_periph_clock_enable(RCC_GPIOC);
  // combus
  gpio_mode_setup(GPIO_CMBUS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN,
                  GPIO_SI2C_CMBUS);
  SET_COMBUS_LOW();
}

// mi2c and si2c is irq
void msi2c_irq_Loop(void) {
  mi2c0_init_irq();
  si2c1_init_irq();
  // si2c_libopencm3_init();

  for (uint8_t i = 0; i < 16; i++) {
    i2c_buffer_transmitter[i] = i + 0xA0;
    i2c_buffer_receiver[i] = 0xFF;
  }
  /* initialize i2c_txbuffer, i2c_rxbuffer, i2c_nbytes and status */
  i2c_txbuffer = i2c_buffer_transmitter;
  i2c_rxbuffer = i2c_buffer_receiver;
  i2c_nbytes = 18;
  status = ERROR;

  /* enable the I2C0 interrupt */
  i2c_interrupt_enable(I2C0, I2C_INT_ERR);
  i2c_interrupt_enable(I2C0, I2C_INT_EV);
  i2c_interrupt_enable(I2C0, I2C_INT_BUF);

  /* enable the I2C1 interrupt */
  i2c_interrupt_enable(I2C1, I2C_INT_ERR);
  i2c_interrupt_enable(I2C1, I2C_INT_EV);
  i2c_interrupt_enable(I2C1, I2C_INT_BUF);

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

/* se mi2c driver */
static uint8_t ucXorCheck(uint8_t ucInputXor, uint8_t *pucSrc, uint16_t usLen) {
  uint16_t i;
  uint8_t ucXor;

  ucXor = ucInputXor;
  for (i = 0; i < usLen; i++) {
    ucXor ^= pucSrc[i];
  }
  return ucXor;
}

static bool bMI2CDRV_ReadBytes(uint32_t i2c, uint8_t *res,
                               uint16_t *pusOutLen) {
  volatile uint8_t ucLenBuf[2], ucSW[2], ucXor, ucXor1;
  volatile uint16_t i, usRevLen, usTimeout, usRealLen;

  ucXor = 0;
  i = 0;
  usRealLen = 0;
  usTimeout = 0;

  ucLenBuf[0] = 0x00;
  ucLenBuf[1] = 0x00;

  ucSW[0] = 0x00;
  ucSW[1] = 0x00;

  while (1) {
    if (i > 5) {
      return false;
    }
    usTimeout = 0;
    while (i2c_flag_get(i2c, I2C_FLAG_I2CBSY)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        break;
      }
    }
    i2c_start_on_bus(i2c);
    /* enable acknowledge */
    i2c_ack_config(i2c, I2C_ACK_ENABLE);
    usTimeout = 0;
    while (!i2c_flag_get(i2c, I2C_FLAG_SBSEND)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        break;
      }
    }
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c, SE_I2C_ADDRESS7, I2C_RECEIVER);
    usTimeout = 0;
    // Waiting for address is transferred.
    while (!i2c_flag_get(i2c, I2C_FLAG_ADDSEND)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        break;
      }
    }
    if (usTimeout > MI2C_TIMEOUT) {
      usTimeout = 0;
      i++;
      continue;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c, I2C_FLAG_ADDSEND);
    break;
  }
  // rev len
  for (i = 0; i < 2; i++) {
    usTimeout = 0;
    while (!i2c_flag_get(i2c, I2C_FLAG_RBNE))
      ;
    /* read a data from I2C_DATA */
    ucLenBuf[i] = i2c_data_receive(i2c);
  }
  // cal len xor
  ucXor = ucXorCheck(ucXor, ucLenBuf, sizeof(ucLenBuf));
  // len-SW1SW2
  usRevLen = (ucLenBuf[0] << 8) + (ucLenBuf[1] & 0xFF) - 2;
  if (usRevLen > 0 && (res == NULL)) {
    i2c_stop_on_bus(i2c);
    while (I2C_CTL0(i2c) & I2C_CTL0_STOP)
      ;
    return false;
  }

  // rev data
  for (i = 0; i < usRevLen; i++) {
    while (!i2c_flag_get(i2c, I2C_FLAG_RBNE))
      ;
    if (i < *pusOutLen) {
      res[i] = i2c_data_receive(i2c);
      // cal data xor
      ucXor = ucXorCheck(ucXor, res + i, 1);
      usRealLen++;
    } else {
      ucLenBuf[0] = i2c_data_receive(i2c);
      ucXor = ucXorCheck(ucXor, ucLenBuf, 1);
    }
  }

  // sw1 sw2 len
  for (i = 0; i < 2; i++) {
    /* wait until the RBNE bit is set */
    while (!i2c_flag_get(i2c, I2C_FLAG_RBNE))
      ;
    ucSW[i] = i2c_data_receive(i2c);
    usRealLen++;
  }
  // cal sw1sw2 xor
  ucXor = ucXorCheck(ucXor, ucSW, sizeof(ucSW));

  // xor len
  /* send a NACK for the last data byte */
  i2c_ack_config(i2c, I2C_ACK_DISABLE);
  for (i = 0; i < MI2C_XOR_LEN; i++) {
    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE))
      ;
    ucXor1 = i2c_data_receive(i2c);
    usRealLen++;
  }

  i2c_stop_on_bus(i2c);
  while (I2C_CTL0(i2c) & I2C_CTL0_STOP)
    ;
  if (0x00 == usRealLen) {
    return false;
  }

  if (ucXor != ucXor1) {
    return false;
  }
  usRealLen -= MI2C_XOR_LEN;

  if ((0x90 != ucSW[0]) || (0x00 != ucSW[1])) {
    if (ucSW[0] == 0x6c) {
      res[0] = ucSW[1];
      *pusOutLen = 1;
    } else {
      *pusOutLen = usRealLen - 2;
    }
    return false;
  }
  *pusOutLen = usRealLen - 2;
  return true;
}

static bool bMI2CDRV_WriteBytes(uint32_t i2c, uint8_t *data,
                                uint16_t ucSendLen) {
  volatile uint8_t ucLenBuf[2], ucXor = 0;
  volatile uint16_t i, usTimeout = 0;

  i = 0;
  while (1) {
    if (i > 5) {
      return false;
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c);

    while (!i2c_flag_get(i2c, I2C_FLAG_SBSEND)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        break;
      }
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(i2c, SE_I2C_ADDRESS7, I2C_TRANSMITTER);
    usTimeout = 0;
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(i2c, I2C_FLAG_ADDSEND)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        break;
      }
    }
    if (usTimeout > MI2C_TIMEOUT) {
      i++;
      usTimeout = 0;
      continue;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c, I2C_FLAG_ADDSEND);
    break;
  }
  // send L + V + xor
  ucLenBuf[0] = ((ucSendLen >> 8) & 0xFF);
  ucLenBuf[1] = ucSendLen & 0xFF;
  // len xor
  ucXor = ucXorCheck(ucXor, ucLenBuf, sizeof(ucLenBuf));
  // send len
  for (i = 0; i < 2; i++) {
    i2c_data_transmit(i2c, ucLenBuf[i]);
    usTimeout = 0;
    while (!i2c_flag_get(i2c, I2C_FLAG_TBE)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        return false;
      }
    }
  }
  // cal xor
  ucXor = ucXorCheck(ucXor, data, ucSendLen);
  // send data
  for (i = 0; i < ucSendLen; i++) {
    i2c_data_transmit(i2c, data[i]);
    usTimeout = 0;
    while (!i2c_flag_get(i2c, I2C_FLAG_TBE)) {
      usTimeout++;
      if (usTimeout > MI2C_TIMEOUT) {
        return false;
      }
    }
  }
  // send Xor
  i2c_data_transmit(i2c, ucXor);
  usTimeout = 0;
  while (!i2c_flag_get(i2c, I2C_FLAG_TBE)) {
    usTimeout++;
    if (usTimeout > MI2C_TIMEOUT) {
      return false;
    }
  }

  i2c_stop_on_bus(i2c);
  usTimeout = 0;
  while (I2C_CTL0(i2c) & I2C_CTL0_STOP) {
    usTimeout++;
    if (usTimeout > MI2C_TIMEOUT) {
      return false;
    }
  }
  //  delay_us(100);
  return true;
}

// static void gd32mi2c_recv(void) {
//   /* wait until I2C bus is idle */
//   while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
//     ;
//   /* send a start condition to I2C bus */
//   i2c_start_on_bus(I2C0);
//   /* wait until SBSEND bit is set */
//   while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
//     ;
//   /* send slave address to I2C bus */
//   i2c_master_addressing(I2C0, I2C1_SLAVE_ADDRESS7, I2C_RECEIVER);
//   /* wait until ADDSEND bit is set */
//   while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
//     ;
//   /* clear ADDSEND bit */
//   i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

//   for (uint8_t i = 0; i < 15; i++) {
//     /* wait until the RBNE bit is set */
//     while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE))
//       ;
//     /* read a data from I2C_DATA */
//     i2c_rxbuffer[i] = i2c_data_receive(I2C0);
//   }
//   /* send a NACK for the last data byte */
//   i2c_ack_config(I2C0, I2C_ACK_DISABLE);

//   /* send a stop condition to I2C bus */
//   i2c_stop_on_bus(I2C0);
//   while (I2C_CTL0(I2C0) & I2C_CTL0_STOP)
//     ;
//   i2c_rxbuffer[i] = i2c_data_receive(I2C0);
//   i2c_ack_config(I2C0, I2C_ACK_ENABLE);
//   /* clear the bit of AERR */
//   i2c_flag_clear(I2C1, I2C_FLAG_AERR);
// }

uint8_t g_ucRandCmd[5] = {0x00, 0x84, 0x00, 0x00, 0x10};
uint8_t g_ucRecvBuf[32];

// mi2c poll and si2c irq. TEST_RECV is si2c switch
void msi2c_com_Loop(void) {
  uint8_t i;

  comBus_init();
  // mi2c0_init_common();
  // vMI2CDRV_Init_ex();
  // si2c1_init_irq();
  si2c_libopencm3_init();

  i2c_txbuffer = i2c_buffer_transmitter;
  i2c_rxbuffer = i2c_buffer_receiver;
  i2c_nbytes = 18;
  status = ERROR;

  for (i = 0; i < 16; i++) {
    mi2c_se_transBuff[i] = i + 0x80;
    i2c_buffer_transmitter[i] = i + 0xA0;
    i2c_buffer_receiver[i] = 0xFF;
  }

  memcpy(i2c_buffer_transmitter,
         "\x00\x12\xBC\x52\x17\x65\xA4\xE7\x9D\x3D\x67\xEE\x6E\x67\xC7\xDA\x85"
         "\xC8\x90\x00\x2D",
         21);

#ifdef TEST_RECV
  volatile uint16_t retLen = 0xff;
  bMI2CDRV_WriteBytes(MI2CX, g_ucRandCmd, 5);
  delay_ms(10);
  bMI2CDRV_ReadBytes(MI2CX, g_ucRecvBuf, &retLen);
  while (1)
    ;
  state = memory_compare(g_ucRandCmd, i2c_buffer_receiver + 2, 5);
  if (SUCCESS == state) {
    /* if success, LED1 and LED2 are on */
    while (1) {
    }
  }

#else
  while (i2c_nbytes != 0) {
    ;
  }
  // 判断数据正确性
  if ((0x5a == i2c_buffer_receiver[0]) && (0xa5 == i2c_buffer_receiver[17])) {
    memcpy(i2c_buffer_transmitter, i2c_buffer_receiver, 18);
    SET_COMBUS_HIGH();
  } else {
    SET_COMBUS_LOW();
  }

  while (1)
    ;
#endif
  while (1) {
  }
}

int main(void) {
  // led_test();
  // usb_gpio_init();
  // usbLoop();

  // firmware_usbLoop();
  // spi_master_init();
  // oledInit();
  // spiLoop();
  usartLoop();
  // msi2c_irq_Loop();
  // msi2c_com_Loop();
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

volatile uint8_t recvBuf[64];
volatile uint8_t recvCunt = 0;

void usart1_isr(void) {
  // if ((RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)) &&
  //     (RESET != usart_flag_get(USART1, USART_FLAG_RBNE))) {
  if (RESET != usart_flag_get(BLE_UART, USART_FLAG_RBNE)) {
    /* receive data */
    // receiver_buffer1[rxcount0++] = usart_data_receive(USART1);
    ble_read_byte(&receiver_buffer1[rxcount0++]);
  }
}

static void ble_gd32usart_init(void) {
  /*configure USART1*/
  rcu_periph_clock_enable(RCU_USART1);
  rcu_periph_clock_enable(RCU_GPIOA);
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
  usart_word_length_set(USART1, USART_WL_8BIT);
  usart_parity_config(USART1, USART_PM_NONE);
  usart_stop_bit_set(USART1, USART_STB_1BIT);
  usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
  /* configure USART1 transmitter */
  usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
  /* configure USART1 receiver */
  usart_receive_config(USART1, USART_RECEIVE_ENABLE);
  /* enable USART1 */
  gd32usart_enable(USART1);
  usart_interrupt_enable(USART1, USART_INT_RBNE);
}

void usartLoop(void) {
  comBus_init();
  SET_COMBUS_LOW();
  recvCunt = 0;
  for (uint8_t i = 0; i < 18; i++) {
    transmitter_buffer[i] = 0x5a;
    // receiver_buffer1[i] = 0xFF;
  }

#ifndef _SUPPORT_GD32DRIVERS
  ble_gd32usart_init();
#else
  // libopencm3 init usart1
  ble_usart_init();
#endif

#ifdef TEST_SELF
  /* 开发板需要短接 PA2(TX) == PA3(RX)*/
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
#else
  // /* 与nordic52832调试usart功能*/
  // /* 判断数据正确性 */
  // transmitter_buffer[0] = 0xa1;
  // transmitter_buffer[17] = 0xb2;
  // ble_usart_send(transmitter_buffer, 18);
  // while (1)
  //   ;

  while (recvCunt != 0x12) {
    ;
  }

  if ((0x5a == recvBuf[0]) && (0xa5 == recvBuf[17])) {
    SET_COMBUS_HIGH();
    memcpy(transmitter_buffer, recvBuf, 18);
    ble_usart_send(transmitter_buffer, 18);
  } else {  // 0xff
    SET_COMBUS_LOW();
  }

  while (1)
    ;
#endif
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
