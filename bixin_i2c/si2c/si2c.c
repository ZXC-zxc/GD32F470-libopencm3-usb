#include <errno.h>
// #include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <stdio.h>
#include <string.h>

// #include "./firmware/chinese.h"
// #include "buttons.h"
// #include "common.h"
// #include "layout.h"
#include "oled.h"
#include "si2c.h"
// #include "sys.h"
// #include "timer.h"
// #include "usart.h"
#include "util.h"

ChannelType host_channel = CHANNEL_NULL;
volatile uint32_t i2c_data_inlen;
volatile bool i2c_recv_done = false;
uint8_t i2c_data_out[SI2C_BUF_MAX_OUT_LEN];
volatile uint32_t i2c_data_outlen, i2c_data_out_pos;
static volatile bool wait_response = true;

trans_fifo i2c_fifo_in = {.p_buf = i2c_data_out,
                          .buf_size = sizeof(i2c_data_out),
                          .over_pre = false,
                          .read_pos = 0,
                          .write_pos = 0,
                          .lock_pos = 0};

trans_fifo i2c_fifo_out = {.p_buf = i2c_data_out,
                           .buf_size = SI2C_BUF_MAX_OUT_LEN,
                           .over_pre = false,
                           .read_pos = 0,
                           .write_pos = 0,
                           .lock_pos = 0};

void i2c_slave_init_irq(void) {
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_GPIOB);

  i2c_reset(I2C2);

  gpio_set_output_options(GPIO_SI2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  gpio_mode_setup(GPIO_SI2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  gpio_set_af(GPIO_SI2C_PORT, GPIO_AF4, GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  i2c_peripheral_disable(I2C2);
  /*	//HSI is at 2Mhz */
  i2c_set_fast_mode(I2C2);
  i2c_set_speed(I2C2, i2c_speed_fm_400k, 30);
  /*	//addressing mode*/
  i2c_set_own_7bit_slave_address(I2C2, SI2C_ADDR);
  i2c_enable_ack(I2C2);

  // use interrupt
  i2c_enable_interrupt(I2C2,
                       I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

  // I2C_CR1(I2C2) |= I2C_CR1_NOSTRETCH;
  i2c_peripheral_enable(I2C2);

  // // set NVIC
  // nvic_set_priority(NVIC_I2C2_EV_IRQ, 0);
  // nvic_enable_irq(NVIC_I2C2_EV_IRQ);

  i2c_enable_ack(I2C2);

  memset(i2c_data_out, 0x00, SI2C_BUF_MAX_OUT_LEN);
}

void i2c_slave_init(void) {
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_GPIOB);

  i2c_reset(I2C2);

  gpio_set_output_options(GPIO_SI2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  gpio_mode_setup(GPIO_SI2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  gpio_set_af(GPIO_SI2C_PORT, GPIO_AF4, GPIO_SI2C_SCL | GPIO_SI2C_SDA);
  i2c_peripheral_disable(I2C2);
  /*	//HSI is at 2Mhz */
  i2c_set_fast_mode(I2C2);
  i2c_set_speed(I2C2, i2c_speed_fm_400k, 32);
  /*	//addressing mode*/
  i2c_set_own_7bit_slave_address(I2C2, SI2C_ADDR);
  i2c_enable_ack(I2C2);

  // I2C_CR1(I2C2) |= I2C_CR1_NOSTRETCH;
  i2c_peripheral_enable(I2C2);

  i2c_enable_ack(I2C2);

  memset(i2c_data_out, 0x00, SI2C_BUF_MAX_OUT_LEN);
}

static void i2c_delay(void) {
  volatile uint32_t i = 1000;
  while (i--)
    ;
}

extern uint32_t flash_pos;
void i2c2_ev_isr() {
  uint32_t sr1, sr2;
  static uint8_t dir = 0;  // 0-receive 1-send
  sr1 = I2C_SR1(I2C2);
  if (sr1 & I2C_SR1_ADDR) {  // EV1
    sr2 = I2C_SR2(I2C2);     // clear flag
    dir = sr2 & I2C_SR2_TRA;
  }
  if (sr1 & I2C_SR1_RxNE) {  // EV2
    if (!fifo_put_no_overflow(&i2c_fifo_in, i2c_get_data(I2C2))) {
      // layoutError("buffer overflow", "i2c receive");
    }
  }
  if (dir & I2C_SR2_TRA) {
    if (sr1 & I2C_SR1_TxE) {  // EV3 ev3-1
      if (i2c_data_outlen > 0) {
        i2c_send_data(I2C2, i2c_data_out[i2c_data_out_pos++]);
        do {
          i2c_delay();
          sr1 = I2C_SR1(I2C2);
        } while ((!(sr1 & I2C_SR1_BTF)) && (!((sr1 & I2C_SR1_AF))));
        i2c_data_outlen--;
        if (i2c_data_outlen == 0) {
          SET_COMBUS_LOW();
        }
      } else {
        i2c_send_data(I2C2, '#');
      }
    } else if (sr1 & I2C_SR1_BTF) {
      i2c_send_data(I2C2, i2c_data_out[i2c_data_out_pos++]);
      i2c_data_outlen--;
      if (i2c_data_outlen == 0) {
        SET_COMBUS_LOW();
      }
    }
  }
  if (sr1 & I2C_SR1_STOPF) {  // EV4
    I2C_CR1(I2C2) |= I2C_CR1_PE;
    fifo_lockpos_set(&i2c_fifo_in);
    i2c_recv_done = true;
    i2c_data_outlen = 0;  // discard former response
    SET_COMBUS_LOW();
  }
  if (sr1 & I2C_SR1_AF) {  // EV4
    I2C_SR1(I2C2) &= ~I2C_SR1_AF;
  }
}

void i2c2_er_isr(void) {}

void i2c_set_wait(bool flag) { wait_response = flag; }

// bool i2c_slave_send(uint32_t data_len) {
//   uint32_t len = 0;
//   uint32_t i;
//   int32_t counter, counter_bak, counter_set;
//   bool flag = true;
//   bool status = false;

//   if (data_len > 64) len = data_len - 64;
//   if (len) {
//     for (i = 0; i < (len / 64); i++) {
//       memmove(i2c_data_out + 64 + i * 63, i2c_data_out + (i + 1) * 64 + 1,
//       63);
//     }
//   }
//   i2c_data_outlen = (i2c_data_out[5] << 24) + (i2c_data_out[6] << 16) +
//                     (i2c_data_out[7] << 8) + i2c_data_out[8] + 9;
//   i2c_data_out_pos = 0;
//   SET_COMBUS_HIGH();
//   if (wait_response) {
//     // timer_out_set(timer_out_resp, default_resp_time);
//     // counter = counter_bak = counter_set = default_resp_time / timer1s;
//   } else {
//     wait_response = true;
//     // timer_out_set(timer_out_resp, timer1s);
//     // counter = counter_bak = counter_set = timer1s / timer1s;
//   }

//   while (1) {
//     // if (checkButtonOrTimeout(BTN_PIN_NO, timer_out_resp) == true) {
//     //   break;
//     // }
//     if (i2c_data_outlen == 0) {
//       status = true;
//       break;
//     } else {
//       // counter = timer_out_get(timer_out_resp) / timer1s;
//       // show timer count down after 2 seconds
//       if (counter <= counter_set - 2) {
//         if (counter_bak != counter) {
//           if (flag) {
//             flag = false;
//             oledBufferBak();
//             oledClear();
//             // if (ui_language) {
//             //   oledDrawStringCenterAdapter(OLED_WIDTH / 2, 32,
//             "等待连接...",
//             //                               FONT_STANDARD);
//             // } else {
//             //   oledDrawStringCenter(OLED_WIDTH / 2, 32, "Waiting
//             connect...",
//             //                        FONT_STANDARD);
//             // }
//           }
//           char asc_buf[16] = {0};
//           uint2str(counter, asc_buf);
//           oledBox(0, 20, OLED_WIDTH, 20 + 8, false);
//           counter_bak = counter;
//           // oledDrawStringCenter(OLED_WIDTH / 2, 20, asc_buf,
//           FONT_STANDARD); oledRefresh();
//         }
//       }
//     }
//   }
//   i2c_data_outlen = 0;
//   // timer_out_set(timer_out_resp, 0);
//   SET_COMBUS_LOW();
//   if (!flag) {
//     oledBufferResume();
//     oledRefresh();
//   }
//   return status;
// }

// // used in bootloader,no chinese prompt
// void i2c_slave_send_ex(uint32_t data_len) {
//   uint32_t len = 0;
//   uint32_t i;
//   int32_t counter, counter_bak, counter_set;
//   bool flag = true;

//   if (data_len > 64) len = data_len - 64;
//   if (len) {
//     for (i = 0; i < (len / 64); i++) {
//       memmove(i2c_data_out + 64 + i * 63, i2c_data_out + (i + 1) * 64 + 1,
//       63);
//     }
//   }
//   i2c_data_outlen = (i2c_data_out[5] << 24) + (i2c_data_out[6] << 16) +
//                     (i2c_data_out[7] << 8) + i2c_data_out[8] + 9;
//   i2c_data_out_pos = 0;
//   SET_COMBUS_HIGH();
//   if (wait_response) {
//     timer_out_set(timer_out_resp, default_resp_time);
//     counter = counter_bak = counter_set = default_resp_time / timer1s;
//   } else {
//     wait_response = true;
//     timer_out_set(timer_out_resp, timer1s);
//     counter = counter_bak = counter_set = timer1s / timer1s;
//   }

//   while (1) {
//     if (checkButtonOrTimeout(BTN_PIN_NO, timer_out_resp) == true ||
//         i2c_data_outlen == 0) {
//       break;
//     } else {
//       counter = timer_out_get(timer_out_resp) / timer1s;
//       // show timer count down after 2 seconds
//       if (counter <= counter_set - 2) {
//         if (counter_bak != counter) {
//           if (flag) {
//             flag = false;
//             oledBufferBak();
//             oledClear();
//             oledDrawStringCenter(OLED_WIDTH / 2, 32, "Waiting connect...",
//                                  FONT_STANDARD);
//           }
//           char asc_buf[16] = {0};
//           uint2str(counter, asc_buf);
//           oledBox(0, 20, OLED_WIDTH, 20 + 8, false);
//           counter_bak = counter;
//           oledDrawStringCenter(OLED_WIDTH / 2, 20, asc_buf, FONT_STANDARD);
//           oledRefresh();
//         }
//       }
//     }
//   }
//   i2c_data_outlen = 0;
//   timer_out_set(timer_out_resp, 0);
//   SET_COMBUS_LOW();
//   if (!flag) {
//     oledBufferResume();
//     oledRefresh();
//   }
// }

// gd32 driver for i2c interrupt handle
#include "gd32f4xx.h"
#define I2C0_SLAVE_ADDRESS7 0x82
#define I2C1_SLAVE_ADDRESS7 0x72

extern volatile ErrStatus status;
extern volatile uint8_t *i2c_txbuffer;
extern volatile uint8_t *i2c_rxbuffer;
extern volatile uint16_t i2c_nbytes;

/*!
    \brief      handle I2C0 event interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c0_event_irq_handler(void) {
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SBSEND)) {
    /* the master sends slave address */
    i2c_master_addressing(I2C0, I2C1_SLAVE_ADDRESS7, I2C_RECEIVER);
  } else if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_ADDSEND)) {
    if ((1 == i2c_nbytes) || (2 == i2c_nbytes)) {
      /* clear the ACKEN before the ADDSEND is cleared */
      i2c_ack_config(I2C0, I2C_ACK_DISABLE);
      /* clear the ADDSEND bit */
      i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_ADDSEND);
    } else {
      /* clear the ADDSEND bit */
      i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_ADDSEND);
    }
  } else if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_RBNE)) {
    if (i2c_nbytes > 0) {
      if (3 == i2c_nbytes) {
        /* wait until the second last data byte is received into the shift
         * register */
        while (!i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BTC))
          ;
        /* send a NACK for the last data byte */
        i2c_ack_config(I2C0, I2C_ACK_DISABLE);
      }
      /* read a data byte from I2C_DATA*/
      *i2c_rxbuffer++ = i2c_data_receive(I2C0);
      i2c_nbytes--;
      if (0 == i2c_nbytes) {
        /* send a stop condition */
        i2c_stop_on_bus(I2C0);
        status = SUCCESS;
        i2c_ack_config(I2C0, I2C_ACK_ENABLE);
        i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
        /* disable the I2C0 interrupt */
        i2c_interrupt_disable(I2C0, I2C_INT_ERR);
        i2c_interrupt_disable(I2C0, I2C_INT_BUF);
        i2c_interrupt_disable(I2C0, I2C_INT_EV);
      }
    }
  }
}

/*!
    \brief      handle I2C0 error interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c0_error_irq_handler(void) {
  /* no acknowledge received */
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_AERR)) {
    i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_AERR);
  }

  /* SMBus alert */
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBALT)) {
    i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBALT);
  }

  /* bus timeout in SMBus mode */
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SMBTO)) {
    i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_SMBTO);
  }

  /* over-run or under-run when SCL stretch is disabled */
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_OUERR)) {
    i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_OUERR);
  }

  /* arbitration lost */
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_LOSTARB)) {
    i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_LOSTARB);
  }

  /* bus error */
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_BERR)) {
    i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_BERR);
  }

  /* CRC value doesn't match */
  if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_PECERR)) {
    i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_PECERR);
  }

  /* disable the I2C0 interrupt */
  i2c_interrupt_disable(I2C0, I2C_INT_ERR);
  i2c_interrupt_disable(I2C0, I2C_INT_BUF);
  i2c_interrupt_disable(I2C0, I2C_INT_EV);
}

// 需要注意一点 libopencm3中的I2C2 = gd32 sdk 中I2C1
/*
I2C1 == I2C0
I2C2 == I2C1
I2C3 == I2C2
*/
void gd32i2c_ev_recv_isr() {
  uint32_t sr1, sr2;
  static uint8_t dir = 0;  // 0-receive 1-send
  sr1 = I2C_SR1(I2C1);
  if (sr1 & I2C_SR1_ADDR) {  // EV1
    sr2 = I2C_SR2(I2C1);     // clear flag
    // i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_ADDSEND);
    dir = sr2 & I2C_SR2_TRA;
  }
  if (sr1 & I2C_SR1_RxNE) {  // EV2
    // if (!fifo_put_no_overflow(&i2c_fifo_in, i2c_get_data(I2C2))) {
    //   // layoutError("buffer overflow", "i2c receive");
    // }
    *i2c_rxbuffer++ = i2c_get_data(I2C1);
  }

  if (sr1 & I2C_SR1_STOPF) {  // EV4
    I2C_CR1(I2C1) |= I2C_CR1_PE;
    status = SUCCESS;
    /* clear the STPDET bit */
    i2c_enable(I2C1);
    // fifo_lockpos_set(&i2c_fifo_in);
    i2c_recv_done = true;
    i2c_data_outlen = 0;  // discard former response
    // SET_COMBUS_LOW();
  }
  if (sr1 & I2C_SR1_AF) {  // EV4
    I2C_SR1(I2C1) &= ~I2C_SR1_AF;
  }
}

//
void gd32i2c_ev_send_isr() {
  uint32_t sr1, sr2;
  static uint8_t dir = 0;  // 0-receive 1-send
  sr1 = I2C_SR1(I2C1);
  if (sr1 & I2C_SR1_ADDR) {  // EV1
    sr2 = I2C_SR2(I2C1);     // clear flag
    dir = sr2 & I2C_SR2_TRA;
  }
  if (sr1 & I2C_SR1_RxNE) {  // EV2
    // if (!fifo_put_no_overflow(&i2c_fifo_in, i2c_get_data(I2C2))) {
    // layoutError("buffer overflow", "i2c receive");
    // }
    *i2c_rxbuffer++ = i2c_get_data(I2C1);
  }
  if (dir & I2C_SR2_TRA) {
    if ((sr1 & I2C_SR1_TxE) || (sr1 & I2C_SR1_BTF)) {
      i2c_send_data(I2C1, *i2c_txbuffer++);
    }
    // if (sr1 & I2C_SR1_TxE) {  // EV3 ev3-1
    //   if (i2c_data_outlen > 0) {
    //     i2c_send_data(I2C2, i2c_data_out[i2c_data_out_pos++]);
    //     do {
    //       i2c_delay();
    //       sr1 = I2C_SR1(I2C2);
    //     } while ((!(sr1 & I2C_SR1_BTF)) && (!((sr1 & I2C_SR1_AF))));
    //     i2c_data_outlen--;
    //     if (i2c_data_outlen == 0) {
    //       SET_COMBUS_LOW();
    //     }
    //   } else {
    //     i2c_send_data(I2C2, '#');
    //   }
    // } else if (sr1 & I2C_SR1_BTF) {
    //   i2c_send_data(I2C2, i2c_data_out[i2c_data_out_pos++]);
    //   i2c_data_outlen--;
    //   if (i2c_data_outlen == 0) {
    //     SET_COMBUS_LOW();
    //   }
    // }
  }
  if (sr1 & I2C_SR1_STOPF) {  // EV4
    I2C_CR1(I2C1) |= I2C_CR1_PE;
    // fifo_lockpos_set(&i2c_fifo_in);
    // i2c_recv_done = true;
    // i2c_data_outlen = 0;  // discard former response
    // SET_COMBUS_LOW();
  }
  if (sr1 & I2C_SR1_AF) {  // EV4
    I2C_SR1(I2C1) &= ~I2C_SR1_AF;
  }
}

void gd32si2c_ev_isr() {
  uint32_t sr1, sr2;
  static uint8_t dir = 0;  // 0-receive 1-send
  sr1 = I2C_SR1(BLE_SI2C);
  if (sr1 & I2C_SR1_ADDR) {   // EV1
    sr2 = I2C_SR2(BLE_SI2C);  // clear flag
    dir = sr2 & I2C_SR2_TRA;
  }
  if (sr1 & I2C_SR1_RxNE) {  // EV2
    // if (!fifo_put_no_overflow(&i2c_fifo_in, i2c_get_data(I2C1))) {
    //   layoutError("buffer overflow", "i2c receive");
    // }
    *i2c_rxbuffer++ = i2c_get_data(BLE_SI2C);
  }
  if (dir & I2C_SR2_TRA) {
    if ((sr1 & I2C_SR1_TxE) || (sr1 & I2C_SR1_BTF)) {
      i2c_send_data(BLE_SI2C, *i2c_txbuffer++);
    }
    // if (sr1 & I2C_SR1_TxE) {  // EV3 ev3-1
    //   if (i2c_data_outlen > 0) {
    //     i2c_send_data(I2C1, i2c_data_out[i2c_data_out_pos++]);
    //     do {
    //       i2c_delay();
    //       sr1 = I2C_SR1(I2C1);
    //     } while ((!(sr1 & I2C_SR1_BTF)) && (!((sr1 & I2C_SR1_AF))));
    //     i2c_data_outlen--;
    //     if (i2c_data_outlen == 0) {
    //       SET_COMBUS_LOW();
    //     }
    //   } else {
    //     i2c_send_data(I2C1, '#');
    //   }
    // } else if (sr1 & I2C_SR1_BTF) {
    //   i2c_send_data(I2C2, i2c_data_out[i2c_data_out_pos++]);
    //   i2c_data_outlen--;
    //   if (i2c_data_outlen == 0) {
    //     SET_COMBUS_LOW();
    //   }
    // }
  }
  if (sr1 & I2C_SR1_STOPF) {  // EV4
    I2C_CR1(BLE_SI2C) |= I2C_CR1_PE;
    // fifo_lockpos_set(&i2c_fifo_in);
    // i2c_recv_done = true;
    // i2c_data_outlen = 0;  // discard former response
    // SET_COMBUS_LOW();
  }
  if (sr1 & I2C_SR1_AF) {  // EV4
    I2C_SR1(BLE_SI2C) &= ~I2C_SR1_AF;
  }
}

/*!
    \brief      handle I2C1 event interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c1_event_irq_handler(void) {
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_ADDSEND)) {
    /* clear the ADDSEND bit */
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_ADDSEND);
  } else if ((i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_TBE)) &&
             (!i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_AERR))) {
    /* send a data byte */
    i2c_data_transmit(I2C1, *i2c_txbuffer++);
  }
}

/*!
    \brief      handle I2C1 error interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c1_error_irq_handler(void) {
  /* no acknowledge received */
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_AERR)) {
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_AERR);
  }

  /* SMBus alert */
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_SMBALT)) {
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_SMBALT);
  }

  /* bus timeout in SMBus mode */
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_SMBTO)) {
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_SMBTO);
  }

  /* over-run or under-run when SCL stretch is disabled */
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_OUERR)) {
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_OUERR);
  }

  /* arbitration lost */
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_LOSTARB)) {
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_LOSTARB);
  }

  /* bus error */
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_BERR)) {
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_BERR);
  }

  /* CRC value doesn't match */
  if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_PECERR)) {
    i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_PECERR);
  }

  /* disable the I2C0 interrupt */
  i2c_interrupt_disable(I2C1, I2C_INT_ERR);
  i2c_interrupt_disable(I2C1, I2C_INT_BUF);
  i2c_interrupt_disable(I2C1, I2C_INT_EV);
}