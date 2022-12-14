/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
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

#include <errno.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <string.h>

// #include "ble.h"
#include "usart.h"

#ifdef _SUPPORT_GD32DRIVERS
// #include "gd32f4xx.h"
#endif

#if (_SUPPORT_DEBUG_UART_)
/************************************************************************
函数名称:vUART_HtoA
参数:
        pucSrc:		要格式化的源数据
        usLen:		要格式化的源数据长度
        pucDes:		格式化后的数据
返回:
        NULL
功能:
        该函数用于UART数据格式化数据。
************************************************************************/
static void vUART_HtoA(uint8_t *pucSrc, uint16_t usLen, uint8_t *pucDes) {
  uint16_t i, j;
  uint8_t mod = 1;  //,sign;

  for (i = 0, j = 0; i < 2 * usLen; i += 2, j++) {
    mod = (pucSrc[j] >> 4) & 0x0F;
    if (mod <= 9)
      pucDes[i] = mod + 48;
    else
      pucDes[i] = mod + 55;

    mod = pucSrc[j] & 0x0F;
    if (mod <= 9)
      pucDes[i + 1] = mod + 48;
    else
      pucDes[i + 1] = mod + 55;
  }
}
/************************************************************************
函数名称:vUART_DebugInfo
参数:
        pucSendData:		要发送的数据
        usStrLen:			要发送的数据长度
返回:
        NULL
功能:
        该函数用于UART数据发送。
************************************************************************/
static void vUART_SendData(uint8_t *pucSendData, uint16_t usStrLen) {
  uint16_t i;
  for (i = 0; i < usStrLen; i++) {
#ifdef _SUPPORTR_GD32DRIVERS
    usart_send_blocking(USART0, pucSendData[i]);
#else
    usart_send_blocking(USART1, pucSendData[i]);
#endif
  }
}

/************************************************************************
函数名称:vUART_DebugInfo
参数:
        pcMsgTag:		提示消息
        pucSendData:	需要格式化发送的数据
        usStrLen:		数据长度
返回:
        NULL
功能:
        该函数用于格式化发送数据。
************************************************************************/
void vUART_DebugInfo(char *pcMsg, uint8_t *pucSendData, uint16_t usStrLen) {
  uint8_t ucBuff[600];

  vUART_SendData((uint8_t *)pcMsg, strlen(pcMsg));
  if (pucSendData != NULL) {
    vUART_HtoA(pucSendData, usStrLen, ucBuff);
    vUART_SendData(ucBuff, usStrLen * 2);
  }
  vUART_SendData((uint8_t *)"\n", 1);
}

#ifdef _SUPPORT_GD32DRIVERS
void gd32debug_usart_setup(void) {
  /* enable GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOA);
  /* enable USART clock */
  rcu_periph_clock_enable(RCU_USART0);
  /* configure the USART0 TX pin and USART0 RX pin */
  gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9);
  gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_10);

  /* configure USART0 TX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

  /* configure USART0 RX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

  /* USART configure */
  usart_deinit(USART0);
  usart_baudrate_set(USART0, 115200U);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  usart_enable(USART0);
}
#else
void usart_setup(void) {
  rcc_periph_clock_enable(RCC_USART1);
  // rcc_periph_clock_enable(RCC_GPIOA);
  // gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
  // gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
  gpio_set_af(GPIOB, GPIO_AF7, GPIO6 | GPIO7);

  /* Setup UART parameters. */
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART1, USART_MODE_TX);

  /* Finally enable the USART. */
  usart_enable(USART1);
}
#endif

#endif

void ble_usart_init(void) {
  // enable USART clock
  rcc_periph_clock_enable(RCC_USART2);
  //	set GPIO for USART1
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2 | GPIO3);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
  usart_disable(BLE_UART);
  // usart2 set
  usart_set_baudrate(BLE_UART, 115200);
  usart_set_databits(BLE_UART, 8);
  usart_set_stopbits(BLE_UART, USART_STOPBITS_1);
  usart_set_parity(BLE_UART, USART_PARITY_NONE);
  usart_set_flow_control(BLE_UART, USART_FLOWCONTROL_NONE);
  usart_set_mode(BLE_UART, USART_MODE_TX_RX);
  usart_enable(BLE_UART);
  // set NVIC
  ble_usart_irq_set();
}

void ble_usart_irq_set(void) {
  // set NVIC
  nvic_set_priority(NVIC_USART2_IRQ, 0);
  nvic_enable_irq(NVIC_USART2_IRQ);
  usart_enable_rx_interrupt(BLE_UART);
}

void ble_usart_enable(void) { usart_enable(BLE_UART); }
void ble_usart_disable(void) { usart_disable(BLE_UART); }

void ble_usart_irq_enable(void) { usart_enable_rx_interrupt(BLE_UART); }
void ble_usart_irq_disable(void) { usart_disable_rx_interrupt(BLE_UART); }

void ble_usart_sendByte(uint8_t data) {
  usart_send_blocking(BLE_UART, data);
  while (!usart_get_flag(BLE_UART, USART_SR_TXE))
    ;
}

void ble_usart_send(uint8_t *buf, uint32_t len) {
  uint32_t i;
  for (i = 0; i < len; i++) {
    usart_send_blocking(BLE_UART, buf[i]);
    while (!usart_get_flag(BLE_UART, USART_SR_TXE))
      ;
  }
}

bool ble_read_byte(uint8_t *buf) {
  uint16_t tmp;
  if (usart_get_flag(BLE_UART, USART_SR_RXNE) != 0) {
    tmp = usart_recv(BLE_UART);
    buf[0] = (uint8_t)tmp;
    return true;
  }
  return false;
}

void usart2_isr(void) {
  if (usart_get_flag(BLE_UART, USART_SR_RXNE) != 0) {
    ble_uart_poll();
  }
}

extern volatile uint8_t recvBuf[64];
extern volatile uint8_t recvCunt;
void usart1_Ex_isr(void) {
  if (usart_get_flag(BLE_UART, USART_SR_RXNE) != 0) {
    ble_read_byte(&recvBuf[recvCunt++]);
    if ((recvCunt == 1) && (recvBuf[0] != 0x5a)) {  // 测试需要
      recvCunt = 0;
    }
  }
}
