#ifndef _mi2c_H_
#define _mi2c_H_

#include <stdint.h>
#include <string.h>

#include <stdbool.h>

// #include "sys.h"
// #include "usart.h"

#define MI2C_TIMEOUT (40000)
#define MI2C_BUF_MAX_LEN (1024 + 512)
#define MI2C_SEND_MAX_LEN (1024 + 512)

#ifdef GD32F470
#include "gd32f4xx.h"
#define MI2CX I2C0
// master I2C gpio
#define GPIO_MI2C_PORT GPIOB
//#define MI2C_COMBUS     GPIO2
#define GPIO_MI2C_SCL GPIO6
#define GPIO_MI2C_SDA GPIO7
#else
#define MI2CX I2C1
// master I2C gpio
#define GPIO_MI2C_PORT GPIOB
//#define MI2C_COMBUS     GPIO2
#define GPIO_MI2C_SCL GPIO8
#define GPIO_MI2C_SDA GPIO9
#endif

#ifdef NORMAL_PCB
// SE power IO
#define GPIO_SE_PORT GPIOB
#define GPIO_SE_POWER GPIO13
#else
// SE power IO
#define GPIO_SE_PORT GPIOC
#define GPIO_SE_POWER GPIO8
#endif

// power control SE
#define POWER_ON_SE() (gpio_set(GPIO_SE_PORT, GPIO_SE_POWER))
#define POWER_OFF_SE() (gpio_clear(GPIO_SE_PORT, GPIO_SE_POWER))

// master I2C addr
#define MI2C_ADDR 0x10
#ifndef GD32F470
#include "gd32f4xx.h"
#define MI2C_READ I2C_RECEIVER
#define MI2C_WRITE I2C_TRANSMITTER
#else
#define MI2C_READ 0x01
#define MI2C_WRITE 0x00
#endif

#define MI2C_XOR_LEN (1)

//#define	GET_MI2C_COMBUS	        (gpio_get(GPIO_MI2C_PORT, MI2C_COMBUS))

extern uint8_t g_ucMI2cRevBuf[MI2C_BUF_MAX_LEN];
extern uint8_t g_ucMI2cSendBuf[MI2C_BUF_MAX_LEN];

extern uint16_t g_usMI2cRevLen;

#define CLA (g_ucMI2cSendBuf[0])
#define INS (g_ucMI2cSendBuf[1])
#define P1 (g_ucMI2cSendBuf[2])
#define P2 (g_ucMI2cSendBuf[3])
#define P3 (g_ucMI2cSendBuf[4])

#define SH_IOBUFFER (g_ucMI2cSendBuf + 5)
#define SH_CMDHEAD (g_ucMI2cSendBuf)

#if !EMULATOR
extern void vMI2CDRV_Init(void);
extern bool bMI2CDRV_ReceiveData(uint8_t *pucStr, uint16_t *pusRevLen);
extern bool bMI2CDRV_SendData(uint8_t *pucStr, uint16_t usStrLen);
#else
#define vMI2CDRV_Init(...)
#define bMI2CDRV_SendData(...) true
#define bMI2CDRV_ReceiveData(...) true
#endif

#endif
