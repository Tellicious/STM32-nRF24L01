/* BEGIN Header */
/**
 ******************************************************************************
 * @file    nRF24L01.h
 * @author  Andrea Vivani
 * @brief   Nordic Semiconductor nRF24L01+ driver
 ******************************************************************************
 * @copyright
 *
 * Copyright 2023 Andrea Vivani
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 ******************************************************************************
 */
/* END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/

#include "stdint.h"

/* Macros --------------------------------------------------------------------*/

// PA Level
#define NRF24L01_PA_MIN       0x00
#define NRF24L01_PA_LOW       0x01
#define NRF24L01_PA_HIGH      0x02
#define NRF24L01_PA_MAX       0x03

// Data rate
#define NRF24L01_250KBPS      0x00
#define NRF24L01_1MBPS        0x01
#define NRF24L01_2MBPS        0x02

// CRC Length
#define NRF24L01_CRC_0        0x00
#define NRF24L01_CRC_8        0x01
#define NRF24L01_CRC_16       0x02

// Dyn Payload
#define NRF24L01_DYN_PYL_EN   0x01
#define NRF24L01_DYN_PYL_DIS  0x00

// Auto ACK
#define NRF24L01_AUT_ACK_EN   0x01
#define NRF24L01_AUT_ACK_DIS  0x00

// Address width
#define NRF24L01_ADDR_1       0x01
#define NRF24L01_ADDR_2       0x02
#define NRF24L01_ADDR_3       0x03
#define NRF24L01_ADDR_4       0x04
#define NRF24L01_ADDR_5       0x05

// Auto-Retr Delay
#define NRF24L01_ARD_250      0x00
#define NRF24L01_ARD_500      0x01
#define NRF24L01_ARD_750      0x02
#define NRF24L01_ARD_1000     0x03
#define NRF24L01_ARD_1250     0x04
#define NRF24L01_ARD_1500     0x05
#define NRF24L01_ARD_1750     0x06
#define NRF24L01_ARD_2000     0x07
#define NRF24L01_ARD_2250     0x08
#define NRF24L01_ARD_2500     0x09
#define NRF24L01_ARD_2750     0x0A
#define NRF24L01_ARD_3000     0x0B
#define NRF24L01_ARD_3250     0x0C
#define NRF24L01_ARD_3500     0x0D
#define NRF24L01_ARD_3750     0x0E
#define NRF24L01_ARD_4000     0x0F

// Auto-Retr Count
#define NRF24L01_ARC_0        0x00
#define NRF24L01_ARC_1        0x01
#define NRF24L01_ARC_2        0x02
#define NRF24L01_ARC_3        0x03
#define NRF24L01_ARC_4        0x04
#define NRF24L01_ARC_5        0x05
#define NRF24L01_ARC_6        0x06
#define NRF24L01_ARC_7        0x07
#define NRF24L01_ARC_8        0x08
#define NRF24L01_ARC_9        0x09
#define NRF24L01_ARC_10       0x0A
#define NRF24L01_ARC_11       0x0B
#define NRF24L01_ARC_12       0x0C
#define NRF24L01_ARC_13       0x0D
#define NRF24L01_ARC_14       0x0E
#define NRF24L01_ARC_15       0x0F

// STATUS vals
// usage is_tx_sent=!(whatHappened() % NRF24L01_TX_SENT);
#define NRF24L01_TX_SENT      5
#define NRF24L01_MAX_RETR     3
#define NRF24L01_RX_AVAIL     2

/* Function prototypes --------------------------------------------------------*/

/*!
 * @brief Init button structure
 *
 * @param[in] button           pointer to button object
 * @param[in] debounceTicks    number of ticks used for debouncing. Use 20ms as a starting point
 * @param[in] longPressTicks   number of ticks that the button needs to be pressed to detect long-press
 */

void NRF24L01_init(void);
uint8_t NRF24L01_config(uint8_t channel, uint8_t power_level, uint8_t data_rate, uint8_t payload_size, uint8_t dyn_payload, uint8_t addr_width, uint8_t CRC_length, uint8_t auto_ack_all, uint8_t delay_retr, uint8_t count_retr);
void NRF24L01_setAutoAck(uint8_t pipe, uint8_t enable);
uint8_t NRF24L01_available(void);
uint8_t NRF24L01_pipeAvailable(void);
void NRF24L01_turnOnRadio(void);
void NRF24L01_turnOffRadio(void);
uint8_t NRF24L01_getStatus(void);
uint8_t NRF24L01_flushTXBuffer(void);
uint8_t NRF24L01_flushRXBuffer(void);
void NRF24L01_maskIRQ(uint8_t tx, uint8_t fail, uint8_t rx);
uint8_t NRF24L01_getChannel(void);
uint8_t NRF24L01_getDynamicPayloadLength(void);
void NRF24L01_setCRCLength(uint8_t length);
uint8_t NRF24L01_getCRCLength(void);
uint8_t NRF24L01_checkRPD(void);
void NRF24L01_reUseTX(void);
void NRF24L01_setPAPower(uint8_t level);
uint8_t NRF24L01_getPAPower(void);
uint8_t NRF24L01_setDataRate(uint8_t rate);
uint8_t NRF24L01_getDataRate(void);
void NRF24L01_enableAckPayload(void);
void NRF24L01_enableDynamicAck(void);
uint8_t NRF24L01_whatHappened(void);
void NRF24L01_startTransmitter(void);
void NRF24L01_startReceiver(void);
void NRF24L01_openWritingPipe(const void* address);
void NRF24L01_reopenWritingPipe(void);
void NRF24L01_openReadingPipe(uint8_t pipe, const void* address);
void NRF24L01_closeReadingPipe(uint8_t pipe);
void NRF24L01_read(void* buf, uint8_t len);
void NRF24L01_writeAckPayload(uint8_t pipe, const void* buf, uint8_t data_len);
uint8_t NRF24L01_write(const void* buf, uint8_t len, const uint8_t multicast, const uint8_t wait, uint32_t timeout);
uint8_t NRF24L01_safeWrite(const void* buf, uint8_t len, const uint8_t multicast, const uint8_t wait, uint32_t timeout);
void NRF24L01_directWrite(const void* buf, uint8_t len, const uint8_t multicast, uint8_t startTx);
uint8_t NRF24L01_TXStandby(uint8_t wait_dispatch, uint32_t timeout);
uint8_t NRF24L01_RXMatch(void* address, uint8_t working_byte, uint32_t delayms, uint32_t timeout); // matches the sender address, delay and timeout in ms
void NRF24L01_RandTXAddr(void); // choose a random TX address
// Ex private members
uint8_t NRF24L01_readRegister(uint8_t thisRegister);
void NRF24L01_readMultRegister(uint8_t* buffer, uint8_t thisRegister, uint8_t length);
void NRF24L01_writeRegister(uint8_t thisRegister, const uint8_t thisValue);
void NRF24L01_writeMultRegister(uint8_t thisRegister, const void* thisValue, uint8_t length);
uint8_t NRF24L01_sendCMD(uint8_t cmd);
uint8_t NRF24L01_write_payload(const void* buf, uint8_t data_len, const uint8_t writeType);
uint8_t NRF24L01_read_payload(void* buf, uint8_t data_len);
extern uint8_t NRF24L01_tx_address[5]; //TX address

#ifdef __cplusplus
}
#endif

#endif // __NRF24L01_H__
