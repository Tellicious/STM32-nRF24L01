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
#define NRF24L01_AUT_ACK_EN   0x3F
#define NRF24L01_AUT_ACK_DIS  0x00

// Address width
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

/* Typedefs ------------------------------------------------------------------*/

/*
* nRF24L01+ return status
*/
typedef enum
{
	NRF24L01_SUCCESS = 0,
	NRF24L01_ERROR = 1,
	NRF24L01_TIMEOUT = 2
} nRF24L01_retStatus_t;

/*
* nRF24L01+ HW pins
*/
typedef struct
{
	int pin;
	void *port;
} nRF24L01_HWPin_t;

/*
 * nRF24L01+ struct
 */
typedef struct
{
  nRF24L01_HWPin_t cs;
  nRF24L01_HWPin_t ce;
  void *spi;
  uint8_t channel, powerLevel, dataRate, payloadSize, addrWidth, CRCLength, autoAck, retrDelay, retrCount, dynPayloadEn;
  uint8_t txAddress[5], rx0Address[5];
} nRF24L01_t;

/* Function prototypes --------------------------------------------------------*/

/*!
 * @brief Initialize nRF24L01+ module according to parameters stored in module structure. Randomizes nRF24L01.txAddress if not already set
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return NRF24L01_SUCCESS if module is initialized succesfully, NRF24L01_ERROR otherwise
 */
nRF24L01_retStatus_t nRF24L01_init(nRF24L01_t *nRF24L01);

/*!
 * @brief Set auto-acknowledge on a specific pipe
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] pipe                target pipe
 * @param[in] enable              1 to enable, 0 to disable
 *
 * @return NRF24L01_SUCCESS if operation is completed succesfully, NRF24L01_ERROR otherwise
 */
nRF24L01_retStatus_t nRF24L01_setAutoAck(nRF24L01_t *nRF24L01, uint8_t pipe, uint8_t enable);

/*!
 * @brief Check if there is data available in FIFO buffer
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return 1 if there is data available, 0 otherwise
 */
uint8_t nRF24L01_available(nRF24L01_t *nRF24L01);

/*!
 * @brief Check what pipe received the available data
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return number of pipe that received the data (0-5). 7 if no data received
 */
uint8_t nRF24L01_pipeAvailable(nRF24L01_t *nRF24L01);

/*!
 * @brief Turn on the module
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_turnOnRadio(nRF24L01_t *nRF24L01);

/*!
 * @brief Turn off the module
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_turnOffRadio(nRF24L01_t *nRF24L01);

/*!
 * @brief Check value of status register
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return raw value of status register
 */
uint8_t nRF24L01_getStatus(nRF24L01_t *nRF24L01);

/*!
 * @brief Flush transmit buffer
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
uint8_t nRF24L01_flushTXBuffer(nRF24L01_t *nRF24L01);

/*!
 * @brief Flush receive buffer
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
uint8_t nRF24L01_flushRXBuffer(nRF24L01_t *nRF24L01);

/*!
 * @brief Mask specified interrupt avoiding them being reflected on IRQ pin
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] tx                  1 to disable transmit data sent interrupt
 * @param[in] maxRt               1 to disable maximum retries interrupt
 * @param[in] rx                  1 to disable rx data received interrupt
 */
void nRF24L01_maskIRQ(nRF24L01_t *nRF24L01, uint8_t tx, uint8_t maxRt, uint8_t rx);

/*!
 * @brief Get current channel
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return channel number
 */
uint8_t nRF24L01_getChannel(nRF24L01_t *nRF24L01);

/*!
 * @brief Get payload length when dynamic payload is enabled
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return payload length
 */
uint8_t nRF24L01_getDynamicPayloadLength(nRF24L01_t *nRF24L01);

/*!
 * @brief Set CRC length according to parameters stored in module structure
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_setCRCLength(nRF24L01_t *nRF24L01);

/*!
 * @brief Get current CRC length
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return NRF24L01_CRC_0, NRF24L01_CRC_8 or NRF24L01_CRC_16
 */
uint8_t nRF24L01_getCRCLength(nRF24L01_t *nRF24L01);

/*!
 * @brief Check received power
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return 1 if received power is greater than -64dBm, 0 otherwise
 */
uint8_t nRF24L01_checkRPD(nRF24L01_t *nRF24L01);

/*!
 * @brief Re-transmit previous TX payload
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_reUseTX(nRF24L01_t *nRF24L01);

/*!
 * @brief Set output power level according to parameters stored in module structure
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_setPAPower(nRF24L01_t *nRF24L01);

/*!
 * @brief Get current power level
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return NRF24L01_PA_MIN, NRF24L01_PA_LOW, NRF24L01_PA_HIGH or NRF24L01_PA_MAX
 */
uint8_t nRF24L01_getPAPower(nRF24L01_t *nRF24L01);

/*!
 * @brief Set data rate according to parameters stored in module structure
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return NRF24L01_SUCCESS if operation is completed succesfully, NRF24L01_ERROR otherwise
 */
nRF24L01_retStatus_t nRF24L01_setDataRate(nRF24L01_t *nRF24L01);

/*!
 * @brief Get current data rate
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return NRF24L01_250KBPS, NRF24L01_1MBPS or NRF24L01_2MBPS
 */
uint8_t nRF24L01_getDataRate(nRF24L01_t *nRF24L01);

/*!
 * @brief Enable acknowledge payload
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_enableAckPayload(nRF24L01_t *nRF24L01);

/*!
 * @brief Enable dynamic acknowledge
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_enableDynamicAck(nRF24L01_t *nRF24L01);

/*!
 * @brief Check what caused an interrupt
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 *
 * @return 5 if TX has been sent, 3 if it reached the max number of retries and 2 if data received (multiplied if more than 1)
 */
uint8_t nRF24L01_whatHappened(nRF24L01_t *nRF24L01);

/*!
  * @brief Clear interrupts
  *
  * @param[in] nRF24L01            pointer to nRF24L01 structure
  * @param[in] tx                  1 to clear transmit data sent interrupt
  * @param[in] maxRt               1 to clear maximum retries interrupt
  * @param[in] rx                  1 to clear rx data received interrupt
  */
void nRF24L01_clearIrq(nRF24L01_t *nRF24L01, uint8_t tx, uint8_t maxRt, uint8_t rx);

/*!
 * @brief Open writing pipe with a specific address
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] address             pointer to TX address
 */
void nRF24L01_openWritingPipe(nRF24L01_t *nRF24L01, const void *address);

/*!
 * @brief Re-open writing pipe with TX address stored in module structure
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_reopenWritingPipe(nRF24L01_t *nRF24L01);

/*!
 * @brief Open a specific reading pipe with a specific address
 *
 * @attention Pipe 1 and 0 cannot use the same address. Pipes 2-5 use the same address as pipe 1, except for the LSB
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] pipe                number of reading pipe (0-5)
 * @param[in] address             pointer to RX address
 */
void nRF24L01_openReadingPipe(nRF24L01_t *nRF24L01, uint8_t pipe, void *address);

/*!
 * @brief Close a specific reading pipe
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] pipe                number of reading pipe (0-5)
 */
void nRF24L01_closeReadingPipe(nRF24L01_t *nRF24L01, uint8_t pipe);

/*!
 * @brief Start transmitter after having configured a writing pipe
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_startTransmitter(nRF24L01_t *nRF24L01);

/*!
 * @brief Start receiver after having configured a reading pipe. Pipe 0 is configured automatically if nRF24L01.rx0Address is set
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 */
void nRF24L01_startReceiver(nRF24L01_t *nRF24L01);

/*!
 * @brief Write data to be sent, without waiting for it to be sent
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] buffer              pointer to data to be sent
 * @param[in] length              length of data to be sent
 * @param[in] multicast           1 to require ACK from receiver, 0 othwerise. (to use 0, nRF24L01_enableDynamicAck(nRF24L01) must be called during initialization)
 * @param[in] wait                1 to empty current buffer before transmitting new data, 0 otherwise
 * @param[in] timeout             timeout in [ms]
 *
 * @return NRF24L01_SUCCESS if data is sent succesfully, NRF24L01_TIMEOUT if timeout is reached
 */
nRF24L01_retStatus_t nRF24L01_write(nRF24L01_t *nRF24L01, const void *buffer, uint8_t length, const uint8_t multicast, const uint8_t wait, uint32_t timeout);

/*!
 * @brief Write data to be sent, waiting for it to be sent
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] buffer              pointer to data to be sent
 * @param[in] length              length of data to be sent
 * @param[in] multicast           1 to require ACK from receiver, 0 othwerise. (to use 0, nRF24L01_enableDynamicAck(nRF24L01) must be called during initialization)
 * @param[in] wait                1 to empty current buffer before transmitting new data, 0 otherwise
 * @param[in] timeout             timeout in [ms]
 *
 * @return NRF24L01_SUCCESS if data is sent succesfully, NRF24L01_TIMEOUT if timeout is reached
 */
nRF24L01_retStatus_t nRF24L01_safeWrite(nRF24L01_t *nRF24L01, const void *buffer, uint8_t length, const uint8_t multicast, const uint8_t wait, uint32_t timeout);

/*!
 * @brief Put TX into Standby I mode. To be used together with nRF24L01_write fcn. Not necessary if safeWrite is used
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] wait_dispatch       1 to wait for all waiting TX data to be sent, 0 otherwise
 * @param[in] timeout             timeout in [ms]
 */
uint8_t nRF24L01_TXStandby(nRF24L01_t *nRF24L01, uint8_t wait_dispatch, uint32_t timeout);

/*!
 * @brief Write a special payload that is re-transmitted by the receiver together with the ACK message. nRF24L01_enableAckPayload(nRF24L01) must be set
 *
 * @attention If ACK packet payload is activated, ACK packets have dynamic payload lengths and the Dynamic Payload Length feature should be enabled for pipe 0 on the PTX and PRX. This is to ensure that they receive the ACK packets with payloads. If the ACK payload is more than 15 byte in 2Mbps mode the ARD must be 500μS or more, and if the ACK payload is more than 5 byte in 1Mbps mode the ARD must be 500μS or more. In 250kbps mode (even when the payload is not in ACK) the ARD must be 500μS or more.
 * 
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] pipe                number of reading pipe (0-5)
 * @param[in] buffer              pointer to data to be sent
 * @param[in] length              length of data to be sent
 * @param[in] timeout             timeout in [ms]
 */
void nRF24L01_writeAckPayload(nRF24L01_t *nRF24L01, uint8_t pipe, void *buffer, uint8_t length);

/*!
 * @brief Read received data
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[out] buffer             pointer to data to be read
 * @param[in] length              length of data to be read
 */
void nRF24L01_read(nRF24L01_t *nRF24L01, void *buffer, uint8_t length);

/*!
 * @brief Get a random TX address
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] workingByte         number of byte to be randomized (0-4)
 */
void nRF24L01_randTXAddr(nRF24L01_t *nRF24L01, uint8_t workingByte);

/*!
 * @brief Match sender address and start receiving on the specified pipe once matched
 *
 * @param[in] nRF24L01            pointer to nRF24L01 structure
 * @param[in] pipe                number of pipe where to perform the match (0-5)
 * @param[out] address            pointer to starting receiver address
 * @param[in] workingByte         byte to be manipulated during search (0-4). If pipe 2-5 is selected, 0 is the only valid option
 * @param[in] delay               delay in [ms] between opening of reading pipe anche check for data availability (depends on transmitter frequency)
 * @param[in] timeout             timeout in [ms]
 *
 * @return NRF24L01_SUCCESS if a match is found, NRF24L01_TIMEOUT if timeout is reached
 */
nRF24L01_retStatus_t nRF24L01_RXMatch(nRF24L01_t *nRF24L01, uint8_t pipe, void *address, uint8_t workingByte, uint32_t delay, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif // __NRF24L01_H__
