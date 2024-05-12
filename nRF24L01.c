/* BEGIN Header */
/**
 ******************************************************************************
 * \file    nRF24L01.c
 * \author  Andrea Vivani
 * \brief             Nordic Semiconductor nRF24L01+ driver
 ******************************************************************************
 * \copyright
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

/* Includes ------------------------------------------------------------------*/

#include "nRF24L01.h"
#include "gpio.h"
#include "spi.h"
#include "string.h"

/* Macros ---------------------------------------------------------------------*/

/*======================================Device Commands===========================================*/
#define W_REGISTER                            0x20
#define REGISTER_MASK                         0x1F
#define ACTIVATE                              0x50
#define R_RX_PL_WID                           0x60
#define R_RX_PAYLOAD                          0x61
#define W_TX_PAYLOAD                          0xA0
#define W_TX_PAYLOAD_NO_ACK                   0xB0
#define W_ACK_PAYLOAD                         0xA8
#define FLUSH_TX                              0xE1
#define FLUSH_RX                              0xE2
#define REUSE_TX_PL                           0xE3
#define NOP                                   0xFF
/*====================================Registers Addresses=========================================*/
#define REG_CONFIG                            0x00
#define REG_EN_AA                             0x01
#define REG_EN_RXADDR                         0x02
#define REG_SETUP_AW                          0x03
#define REG_SETUP_RETR                        0x04
#define REG_RF_CH                             0x05
#define REG_RF_SETUP                          0x06
#define REG_STATUS                            0x07
#define REG_OBSERVE_TX                        0x08
#define REG_RPD                               0x09
#define REG_RX_ADDR_P0                        0x0A
#define REG_RX_ADDR_P1                        0x0B
#define REG_RX_ADDR_P2                        0x0C
#define REG_RX_ADDR_P3                        0x0D
#define REG_RX_ADDR_P4                        0x0E
#define REG_RX_ADDR_P5                        0x0F
#define REG_TX_ADDR                           0x10
#define REG_RX_PW_P0                          0x11
#define REG_RX_PW_P1                          0x12
#define REG_RX_PW_P2                          0x13
#define REG_RX_PW_P3                          0x14
#define REG_RX_PW_P4                          0x15
#define REG_RX_PW_P5                          0x16
#define REG_FIFO_STATUS                       0x17
#define REG_DYNPD                             0x1C
#define REG_FEATURE                           0x1D
/*========================================Bit Mnemonics============================================*/
/* Bit Mnemonics */
#define MASK_RX_DR                            6
#define MASK_TX_DS                            5
#define MASK_MAX_RT                           4
#define EN_CRC                                3
#define CRCO                                  2
#define PWR_UP                                1
#define PRIM_RX                               0
#define ENAA_P5                               5
#define ENAA_P4                               4
#define ENAA_P3                               3
#define ENAA_P2                               2
#define ENAA_P1                               1
#define ENAA_P0                               0
#define ERX_P5                                5
#define ERX_P4                                4
#define ERX_P3                                3
#define ERX_P2                                2
#define ERX_P1                                1
#define ERX_P0                                0
#define AW                                    0
#define ARD                                   4
#define ARC                                   0
#define RF_DR_LOW                             5
#define PLL_LOCK                              4
#define RF_DR_HIGH                            3
#define RX_DR                                 6
#define TX_DS                                 5
#define MAX_RT                                4
#define RX_P_NO                               1
#define TX_FULL                               0
#define PLOS_CNT                              4
#define ARC_CNT                               0
#define TX_REUSE                              6
#define FIFO_FULL                             5
#define TX_EMPTY                              4
#define RX_FULL                               1
#define RX_EMPTY                              0
#define DPL_P5                                5
#define DPL_P4                                4
#define DPL_P3                                3
#define DPL_P2                                2
#define DPL_P1                                1
#define DPL_P0                                0
#define EN_DPL                                2
#define EN_ACK_PAY                            1
#define EN_DYN_ACK                            0
/*========================================AUX MACRO============================================*/
#define NRF24L01_BIT(x)                       (1 << (x))
#define NRF24L01_MAX(a, b)                    ((a) > (b) ? (a) : (b))
#define NRF24L01_MIN(a, b)                    ((a) < (b) ? (a) : (b))
#define NRF24L01_CONSTRAIN(a, min, max)       (((a) < (min)) ? (min) : (((a) > max) ? (max) : (a)))

/* HW Interface  functions ----------------------------------------------------*/

#define NRF24L01_DELAY(x)                     HAL_Delay(x);
#define NRF24L01_TICK                         HAL_GetTick();

#define NRF24L01_WRITE_PIN(port, pin, status) HAL_GPIO_WritePin(port, pin, status)
#define NRF24L01_READ_PIN(port, pin)          HAL_GPIO_ReadPin(port, pin)

#define NRF24L01_PIN_SET                      GPIO_PIN_SET
#define NRF24L01_PIN_RESET                    GPIO_PIN_RESET

/* Private  functions ---------------------------------------------------------*/

/*--------------------Faster SPI transmit----------------------*/
static void SPI_fastTransmit(SPI_HandleTypeDef* xSpiHandle, uint8_t* txBuf, uint16_t length) {
    /* Check if the SPI is already enabled */
    if ((xSpiHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(xSpiHandle);
    }
    while (length--) {
        while (!(xSpiHandle->Instance->SR & SPI_FLAG_TXE)) {}
        *(__IO uint8_t*)&xSpiHandle->Instance->DR = *txBuf++;
    }
    /* Wait BSY flag */
    while (!(xSpiHandle->Instance->SR & SPI_FLAG_TXE)) {}
    while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) {}
    __HAL_SPI_CLEAR_OVRFLAG(xSpiHandle);
}

/*---------------------Faster SPI receive----------------------*/
static void SPI_fastReceive(SPI_HandleTypeDef* xSpiHandle, uint8_t* rxBuf, uint16_t length) {
    /* Check if the SPI is already enabled */
    if ((xSpiHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(xSpiHandle);
    }
    while (length--) {
        while (!(xSpiHandle->Instance->SR & SPI_SR_TXE)) {}
        *(__IO uint8_t*)&xSpiHandle->Instance->DR = 0;
        /* Check the RXNE flag */
        while (!(xSpiHandle->Instance->SR & SPI_FLAG_RXNE)) {}
        *rxBuf++ = *(__IO uint8_t*)&xSpiHandle->Instance->DR;
    }
    /* Wait BSY flag */
    while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) {}
    __HAL_SPI_CLEAR_OVRFLAG(xSpiHandle);
}

/*----------------Faster SPI transmit/receive------------------*/
static void SPI_fastTransmitReceive(SPI_HandleTypeDef* xSpiHandle, uint8_t* txBuf, uint8_t* rxBuf, uint16_t length) {
    /* Check if the SPI is already enabled */
    if ((xSpiHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(xSpiHandle);
    }
    while (length--) {
        while (!(xSpiHandle->Instance->SR & SPI_SR_TXE)) {}
        *(__IO uint8_t*)&xSpiHandle->Instance->DR = *txBuf++;
        while (!(xSpiHandle->Instance->SR & SPI_SR_RXNE)) {}
        *rxBuf++ = *(__IO uint8_t*)&xSpiHandle->Instance->DR;
    }
    /* Wait BSY flag */
    while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) {}
    __HAL_SPI_CLEAR_OVRFLAG(xSpiHandle);
}

/*-------------------Write data to the SPI---------------------*/
static void nRF24L01_SPIWrite(nRF24L01_t* nRF24L01, uint8_t addr, uint8_t* buffer, uint8_t length) {
    addr &= REGISTER_MASK;
    addr |= W_REGISTER;
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_RESET);
    SPI_fastTransmit(nRF24L01->spi, &addr, 1);
    SPI_fastTransmit(nRF24L01->spi, buffer, length);
    //HAL_SPI_Transmit(nRF24L01->spi, &addr, 1, 1000);
    //HAL_SPI_Transmit(nRF24L01->spi, buffer, length, 1000);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_SET);
}

/*-------------------Send command via SPI----------------------*/
static uint8_t nRF24L01_SPICmd(nRF24L01_t* nRF24L01, uint8_t cmd) {
    uint8_t status;
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_RESET);
    SPI_fastTransmitReceive(nRF24L01->spi, &cmd, &status, 1);
    // HAL_SPI_TransmitReceive(nRF24L01->spi, &cmd, &status, 1, 1000);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_SET);
    return status;
}

/*-------------------Read data from the SPI--------------------*/
static void nRF24L01_SPIRead(nRF24L01_t* nRF24L01, uint8_t addr, uint8_t* buffer, uint8_t length) {
    addr &= REGISTER_MASK;
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_RESET);
    SPI_fastTransmit(nRF24L01->spi, &addr, 1);
    SPI_fastReceive(nRF24L01->spi, buffer, length);
    // HAL_SPI_Transmit(nRF24L01->spi, &addr, 1, 1000);
    // HAL_SPI_Receive(nRF24L01->spi, buffer, length, 1000);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_SET);
}

/*-----------------------Write payload-------------------------*/
static void nRF24L01_writePayload(nRF24L01_t* nRF24L01, const void* buffer, uint8_t length, const uint8_t writeType) {
    length = NRF24L01_MIN(length, nRF24L01->payloadSize);
    uint8_t txLen = 1 + (nRF24L01->dynPayloadEn ? length : nRF24L01->payloadSize);
    uint8_t txBuffer[txLen];
    memset(txBuffer, 0x00, txLen);
    txBuffer[0] = writeType;
    memcpy(txBuffer + 1, buffer, length);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_RESET);
    SPI_fastTransmit(nRF24L01->spi, txBuffer, txLen);
    // HAL_SPI_Transmit(nRF24L01->spi, txBuffer, txLen, 1000);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_SET);
}

/*-------------------------Write ACK payload---------------------*/
void nRF24L01_writeAckPayload(nRF24L01_t* nRF24L01, uint8_t pipe, void* buffer, uint8_t length) {
    length = NRF24L01_MIN(length, 32);
    uint8_t txLen = 1 + (nRF24L01->dynPayloadEn ? length : nRF24L01->payloadSize);
    uint8_t txBuffer[txLen];
    memset(txBuffer, 0x00, txLen);
    txBuffer[0] = W_ACK_PAYLOAD | (pipe & 0x07);
    memcpy(txBuffer + 1, buffer, length);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_RESET);
    SPI_fastTransmit(nRF24L01->spi, txBuffer, txLen);
    // HAL_SPI_Transmit(nRF24L01->spi, txBuffer, txLen, 1000);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_SET);
}

/*-----------------------Read payload-------------------------*/
static void nRF24L01_readPayload(nRF24L01_t* nRF24L01, void* buffer, uint8_t length) {
    length = NRF24L01_MIN(length, nRF24L01->payloadSize);
    uint8_t rxLen = ((nRF24L01->dynPayloadEn == NRF24L01_DYN_PYL_EN) ? length : nRF24L01->payloadSize);
    uint8_t txBuffer = R_RX_PAYLOAD;
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_RESET);
    SPI_fastTransmit(nRF24L01->spi, &txBuffer, 1);
    SPI_fastReceive(nRF24L01->spi, buffer, rxLen);
    // HAL_SPI_Transmit(nRF24L01->spi, &txBuffer, 1, 1000);
    // HAL_SPI_Receive(nRF24L01->spi, buffer, rxLen, 1000);
    NRF24L01_WRITE_PIN(nRF24L01->cs.port, nRF24L01->cs.pin, NRF24L01_PIN_SET);
}

/*====================================Public Members=========================================*/
/*------------------------Configure radio-------------------------*/
nRF24L01_retStatus_t nRF24L01_init(nRF24L01_t* nRF24L01) {
    uint8_t buffer = 0;
    NRF24L01_DELAY(100);
    nRF24L01_turnOffRadio(nRF24L01);
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
    if (nRF24L01->txAddress[4] == 0x00) {
        nRF24L01_randTXAddr(nRF24L01, 0);
    }
    /* Set selected channel */
    buffer = NRF24L01_MIN(nRF24L01->channel, 127);
    nRF24L01_SPIWrite(nRF24L01, REG_RF_CH, &buffer, 1);
    /* Set selected payload size */
    nRF24L01->payloadSize = NRF24L01_CONSTRAIN(nRF24L01->payloadSize, 1, 32);
    /* Set selected address width */
    buffer = nRF24L01->addrWidth - 2;
    nRF24L01_SPIWrite(nRF24L01, REG_SETUP_AW, &buffer, 1);
    /* Set auto-ack */
    buffer = nRF24L01->autoAck;
    nRF24L01_SPIWrite(nRF24L01, REG_EN_AA, &buffer, 1);
    /* Enable dynamic payload throughout the system */
    if (nRF24L01->dynPayloadEn) {
        nRF24L01_SPIRead(nRF24L01, REG_FEATURE, &buffer, 1);
        buffer |= NRF24L01_BIT(EN_DPL);
        nRF24L01_SPIWrite(nRF24L01, REG_FEATURE, &buffer, 1);
        /* Enable dynamic payload on all pipes */
        nRF24L01_SPIRead(nRF24L01, REG_DYNPD, &buffer, 1);
        buffer |= NRF24L01_BIT(DPL_P5) | NRF24L01_BIT(DPL_P4) | NRF24L01_BIT(DPL_P3) | NRF24L01_BIT(DPL_P2)
                  | NRF24L01_BIT(DPL_P1) | NRF24L01_BIT(DPL_P0);
        nRF24L01_SPIWrite(nRF24L01, REG_DYNPD, &buffer, 1);
    }
    /* Set CRC length */
    nRF24L01_setCRCLength(nRF24L01);
    /* Set parameters for automatic retransmission */
    buffer = (nRF24L01->retrDelay & 0x0F) << ARD | (nRF24L01->retrCount & 0x0F) << ARC;
    nRF24L01_SPIWrite(nRF24L01, REG_SETUP_RETR, &buffer, 1);
    /* Set output power level */
    nRF24L01_setPAPower(nRF24L01);
    /* Set data rate */
    nRF24L01_setDataRate(nRF24L01);
    /* Reset interrupts */
    nRF24L01_clearIrq(nRF24L01, 1, 1, 1);
    /* Check correct initialization */
    if ((nRF24L01_getChannel(nRF24L01) != nRF24L01->channel)
        || (nRF24L01_getDataRate(nRF24L01) != nRF24L01->dataRate)) {
        return NRF24L01_ERROR;
    }
    /* Flush buffers */
    nRF24L01_flushRXBuffer(nRF24L01);
    nRF24L01_flushTXBuffer(nRF24L01);
    /* Turn on radio */
    nRF24L01_turnOnRadio(nRF24L01); /* Power up by default when begin() is called */
    NRF24L01_DELAY(100);
    /* Enable PTX, do not write CE high so radio will remain in standby I mode (130us max to transition to RX or TX instead of 1500us from turnOnRadio) */
    /* PTX should use only 22uA of power */
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    buffer &= ~NRF24L01_BIT(PRIM_RX);
    nRF24L01_SPIWrite(nRF24L01, REG_CONFIG, &buffer, 1);

    return NRF24L01_SUCCESS;
}

/*----------------------Set auto-ack on a single pipe------------------------*/
nRF24L01_retStatus_t nRF24L01_setAutoAck(nRF24L01_t* nRF24L01, uint8_t pipe, uint8_t enable) {
    uint8_t buffer;
    if (pipe < 6) {
        nRF24L01_SPIRead(nRF24L01, REG_EN_AA, &buffer, 1);
        if (enable) {
            buffer |= NRF24L01_BIT(pipe);
        } else {
            buffer &= ~NRF24L01_BIT(pipe);
        }
        nRF24L01_SPIWrite(nRF24L01, REG_EN_AA, &buffer, 1);
        return NRF24L01_SUCCESS;
    }
    return NRF24L01_ERROR;
}

/*-------------------------Check if data available--------------------------*/
uint8_t nRF24L01_available(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_FIFO_STATUS, &buffer, 1);
    buffer &= NRF24L01_BIT(RX_EMPTY);
    return (!buffer);
}

/*-------------------------Check if data available--------------------------*/
uint8_t nRF24L01_pipeAvailable(nRF24L01_t* nRF24L01) {
    /* returns 7 if no data available */
    return ((nRF24L01_getStatus(nRF24L01) >> RX_P_NO) & 0x07);
}

/*---------------------------Power Up----------------------------*/
void nRF24L01_turnOnRadio(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    if (!(buffer & NRF24L01_BIT(PWR_UP))) {
        buffer |= NRF24L01_BIT(PWR_UP);
        nRF24L01_SPIWrite(nRF24L01, REG_CONFIG, &buffer, 1);
        /* For nNRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode. */
        /* There must be a delay of Tpd2stby (see Table 16.) after the nNRF24L01+ leaves power down mode. */
        NRF24L01_DELAY(5);
    }
}

/*--------------------------Power Down---------------------------*/
void nRF24L01_turnOffRadio(nRF24L01_t* nRF24L01) {
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET); /* Guarantee CE is low on powerDown */
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    buffer &= ~NRF24L01_BIT(PWR_UP);
    nRF24L01_SPIWrite(nRF24L01, REG_CONFIG, &buffer, 1);
}

/*-----------------------Get status value-------------------------*/
uint8_t nRF24L01_getStatus(nRF24L01_t* nRF24L01) { return nRF24L01_SPICmd(nRF24L01, NOP); }

/*---------------------Get FIFO status value----------------------*/
uint8_t nRF24L01_getFIFOStatus(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_FIFO_STATUS, &buffer, 1);
    return buffer;
}

/*---------------------------Flush TX----------------------------*/
uint8_t nRF24L01_flushTXBuffer(nRF24L01_t* nRF24L01) { return nRF24L01_SPICmd(nRF24L01, FLUSH_TX); }

/*---------------------------Flush RX----------------------------*/
uint8_t nRF24L01_flushRXBuffer(nRF24L01_t* nRF24L01) { return nRF24L01_SPICmd(nRF24L01, FLUSH_RX); }

/*------------------------Mask interrupts-------------------------*/
void nRF24L01_maskIRQ(nRF24L01_t* nRF24L01, uint8_t tx, uint8_t maxRt, uint8_t rx) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    buffer = (buffer & 0x07) | (maxRt << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR);
    nRF24L01_SPIWrite(nRF24L01, REG_CONFIG, &buffer, 1);
}

/*-----------------------Get in-use channel-----------------------*/
void nRF24L01_setChannel(nRF24L01_t* nRF24L01, uint8_t channel) {
    nRF24L01_SPIWrite(nRF24L01, REG_RF_CH, &channel, 1);
    return;
}

/*-----------------------Get in-use channel-----------------------*/
uint8_t nRF24L01_getChannel(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_RF_CH, &buffer, 1);
    return buffer;
}

/*------------------Get dynamic payload length------------------*/
uint8_t nRF24L01_getDynamicPayloadLength(nRF24L01_t* nRF24L01) {
    uint8_t length = nRF24L01_SPICmd(nRF24L01, R_RX_PL_WID);
    if (length > 32) {
        nRF24L01_flushRXBuffer(nRF24L01);
        NRF24L01_DELAY(2);
        return 0;
    }
    return length;
}

/*--------------------------Set CRC check-------------------------*/
void nRF24L01_setCRCLength(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    buffer &= ~(NRF24L01_BIT(CRCO) | NRF24L01_BIT(EN_CRC));

    if (nRF24L01->CRCLength == NRF24L01_CRC_0) {
        /* Do nothing, we turned it off above. */
    } else if (nRF24L01->CRCLength == NRF24L01_CRC_8) {
        buffer |= NRF24L01_BIT(EN_CRC);
    } else {
        buffer |= (NRF24L01_BIT(EN_CRC) | NRF24L01_BIT(CRCO));
    }
    nRF24L01_SPIWrite(nRF24L01, REG_CONFIG, &buffer, 1);
}

/*------------------------Get CRC length-----------------------*/
uint8_t nRF24L01_getCRCLength(nRF24L01_t* nRF24L01) {
    uint8_t result = NRF24L01_CRC_0;
    uint8_t buffer, buffer2;
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    nRF24L01_SPIRead(nRF24L01, REG_EN_AA, &buffer2, 1);
    buffer &= (NRF24L01_BIT(CRCO) | NRF24L01_BIT(EN_CRC));

    if (buffer & NRF24L01_BIT(EN_CRC) || buffer2) {
        if (buffer & NRF24L01_BIT(CRCO)) {
            result = NRF24L01_CRC_16;
        } else {
            result = NRF24L01_CRC_8;
        }
    }
    return result;
}

/*------------------------Check received power-----------------------*/
uint8_t nRF24L01_checkRPD(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_RPD, &buffer, 1);
    buffer &= 0x01;
    /* returns 1 if power is greater than -64dBm */
    return buffer;
}

/*---------------Continuously re-transmit TX payload-----------------*/
void nRF24L01_reUseTX(nRF24L01_t* nRF24L01) {
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET); /* Re-Transfer packet */
    nRF24L01_clearIrq(nRF24L01, 0, 1, 0);                                        /* Clear max retry flag */
    nRF24L01_SPICmd(nRF24L01, REUSE_TX_PL);
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET);
}

/*-----------------------Set output power level----------------------*/
void nRF24L01_setPAPower(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_RF_SETUP, &buffer, 1);
    buffer &= 0xF8;
    nRF24L01->powerLevel = NRF24L01_MIN(nRF24L01->powerLevel, 3);
    buffer |= (nRF24L01->powerLevel << 1);
    nRF24L01_SPIWrite(nRF24L01, REG_RF_SETUP, &buffer, 1);
}

/*-----------------------Get output power level----------------------*/
uint8_t nRF24L01_getPAPower(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_RF_SETUP, &buffer, 1);
    buffer = (buffer & 0x06) >> 1;
    return buffer;
}

/*-----------------------Set data transmit rate---------------------*/
nRF24L01_retStatus_t nRF24L01_setDataRate(nRF24L01_t* nRF24L01) {
    uint8_t buffer, buffer2;
    nRF24L01_SPIRead(nRF24L01, REG_RF_SETUP, &buffer, 1);
    nRF24L01->dataRate = NRF24L01_MIN(nRF24L01->dataRate, 2);
    /* Set 1 Mb/s: both HIGH and LOW eq to 0 */
    buffer &= ~(NRF24L01_BIT(RF_DR_LOW) | NRF24L01_BIT(RF_DR_HIGH));
    /* txRxDelay = 85; */ // micros
    if (nRF24L01->dataRate == NRF24L01_250KBPS) {
        buffer |= NRF24L01_BIT(RF_DR_LOW);
        /* txRxDelay = 155; */ // micros
    } else if (nRF24L01->dataRate == NRF24L01_2MBPS) {
        /* Set 2 Mb/s */
        buffer |= NRF24L01_BIT(RF_DR_HIGH);
        /* txRxDelay = 65; */ // micros
    }
    nRF24L01_SPIWrite(nRF24L01, REG_RF_SETUP, &buffer, 1);

    /* Verify */
    nRF24L01_SPIRead(nRF24L01, REG_RF_SETUP, &buffer2, 1);
    if (buffer2 == buffer) {
        return NRF24L01_SUCCESS;
    }
    return NRF24L01_ERROR;
}

/*--------------------------Get data rate-------------------------*/
uint8_t nRF24L01_getDataRate(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_RF_SETUP, &buffer, 1);
    buffer &= (NRF24L01_BIT(RF_DR_LOW) | NRF24L01_BIT(RF_DR_HIGH));
    /* Order matters!!! */
    if (buffer == NRF24L01_BIT(RF_DR_LOW)) {
        /* 10 = 250KBPS */
        return NRF24L01_250KBPS;
    } else if (buffer == NRF24L01_BIT(RF_DR_HIGH)) {
        /* 01 = 2MBPS */
        return NRF24L01_2MBPS;
    } else {
        /* 00 = 1MBPS */
        return NRF24L01_1MBPS;
    }
    return 0;
}

/*------------------------Enable ACK payload----------------------*/
void nRF24L01_enableAckPayload(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    /* enable ack payload and dynamic payload */
    nRF24L01_SPIRead(nRF24L01, REG_FEATURE, &buffer, 1);
    buffer |= NRF24L01_BIT(EN_ACK_PAY) | NRF24L01_BIT(EN_DPL);
    nRF24L01_SPIWrite(nRF24L01, REG_FEATURE, &buffer, 1);
    /* Enable dynamic payload on pipes 0 & 1 */
    nRF24L01_SPIRead(nRF24L01, REG_DYNPD, &buffer, 1);
    buffer |= NRF24L01_BIT(DPL_P5) | NRF24L01_BIT(DPL_P4) | NRF24L01_BIT(DPL_P3) | NRF24L01_BIT(DPL_P2)
              | NRF24L01_BIT(DPL_P1) | NRF24L01_BIT(DPL_P0);
    nRF24L01_SPIWrite(nRF24L01, REG_DYNPD, &buffer, 1);
    nRF24L01->dynPayloadEn = NRF24L01_DYN_PYL_EN;
}

/*------------------------Enable Dynamic ACK----------------------*/
void nRF24L01_enableDynamicAck(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_FEATURE, &buffer, 1);
    buffer |= NRF24L01_BIT(EN_DYN_ACK);
    nRF24L01_SPIWrite(nRF24L01, REG_FEATURE, &buffer, 1);
}

/*-----------------------Clear interrupts-------------------------*/
void nRF24L01_clearIrq(nRF24L01_t* nRF24L01, uint8_t tx, uint8_t maxRt, uint8_t rx) {
    uint8_t buffer = (rx << RX_DR) | (maxRt << MAX_RT) | (tx << TX_DS);
    nRF24L01_SPIWrite(nRF24L01, REG_STATUS, &buffer, 1);
}

/*------------------------Open Writing Pipe----------------------*/
void nRF24L01_openWritingPipe(nRF24L01_t* nRF24L01, const void* address) {
    memcpy(nRF24L01->txAddress, address, nRF24L01->addrWidth);
    nRF24L01_SPIWrite(nRF24L01, REG_TX_ADDR, nRF24L01->txAddress, nRF24L01->addrWidth);
}

/*-----------------------Reopen Writing Pipe---------------------*/
void nRF24L01_reopenWritingPipe(nRF24L01_t* nRF24L01) {
    nRF24L01_SPIWrite(nRF24L01, REG_TX_ADDR, nRF24L01->txAddress, nRF24L01->addrWidth);
}

/*------------------------Open Reading Pipe----------------------*/
void nRF24L01_openReadingPipe(nRF24L01_t* nRF24L01, uint8_t pipe, void* address) {
    /* If this is pipe 0, cache the address.  This is needed because */
    /* openWritingPipe() will overwrite the pipe 0 address, so */
    /* startReceiver() will have to restore it. */
    uint8_t buffer;
    if (pipe == 0) {
        memcpy(nRF24L01->rx0Address, address, nRF24L01->addrWidth);
        nRF24L01_SPIWrite(nRF24L01, REG_RX_ADDR_P0, nRF24L01->rx0Address, nRF24L01->addrWidth);
        nRF24L01_SPIWrite(nRF24L01, REG_RX_PW_P0, &(nRF24L01->payloadSize), 1);
    } else if (pipe == 1) {
        nRF24L01_SPIWrite(nRF24L01, REG_RX_ADDR_P1, address, nRF24L01->addrWidth);
        nRF24L01_SPIWrite(nRF24L01, REG_RX_PW_P1, &(nRF24L01->payloadSize), 1);
    } else if ((pipe > 1) && (pipe < 6)) {
        /* For pipes 2-5, only write the LSB */
        nRF24L01_SPIWrite(nRF24L01, (REG_RX_ADDR_P0 + pipe), address, 1);
        nRF24L01_SPIWrite(nRF24L01, (REG_RX_PW_P0 + pipe), &(nRF24L01->payloadSize), 1);
    }

    nRF24L01_SPIRead(nRF24L01, REG_EN_RXADDR, &buffer, 1);
    buffer |= NRF24L01_BIT(pipe);
    nRF24L01_SPIWrite(nRF24L01, REG_EN_RXADDR, &buffer, 1);
}

/*------------------------Close Reading Pipe----------------------*/
void nRF24L01_closeReadingPipe(nRF24L01_t* nRF24L01, uint8_t pipe) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_EN_RXADDR, &buffer, 1);
    buffer &= ~NRF24L01_BIT(pipe);
    nRF24L01_SPIWrite(nRF24L01, REG_EN_RXADDR, &buffer, 1);
}

/*----------------------Set up as a transmitter------------------*/
void nRF24L01_startTransmitter(nRF24L01_t* nRF24L01) {
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
    /* NRF24L01_DELAYus(txRxDelay); */
    NRF24L01_DELAY(1);
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_FEATURE, &buffer, 1);
    if (buffer & NRF24L01_BIT(EN_ACK_PAY)) {
        /* NRF24L01_DELAYus(txRxDelay); */
        NRF24L01_DELAY(1);
        nRF24L01_flushTXBuffer(nRF24L01);
    }
    nRF24L01_SPIWrite(nRF24L01, REG_RX_ADDR_P0, nRF24L01->txAddress, nRF24L01->addrWidth);
    nRF24L01_SPIWrite(nRF24L01, REG_RX_PW_P0, &(nRF24L01->payloadSize), 1);
    nRF24L01_SPIRead(nRF24L01, REG_EN_RXADDR, &buffer, 1);
    buffer |= NRF24L01_BIT(0);
    nRF24L01_SPIWrite(nRF24L01, REG_EN_RXADDR, &buffer, 1);

    /* Setup as a transmitter */
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    buffer &= ~NRF24L01_BIT(PRIM_RX);
    nRF24L01_SPIWrite(nRF24L01, REG_CONFIG, &buffer, 1);
}

/*----------------------Set up as a receiver------------------*/
void nRF24L01_startReceiver(nRF24L01_t* nRF24L01) {
    uint8_t buffer;
    nRF24L01_SPIRead(nRF24L01, REG_CONFIG, &buffer, 1);
    buffer |= NRF24L01_BIT(PRIM_RX);
    nRF24L01_SPIWrite(nRF24L01, REG_CONFIG, &buffer, 1);
    nRF24L01_clearIrq(nRF24L01, 1, 1, 1);

    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET);
    if (nRF24L01->rx0Address[0] | nRF24L01->rx0Address[1] | nRF24L01->rx0Address[2] | nRF24L01->rx0Address[3]
        | nRF24L01->rx0Address[4]) {
        nRF24L01_SPIWrite(nRF24L01, REG_RX_ADDR_P0, nRF24L01->rx0Address, nRF24L01->addrWidth);
        nRF24L01_SPIWrite(nRF24L01, REG_RX_PW_P0, &(nRF24L01->payloadSize), 1);
        nRF24L01_SPIRead(nRF24L01, REG_EN_RXADDR, &buffer, 1);
        buffer |= NRF24L01_BIT(0);
        nRF24L01_SPIWrite(nRF24L01, REG_EN_RXADDR, &buffer, 1);
    } else {
        nRF24L01_closeReadingPipe(nRF24L01, 0);
    }

    nRF24L01_SPIRead(nRF24L01, REG_FEATURE, &buffer, 1);
    if (buffer & NRF24L01_BIT(EN_ACK_PAY)) {
        nRF24L01_flushTXBuffer(nRF24L01);
    }
}

/*------------------------------Write----------------------------*/
nRF24L01_retStatus_t nRF24L01_write(nRF24L01_t* nRF24L01, const void* buffer, uint8_t length, const uint8_t multicast,
                                    const uint8_t wait, uint32_t timeout) {
    nRF24L01_clearIrq(nRF24L01, 1, 1, 1);
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET); /* Put in Standby-II mode */
    if (wait) {
        /* Empty the current buffer */
        while ((nRF24L01_getStatus(nRF24L01)
                & (NRF24L01_BIT(TX_FULL)))) { /* This will loop and block until TX is no longer full */
            if (nRF24L01_getStatus(nRF24L01) & NRF24L01_BIT(MAX_RT)) { /* If MAX Retries have been reached */
                nRF24L01_clearIrq(nRF24L01, 0, 1, 0);                  /* Clear max retry flag */
                NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET); /* Set re-transmit */
            }
            if (--timeout == 0) { /* If the timeout is reached, flush the TX buffer and return */
                NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
                nRF24L01_flushTXBuffer(nRF24L01);
                return NRF24L01_TIMEOUT;
            }
            NRF24L01_DELAY(1);
        }
    } else {
        /* Flush the TX buffer */
        NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
        nRF24L01_flushTXBuffer(nRF24L01);
    }
    /* Write on the TX buffer and start transmitting */
    nRF24L01_writePayload(nRF24L01, buffer, length, multicast ? W_TX_PAYLOAD : W_TX_PAYLOAD_NO_ACK);
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET);
    return NRF24L01_SUCCESS;
}

/*--------------Write and wait for data to be sent----------------*/
nRF24L01_retStatus_t nRF24L01_safeWrite(nRF24L01_t* nRF24L01, const void* buffer, uint8_t length,
                                        const uint8_t multicast, const uint8_t wait, uint32_t timeout) {
    uint32_t _timeout = timeout;
    uint8_t buffer2;
    nRF24L01_clearIrq(nRF24L01, 1, 1, 1);
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET); /* Put in Standby-II mode */
    if (wait) {
        /* Empty the current buffer */
        while ((nRF24L01_getStatus(nRF24L01)
                & (NRF24L01_BIT(TX_FULL)))) { /* This will loop and block until TX is no longer full */
            if (nRF24L01_getStatus(nRF24L01) & NRF24L01_BIT(MAX_RT)) { /* If MAX Retries have been reached */
                nRF24L01_clearIrq(nRF24L01, 0, 1, 0);                  /* Clear max retry flag */
                NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET); /* Set re-transmit */
            }
            if (--_timeout == 0) { /* If the timeout is reached, flush the TX buffer and return */
                NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
                nRF24L01_flushTXBuffer(nRF24L01);
                return NRF24L01_TIMEOUT;
            }
        }
    } else {
        /* Flush the TX buffer */
        NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
        nRF24L01_flushTXBuffer(nRF24L01);
    }
    /* Write on the TX buffer and start transmitting */
    nRF24L01_writePayload(nRF24L01, buffer, length, multicast ? W_TX_PAYLOAD : W_TX_PAYLOAD_NO_ACK);
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET);
    /* Wait until TX is not empty */
    _timeout = timeout;
    nRF24L01_SPIRead(nRF24L01, REG_FIFO_STATUS, &buffer2, 1);
    while (!(buffer2 & NRF24L01_BIT(TX_EMPTY))) {
        if (nRF24L01_getStatus(nRF24L01) & NRF24L01_BIT(MAX_RT)) { /* If MAX Retries have been reached */
            nRF24L01_clearIrq(nRF24L01, 0, 1, 0);                  /* Clear max retry flag */
            NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET); /* Set re-transmit */
        }
        if (--_timeout == 0) { /* If the timeout is reached, flush the TX buffer and return */
            NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
            nRF24L01_flushTXBuffer(nRF24L01);
            return NRF24L01_TIMEOUT;
        }
        NRF24L01_DELAY(1);
        nRF24L01_SPIRead(nRF24L01, REG_FIFO_STATUS, &buffer2, 1);
    }
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
    /* Clear interrupts */
    nRF24L01_clearIrq(nRF24L01, 1, 1, 1);
    /* If it gets at this point, the transmission has been successful */
    return NRF24L01_SUCCESS;
}

/*------------------Put TX into Standby I mode-------------------*/
/* To be used together with nRF24L01_write fcn. Not necessary if safeWrite is used. */
uint8_t nRF24L01_TXStandby(nRF24L01_t* nRF24L01, uint8_t wait_dispatch, uint32_t timeout) {
    uint8_t buffer;
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET);
    if (wait_dispatch) {
        /* Wait until TX is not empty */
        nRF24L01_SPIRead(nRF24L01, REG_FIFO_STATUS, &buffer, 1);
        while (!(buffer & NRF24L01_BIT(TX_EMPTY))) {
            if (nRF24L01_getStatus(nRF24L01) & NRF24L01_BIT(MAX_RT)) { /* If MAX Retries have been reached */
                nRF24L01_clearIrq(nRF24L01, 0, 1, 0);                  /* Clear max retry flag */
                NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_SET); /* Set re-transmit */
            }
            if (--timeout == 0) { /* If the timeout is reached, flush the TX buffer and return */
                NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
                nRF24L01_flushTXBuffer(nRF24L01);
                return 0;
            }
            NRF24L01_DELAY(1);
            nRF24L01_SPIRead(nRF24L01, REG_FIFO_STATUS, &buffer, 1);
        }
    } else {
        nRF24L01_flushTXBuffer(nRF24L01);
    }
    NRF24L01_WRITE_PIN(nRF24L01->ce.port, nRF24L01->ce.pin, NRF24L01_PIN_RESET);
    /* Clear interrupts */
    nRF24L01_clearIrq(nRF24L01, 1, 1, 1);
    /* If it gets at this point, the transmission has been successful */
    return 1;
}

/*------------------------Read Incoming Data---------------------*/
void nRF24L01_read(nRF24L01_t* nRF24L01, void* buffer, uint8_t length) {
    nRF24L01_readPayload(nRF24L01, buffer, length);
    /* Clear interrupts */
    nRF24L01_clearIrq(nRF24L01, 1, 1, 1);
}

/*-------------------Choose random TX address-------------------*/
void nRF24L01_randTXAddr(nRF24L01_t* nRF24L01, uint8_t workingByte) {
    nRF24L01->txAddress[workingByte] = (uint8_t)NRF24L01_TICK;
}

/*--------------------Match sender address---------------------*/
nRF24L01_retStatus_t nRF24L01_RXMatch(nRF24L01_t* nRF24L01, uint8_t pipe, void* address, uint8_t workingByte,
                                      uint32_t delay, uint32_t timeout) {
    uint8_t* ptr = (uint8_t*)address;
    if (pipe > 1) {
        workingByte = 0;
    }
    timeout /= delay;
    while (1) {
        nRF24L01_openReadingPipe(nRF24L01, pipe, address);
        nRF24L01_startReceiver(nRF24L01);
        NRF24L01_DELAY(delay);
        if (nRF24L01_available(nRF24L01)) {
            nRF24L01_flushRXBuffer(nRF24L01);
            return NRF24L01_SUCCESS;
        } else {
            /* Check Overtime */
            if (--timeout == 0) {
                return NRF24L01_TIMEOUT;
            }
            /* Check address */
            (ptr[workingByte] == 0xFF) ? (ptr[workingByte] = 0x00) : (ptr[workingByte]++);
        }
    }
}
