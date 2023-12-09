//  NRF24.c
//
//
//  Created by Andrea Vivani on 24/11/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#include "NRF24.h"
#include "stdio.h"
#include "ConfigTable.h"
#include "stm32f10x_it.h"
#include "delay.h"
#include "string.h"
#include "ConfigParams.h"
#include "SPI.h"

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define NRF24_CEpin_LOW		GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define NRF24_CEpin_HIGH	GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define NRF24_DELAYms(x)	delay_ms(x)
#define NRF24_DELAYus(x)	delay_us(x)

uint8_t NRF24_payload_size; // Size of payloads
uint8_t NRF24_dynamic_payloads_enabled = 0; // Whether dynamic payloads are enabled
uint8_t NRF24_tx_address[5] = {0x34,0xc3,0x10,0x10,0x00}; //TX address
uint8_t NRF24_rx_address[5] = {0x34,0xc3,0x10,0x10,0x00}; //RX address
//uint64_t address = 0x001010C334;
uint8_t NRF24_rx_0_address[5]; //RX0 address
uint8_t NRF24_addr_width; // The address width to use - 3,4 or 5 bytes
uint32_t NRF24_txRxDelay; // Var for adjusting delays depending on DataRate



//======================================Device Commands===========================================// 
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_TX_PAYLOAD_NO_ACK  0xB0
#define W_ACK_PAYLOAD 0xA8
#define flushTX      0xE1
#define flushRX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF
//====================================Registers Addresses=========================================// 
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA       0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW    0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH       0x05
#define NRF24_REG_RF_SETUP    0x06
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_OBSERVE_TX  0x08
#define NRF24_REG_RPD         0x09
#define NRF24_REG_RX_ADDR_P0  0x0A
#define NRF24_REG_RX_ADDR_P1  0x0B
#define NRF24_REG_RX_ADDR_P2  0x0C
#define NRF24_REG_RX_ADDR_P3  0x0D
#define NRF24_REG_RX_ADDR_P4  0x0E
#define NRF24_REG_RX_ADDR_P5  0x0F
#define NRF24_REG_TX_ADDR     0x10
#define NRF24_REG_RX_PW_P0    0x11
#define NRF24_REG_RX_PW_P1    0x12
#define NRF24_REG_RX_PW_P2    0x13
#define NRF24_REG_RX_PW_P3    0x14
#define NRF24_REG_RX_PW_P4    0x15
#define NRF24_REG_RX_PW_P5    0x16
#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD		   0x1C
#define NRF24_REG_FEATURE	   0x1D
//========================================Bit Mnemonics============================================// 
/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define RF_DR_LOW   5
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0
//========================================AUX MACRO============================================//
#define NRF24_BV(x) (1<<(x))
#define NRF24_MAX(a,b) (a>b?a:b)
#define NRF24_MIN(a,b) (a<b?a:b)
#define NRF24_CONSTRAIN(a,min,max) ((a<min)?(min):((a>max)?(max):(a)))
//==================================Auxiliary Functions========================================//
//--------------------SPI write and read------------------------//
/*uint8_t SPI_RW(uint8_t dat) { 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
    SPI_I2S_SendData(SPI1, dat); 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);  
    return SPI_I2S_ReceiveData(SPI1); 
}*/

//---------------Read one register from the SPI-----------------//
uint8_t NRF24_readRegister(uint8_t thisRegister) {
	uint8_t inByte = 0;   	// incoming byte
	thisRegister &= REGISTER_MASK;
	SPI_CSN_L();					 
	SPI_RW(thisRegister);			  
	inByte = SPI_RW(0);	  
	SPI_CSN_H();
	return(inByte);			// return the read byte
}

//------------Read register with length greater than 1B from the SPI--------------//
void NRF24_readMultRegister(uint8_t* buffer, uint8_t thisRegister, uint8_t length) {
	thisRegister &= REGISTER_MASK;
	SPI_CSN_L();					 
	SPI_RW(thisRegister);			  
	while (length--){
		*buffer++ = SPI_RW(0x00);
	}
	SPI_CSN_H();
	return;
}

//---------------Write one register on the SPI-----------------//
void NRF24_writeRegister(uint8_t thisRegister, const uint8_t thisValue) {
  thisRegister &= REGISTER_MASK;
  thisRegister |= W_REGISTER;
	SPI_CSN_L();					 
	SPI_RW(thisRegister);			  
	SPI_RW(thisValue);	  
	SPI_CSN_H();
  return;
}

//------------Write register with length greater than 1B from the SPI--------------//
void NRF24_writeMultRegister(uint8_t thisRegister, const void* thisValue, uint8_t length) {
  const uint8_t* p = (const uint8_t*) thisValue;
	thisRegister &= REGISTER_MASK;
	thisRegister |= W_REGISTER;
	SPI_CSN_L();					 
	SPI_RW(thisRegister);			  
	while (length--){
		SPI_RW(*p++);
	}	  
	SPI_CSN_H();
	return;
}

//-------------------Send command via SPI----------------------//
uint8_t NRF24_sendCMD(uint8_t cmd){
	uint8_t status;
	SPI_CSN_L();					 
	status = SPI_RW(cmd);			   
	SPI_CSN_H();
	return status;
}

//-----------------------Write payload-------------------------//
uint8_t NRF24_write_payload(const void* buf, uint8_t data_len, const uint8_t writeType){
  uint8_t status;
  const uint8_t* current = (const uint8_t*) buf;
  uint8_t blank_len = NRF24_dynamic_payloads_enabled ? 0 : NRF24_payload_size - data_len;
  data_len = NRF24_MIN(data_len, NRF24_payload_size);
	SPI_CSN_L();					 
	status = SPI_RW(writeType);			  
	while (data_len--){
		SPI_RW(*current++);
	}
  while (blank_len--) {
    SPI_RW(0);
  }	
	SPI_CSN_H();
  return status;
}

//-----------------------Read payload-------------------------//
uint8_t NRF24_read_payload(void* buf, uint8_t data_len){
  uint8_t status;
  uint8_t* current = (uint8_t*) buf;
  uint8_t blank_len = NRF24_dynamic_payloads_enabled ? 0 : NRF24_payload_size - data_len;
  data_len = NRF24_MIN(data_len, NRF24_payload_size);
	SPI_CSN_L();					 
	status = SPI_RW(R_RX_PAYLOAD);			  
	while (data_len--){
		*current++ = SPI_RW(0x00);
	}
  while (blank_len--) {
    SPI_RW(0);
  }	
	SPI_CSN_H();
  return status;
}

//-----------------------Initialization-----------------------//
void NRF24_init(void){
    // Initialize pins
	uint64_t tmp = 0;
	//SPI1_INIT();
	NRF24_CEpin_LOW;
	memcpy(NRF24_rx_0_address, &tmp, 5);
	if (NRF24_tx_address[4] == 0x00){
		NRF24_RandTXAddr();
	}
}

//====================================Public Members=========================================//
//------------------------Configure radio-------------------------//
uint8_t NRF24_config(uint8_t channel, uint8_t power_level, uint8_t data_rate, uint8_t payload_size, uint8_t dyn_payload, uint8_t addr_width, uint8_t CRC_length, uint8_t auto_ack_all, uint8_t delay_retr, uint8_t count_retr){
	NRF24_init();
	// Set selected channel
	NRF24_writeRegister(NRF24_REG_RF_CH, NRF24_MIN(channel, 127));
	// Set selected payload size
	NRF24_payload_size = NRF24_CONSTRAIN(payload_size, 1, 32);
	// Set selected address width
	NRF24_addr_width = NRF24_CONSTRAIN(addr_width, 3, 5);
	NRF24_writeRegister(NRF24_REG_SETUP_AW, (NRF24_addr_width - 2));
	// Set auto-ack
	if (auto_ack_all){
		NRF24_writeRegister(NRF24_REG_EN_AA, 0x3F);
	}
	else{
		NRF24_writeRegister(NRF24_REG_EN_AA, 0);
	}
	// Enable dynamic payload throughout the system
	if (dyn_payload){
      NRF24_writeRegister(NRF24_REG_FEATURE,NRF24_readRegister(NRF24_REG_FEATURE) | NRF24_BV(EN_DPL));
      // Enable dynamic payload on all pipes
      NRF24_writeRegister(NRF24_REG_DYNPD, NRF24_readRegister(NRF24_REG_DYNPD) | NRF24_BV(DPL_P5) | NRF24_BV(DPL_P4) | NRF24_BV(DPL_P3) | NRF24_BV(DPL_P2) | NRF24_BV(DPL_P1) | NRF24_BV(DPL_P0));
      NRF24_dynamic_payloads_enabled = 1;
	}
	// Set CRC length
	NRF24_setCRCLength(CRC_length);
	// Set parameters for automatic retransmission
	NRF24_writeRegister(NRF24_REG_SETUP_RETR, (delay_retr & 0x0F) << ARD | (count_retr & 0x0F) << ARC);
	// Set output power level
	NRF24_setPAPower(power_level);
	// Set data rate
	NRF24_setDataRate(data_rate);
	// Reset interrupts
	NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(RX_DR) | NRF24_BV(TX_DS) | NRF24_BV(MAX_RT));
	// Check correct initialization
	if ((NRF24_getChannel() != channel) || (NRF24_getDataRate() != data_rate)){
	  return 0;
	}
	// Flush buffers
	NRF24_flushRXBuffer();
	NRF24_flushTXBuffer();
	// Turn on radio
	NRF24_turnOnRadio(); //Power up by default when begin() is called
	NRF24_DELAYms(100);
  // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from turnOnRadio )
  // PTX should use only 22uA of power
  NRF24_writeRegister(NRF24_REG_CONFIG, (NRF24_readRegister(NRF24_REG_CONFIG) ) & ~NRF24_BV(PRIM_RX));
  return 1;
}

//----------------------Set auto-ack on a single pipe------------------------//
void NRF24_setAutoAck(uint8_t pipe, uint8_t enable){
  if (pipe <= 6){
    uint8_t AA = NRF24_readRegister(NRF24_REG_EN_AA);
    if(enable){
      AA |= NRF24_BV(pipe) ;
    }
    else{
      AA &= ~NRF24_BV(pipe) ;
    }
    NRF24_writeRegister( NRF24_REG_EN_AA, AA);
  }
}

//-------------------------Check if data available--------------------------//
uint8_t NRF24_available(void){
	return (!(NRF24_readRegister(NRF24_REG_FIFO_STATUS) & NRF24_BV(RX_EMPTY)));
}

//-------------------------Check if data available--------------------------//
uint8_t NRF24_pipeAvailable(void){
	// returns 7 if no data available
  return ((NRF24_getStatus() >> RX_P_NO) & 0x07);
}


//---------------------------Power Up----------------------------//
void NRF24_turnOnRadio(void){
  uint8_t cfg = NRF24_readRegister(NRF24_REG_CONFIG);
  if (!(cfg & NRF24_BV(PWR_UP))){
    NRF24_writeRegister(NRF24_REG_CONFIG, (cfg | NRF24_BV(PWR_UP)));
    // For nNRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
    // There must be a delay of Tpd2stby (see Table 16.) after the nNRF24L01+ leaves power down mode.
	NRF24_DELAYms(5);
  }
}

//--------------------------Power Down---------------------------//
void NRF24_turnOffRadio(void){
  NRF24_CEpin_LOW; // Guarantee CE is low on powerDown
  NRF24_writeRegister(NRF24_REG_CONFIG,NRF24_readRegister(NRF24_REG_CONFIG) & ~NRF24_BV(PWR_UP));
}

//-----------------------Get status value-------------------------//
uint8_t NRF24_getStatus(void){
  return NRF24_sendCMD(NOP);
}

//---------------------------Flush TX----------------------------//
uint8_t NRF24_flushTXBuffer(void){
  return NRF24_sendCMD(flushTX);
}

//---------------------------Flush RX----------------------------//
uint8_t NRF24_flushRXBuffer(void){
  return NRF24_sendCMD(flushRX);
}

//------------------------Mask interrupts-------------------------//
void NRF24_maskIRQ(uint8_t tx, uint8_t fail, uint8_t rx){
	NRF24_writeRegister(NRF24_REG_CONFIG, (NRF24_readRegister(NRF24_REG_CONFIG) ) | fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR);
}

//-----------------------Get in-use channel-----------------------//
uint8_t NRF24_getChannel(void){
  return NRF24_readRegister(NRF24_REG_RF_CH);
}

//------------------Get dynamic payload length------------------//
uint8_t NRF24_getDynamicPayloadLength(void){
  uint8_t result = 0;
  result = NRF24_sendCMD(R_RX_PL_WID);
  if(result > 32){
		NRF24_flushRXBuffer();
		NRF24_DELAYms(2);
	  return 0;
	}
  return result;
}

//--------------------------Set CRC check-------------------------//
void NRF24_setCRCLength(uint8_t length)
{
  uint8_t NRF24_REG_CONFIG_val = NRF24_readRegister(NRF24_REG_CONFIG) & ~( NRF24_BV(CRCO) | NRF24_BV(EN_CRC)) ;
  if (length == 0){
    // Do nothing, we turned it off above.
  }
  else if (length == 1){
    NRF24_REG_CONFIG_val |= NRF24_BV(EN_CRC);
  }
  else{
    NRF24_REG_CONFIG_val |= (NRF24_BV(EN_CRC) | NRF24_BV(CRCO));
  }
  NRF24_writeRegister(NRF24_REG_CONFIG, NRF24_REG_CONFIG_val);
}

//------------------------Get CRC length-----------------------//
uint8_t NRF24_getCRCLength(void){
  uint8_t result = 0;
  uint8_t config = NRF24_readRegister(NRF24_REG_CONFIG) & (NRF24_BV(CRCO) | NRF24_BV(EN_CRC)) ;
  uint8_t AA = NRF24_readRegister(NRF24_REG_EN_AA);
  if (config & NRF24_BV(EN_CRC) || AA)
  {
    if (config & NRF24_BV(CRCO))
      result = 2;
    else
      result = 1;
  }
  return result;
}

//------------------------Check received power-----------------------//
uint8_t NRF24_checkRPD(void){
	// returns 1 if power is greater than -64dBm
  return (NRF24_readRegister(NRF24_REG_RPD) & 0x01) ;
}

//---------------Continuously re-transmit TX payload-----------------//
void NRF24_reUseTX(void){
	NRF24_CEpin_LOW;                 //Re-Transfer packet
	NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(MAX_RT));			//Clear max retry flag
	NRF24_sendCMD(REUSE_TX_PL);
	NRF24_CEpin_HIGH;
}

//-----------------------Set output power level----------------------//
void NRF24_setPAPower(uint8_t level){
  uint8_t setup = NRF24_readRegister(NRF24_REG_RF_SETUP) & 0xF8;
  level = NRF24_MIN(level, 3);
  NRF24_writeRegister(NRF24_REG_RF_SETUP, setup |= (level << 1));
}

//-----------------------Get output power level----------------------//
uint8_t NRF24_getPAPower(void){
  return (NRF24_readRegister(NRF24_REG_RF_SETUP) & 0x06) >> 1 ;
}

//-----------------------Set data transmit rate---------------------//
uint8_t NRF24_setDataRate(uint8_t rate){
  uint8_t result = 0;
  uint8_t setup = NRF24_readRegister(NRF24_REG_RF_SETUP);
	rate = NRF24_MIN(rate, 2);
  // Set 1 Mb/s: both HIGH and LOW eq to 0
  setup &= ~(NRF24_BV(RF_DR_LOW) | NRF24_BV(RF_DR_HIGH));
  NRF24_txRxDelay = 85;
  if(rate == 0){
    setup |= NRF24_BV( RF_DR_LOW ) ;
    NRF24_txRxDelay = 155;
  }
  else if (rate == 2){
    // Set 2 Mb/s
    setup |= NRF24_BV(RF_DR_HIGH);  
    NRF24_txRxDelay = 65;
  }
  NRF24_writeRegister(NRF24_REG_RF_SETUP,setup);
  // Verify our result
  if (NRF24_readRegister(NRF24_REG_RF_SETUP) == setup){
    result = 1;
  }
  return result;
}

//--------------------------Get data rate-------------------------//
uint8_t NRF24_getDataRate(void){
  uint8_t dr = NRF24_readRegister(NRF24_REG_RF_SETUP) & (NRF24_BV(RF_DR_LOW) | NRF24_BV(RF_DR_HIGH));
  // Order matters!!!
  if (dr == NRF24_BV(RF_DR_LOW)){
    // 10 = 250KBPS
    return 0;
  }
  else if (dr == NRF24_BV(RF_DR_HIGH)){
    // 01 = 2MBPS
    return 2;
  }
  else{
    // 00 = 1MBPS
    return 1;
  }
}

//------------------------Enable ACK payload----------------------//
void NRF24_enableAckPayload(void){
  // enable ack payload and dynamic payload
  NRF24_writeRegister(NRF24_REG_FEATURE,NRF24_readRegister(NRF24_REG_FEATURE) | NRF24_BV(EN_ACK_PAY) | NRF24_BV(EN_DPL));
  // Enable dynamic payload on pipes 0 & 1
  NRF24_writeRegister(NRF24_REG_DYNPD, NRF24_readRegister(NRF24_REG_DYNPD) | NRF24_BV(DPL_P5) | NRF24_BV(DPL_P4) | NRF24_BV(DPL_P3) | NRF24_BV(DPL_P2) | NRF24_BV(DPL_P1) | NRF24_BV(DPL_P0));
  NRF24_dynamic_payloads_enabled = 1;
}

//------------------------Enable Dynamic ACK----------------------//
void NRF24_enableDynamicAck(void){
  NRF24_writeRegister(NRF24_REG_FEATURE,NRF24_readRegister(NRF24_REG_FEATURE) | NRF24_BV(EN_DYN_ACK));
}

//---------------Returns what caused an interrupt-----------------//
// Returns 5 if TX has been sent, 3 if it reached the max number of retries and 2 if data received (it multiplies if more than 1)
uint8_t NRF24_whatHappened(void){
  // Read the status & reset the status in one easy call
  uint8_t status_val = NRF24_getStatus();
  NRF24_writeRegister(NRF24_REG_STATUS,NRF24_BV(RX_DR) | NRF24_BV(TX_DS) | NRF24_BV(MAX_RT));
  // Report to the user what happened
  return (((status_val & NRF24_BV(TX_DS)) > 0) * 5) * (((status_val & NRF24_BV(MAX_RT)) > 0) * 3) * (((status_val & NRF24_BV(RX_DR)) > 0) * 2);
}

//----------------------Set up as a transmitter------------------//
void NRF24_startTransmitter(void){
	NRF24_CEpin_LOW;
	NRF24_DELAYus(NRF24_txRxDelay);
	if(NRF24_readRegister(NRF24_REG_FEATURE) & NRF24_BV(EN_ACK_PAY)){
		NRF24_DELAYus(NRF24_txRxDelay);
		NRF24_flushTXBuffer();
	}
	NRF24_writeMultRegister(NRF24_REG_RX_ADDR_P0, &NRF24_tx_address, NRF24_addr_width);
	NRF24_writeRegister(NRF24_REG_RX_PW_P0, NRF24_payload_size);
	NRF24_writeRegister(NRF24_REG_EN_RXADDR, NRF24_readRegister(NRF24_REG_EN_RXADDR) | NRF24_BV(0));
	// Setup as a transmitter
	NRF24_writeRegister(NRF24_REG_CONFIG, (NRF24_readRegister(NRF24_REG_CONFIG) ) & ~NRF24_BV(PRIM_RX));
}

//----------------------Set up as a receiver------------------//
void NRF24_startReceiver(void){
  NRF24_writeRegister(NRF24_REG_CONFIG, (NRF24_readRegister(NRF24_REG_CONFIG) ) | NRF24_BV(PRIM_RX));
  NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(RX_DR) | NRF24_BV(TX_DS) | NRF24_BV(MAX_RT));
  NRF24_CEpin_HIGH;
  if (NRF24_rx_0_address[0] | NRF24_rx_0_address[1] | NRF24_rx_0_address[2] | NRF24_rx_0_address[3] | NRF24_rx_0_address[4]){
    NRF24_writeMultRegister(NRF24_REG_RX_ADDR_P0, &NRF24_rx_0_address, NRF24_addr_width);
    NRF24_writeRegister(NRF24_REG_RX_PW_P0, NRF24_payload_size);
    NRF24_writeRegister(NRF24_REG_EN_RXADDR, NRF24_readRegister(NRF24_REG_EN_RXADDR) | NRF24_BV(0));
  }
  else{
    NRF24_closeReadingPipe(0);
  }
  if(NRF24_readRegister(NRF24_REG_FEATURE) & NRF24_BV(EN_ACK_PAY)){
   NRF24_flushTXBuffer();
  }
}

//------------------------Open Writing Pipe----------------------//
void NRF24_openWritingPipe(const void* address){
  memcpy(NRF24_tx_address, &address, NRF24_addr_width);
  NRF24_writeMultRegister(NRF24_REG_TX_ADDR, &address, NRF24_addr_width);
}

//------------------------Open Writing Pipe----------------------//
void NRF24_reopenWritingPipe(void){
  NRF24_writeMultRegister(NRF24_REG_TX_ADDR, &NRF24_tx_address, NRF24_addr_width);
}

//------------------------Open Reading Pipe----------------------//
void NRF24_openReadingPipe(uint8_t pipe, const void* address){
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startReceiver() will have to restore it.
  if (pipe == 0){
    NRF24_writeMultRegister(NRF24_REG_RX_ADDR_P0, &address, NRF24_addr_width);
    NRF24_writeRegister(NRF24_REG_RX_PW_P0, NRF24_payload_size);
    memcpy(NRF24_rx_0_address, &address, NRF24_addr_width);
  }
  else if (pipe == 1){
    NRF24_writeMultRegister(NRF24_REG_RX_ADDR_P1, &address, NRF24_addr_width);
    NRF24_writeRegister(NRF24_REG_RX_PW_P1, NRF24_payload_size);
  }
  else if ((pipe > 1) && (pipe < 6)){
    // For pipes 2-5, only write the LSB
    NRF24_writeMultRegister((NRF24_REG_RX_ADDR_P0 + pipe), &address, 1);
    NRF24_writeRegister((NRF24_REG_RX_PW_P0 + pipe), NRF24_payload_size);
  }
  NRF24_writeRegister(NRF24_REG_EN_RXADDR, NRF24_readRegister(NRF24_REG_EN_RXADDR) | NRF24_BV(pipe));
}

//------------------------Close Reading Pipe----------------------//
void NRF24_closeReadingPipe(uint8_t pipe){
  NRF24_writeRegister(NRF24_REG_EN_RXADDR, NRF24_readRegister(NRF24_REG_EN_RXADDR) & (~ NRF24_BV(pipe)));
}

//------------------------Read Incoming Data---------------------//
void NRF24_read(void* buf, uint8_t len){
  NRF24_read_payload(buf, len);
  // Clear interrupts
  NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(RX_DR) | NRF24_BV(MAX_RT) | NRF24_BV(TX_DS));
}

//-------------------------Write ACK payload---------------------//
void NRF24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t data_len){
	const uint8_t* current = (const uint8_t*) buf;
	data_len = NRF24_MIN(data_len,32);
	SPI_CSN_L();					 
	SPI_RW(W_ACK_PAYLOAD | ( pipe & 0x07));		  
	while (data_len--){
		SPI_RW(*current++);
	}
	SPI_CSN_H();
	return;
}

//------------------------------Write----------------------------//
uint8_t NRF24_write(const void* buf, uint8_t len, const uint8_t multicast, const uint8_t wait, uint32_t timeout){
	// returns 0 only if it exceeds the timeout
	uint32_t end = millis() + timeout;                //Get the time that the payload transmission started
	NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(RX_DR) | NRF24_BV(TX_DS) | NRF24_BV(MAX_RT));      //Clear interrupts
	NRF24_CEpin_HIGH; // Put in Standby-II mode
	if (wait){
	// Empty the current buffer
	while((NRF24_getStatus()  & (NRF24_BV(TX_FULL)))) {     // This will loop and block until TX is no longer full
	  if(NRF24_getStatus() & NRF24_BV(MAX_RT)){           // If MAX Retries have been reached
		NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(MAX_RT));      //Clear max retry flag
		NRF24_CEpin_HIGH;   // Set re-transmit
	  }
	  if(millis() > end){ // If the timeout is reached, flush the TX buffer and return
		NRF24_CEpin_LOW;
		NRF24_flushTXBuffer();
		return 0;
	  }
	}
	}
	else{
	// Flush the TX buffer
	NRF24_CEpin_LOW;
	NRF24_flushTXBuffer();
	}
	// Write on the TX buffer and start transmitting
	NRF24_directWrite(buf, len, multicast, 1);
	return 1;
}

//--------------Write and wait for data to be sent----------------//
uint8_t NRF24_safeWrite(const void* buf, uint8_t len, const uint8_t multicast, const uint8_t wait, uint32_t timeout){
  // returns 0 only if it exceeds the timeout
  uint32_t end = millis() + timeout;                //Get the time that the payload transmission started
  NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(RX_DR) | NRF24_BV(TX_DS) | NRF24_BV(MAX_RT));      //Clear interrupts
  NRF24_CEpin_HIGH; // Put in Standby-II mode
  if (wait){
    // Empty the current buffer
    while((NRF24_getStatus()  & (NRF24_BV(TX_FULL)))) {     // This will loop and block until TX is no longer full
      if(NRF24_getStatus() & NRF24_BV(MAX_RT)){           // If MAX Retries have been reached
        NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(MAX_RT));      //Clear max retry flag
        NRF24_CEpin_HIGH;   // Set re-transmit
      }
	  if(millis() > end){ // If the timeout is reached, flush the TX buffer and return
        NRF24_CEpin_LOW;
        NRF24_flushTXBuffer();
        return 0;
      }
    }
  }
  else{
    // Flush the TX buffer
    NRF24_CEpin_LOW;
    NRF24_flushTXBuffer();
  }
  // Write on the TX buffer and start transmitting
  NRF24_directWrite(buf, len, multicast, 1);
  // Wait until TX is not empty
  while(!(NRF24_readRegister(NRF24_REG_FIFO_STATUS)  & NRF24_BV(TX_EMPTY))) {
    if(NRF24_getStatus() & NRF24_BV(MAX_RT)){           // If MAX Retries have been reached
      NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(MAX_RT));      //Clear max retry flag
      NRF24_CEpin_HIGH;   // Set re-transmit
    }
	  if(millis() > end){ // If the timeout is reached, flush the TX buffer and return
      NRF24_CEpin_LOW;
      NRF24_flushTXBuffer();
      return 0;
    }
  }
  NRF24_CEpin_LOW;
  // Clear interrupts
  NRF24_writeRegister(NRF24_REG_STATUS,NRF24_BV(RX_DR) | NRF24_BV(TX_DS) | NRF24_BV(MAX_RT));
  // If it gets at this point, the transmission has been successful
  return 1;
}

//---------------Write data to FIFO and start TX-----------------//
void NRF24_directWrite(const void* buf, uint8_t len, const uint8_t multicast, uint8_t startTx){
  NRF24_write_payload(buf, len, multicast ? W_TX_PAYLOAD : W_TX_PAYLOAD_NO_ACK);
  if(startTx){
    NRF24_CEpin_HIGH;
  }
}

//------------------Put TX into Standby I mode-------------------//
// To be used together with Write fcn. Not necessary if safeWrite is used.
uint8_t NRF24_TXStandby(uint8_t wait_dispatch, uint32_t timeout){
  uint32_t end = millis() + timeout;                //Get the time that the payload transmission started
  NRF24_CEpin_HIGH;
  if (wait_dispatch){
    // Wait until TX is not empty
    while(!(NRF24_readRegister(NRF24_REG_FIFO_STATUS)  & NRF24_BV(TX_EMPTY))) {
      if(NRF24_getStatus() & NRF24_BV(MAX_RT)){           // If MAX Retries have been reached
        NRF24_writeRegister(NRF24_REG_STATUS, NRF24_BV(MAX_RT));      //Clear max retry flag
        NRF24_CEpin_HIGH;   // Set re-transmit
      }
	  if(millis() > end){ // If the timeout is reached, flush the TX buffer and return
        NRF24_CEpin_LOW;
        NRF24_flushTXBuffer();
        return 0;
      }
    }
  }
  else{
    NRF24_flushTXBuffer();
  }
  NRF24_CEpin_LOW;
  // Clear interrupts
  NRF24_writeRegister(NRF24_REG_STATUS,NRF24_BV(RX_DR) | NRF24_BV(TX_DS) | NRF24_BV(MAX_RT));
  // If it gets at this point, the transmission has been successful
  return 1;
}

//--------------------Match sender address---------------------//
/*uint8_t NRF24_RXMatch(void* address, uint8_t working_byte, uint32_t delay, uint32_t timeout){
uint64_t* ptr = (uint64_t*) address;
uint8_t shift = (working_byte - 1) * 8;
uint64_t add_comp1 = (uint64_t) 0xFF << shift;
uint64_t add_comp2 = ~add_comp1;
uint32_t end = millis() + timeout;
while (1) {
  NRF24_openReadingPipe(1, &address);
  NRF24_startReceiver();
  NRF24_DELAYms(delay);
  if (NRF24_available()) {
    NRF24_flushRXBuffer();
    return 1;
  }
  else {
    // Check Overtime
    if (millis() > end) {
        return 0;
      }
      // Check address
      ((*ptr & add_comp1) == add_comp1) ? (*ptr &= add_comp2) : (*ptr += (uint64_t) 1 << shift);
    }
  }
}*/

uint8_t NRF24_RXMatch(void* address, uint8_t working_byte, uint32_t delay, uint32_t timeout){
uint8_t* ptr = (uint8_t*) address;
uint32_t end = millis() + timeout;
while (1) {
  NRF24_openReadingPipe(1, &address);
  NRF24_startReceiver();
  NRF24_DELAYms(delay);
  if (NRF24_available()) {
    NRF24_flushRXBuffer();
    return 1;
  }
  else {
    // Check Overtime
    if (millis() > end) {
        return 0;
      }
      // Check address
      (ptr[working_byte] == 0xFF)? (ptr[working_byte] = 0x00) : (ptr[working_byte]++);
    }
  }
}

//-------------------Choose random TX address-------------------//
void NRF24_RandTXAddr(void){
	unsigned time = micros();
	NRF24_tx_address[4] = (u8)time;
	SaveParamsToEEPROM();
}
