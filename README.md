# Nordic nRF24L01+ radio driver for STM32

Example usage:
- Initialization:
    ```cpp
    //initialize nRF24L01 module
    nRF24L01.spi = &hspi1;
    nRF24L01.cs.port = nRF24_CS_GPIO_Port;
    nRF24L01.cs.pin = nRF24_CS_Pin;
    nRF24L01.ce.port = nRF24_CE_GPIO_Port;
    nRF24L01.ce.pin = nRF24_CE_Pin;
    nRF24L01.channel = 40;
    nRF24L01.powerLevel = NRF24L01_PA_MAX;
    nRF24L01.dataRate = NRF24L01_2MBPS;
    nRF24L01.payloadSize = 32;
    nRF24L01.dynPayloadEn = NRF24L01_DYN_PYL_DIS;
    nRF24L01.addrWidth = NRF24L01_ADDR_5;
    nRF24L01.CRCLength = NRF24L01_CRC_16;
    nRF24L01.autoAck = NRF24L01_AUT_ACK_EN;
    nRF24L01.retrDelay = NRF24L01_ARD_500;
    nRF24L01.retrCount = NRF24L01_ARC_9;
    memset(nRF24L01.txAddress, 0x00, 5);
    uint64_t address = 0xD21010C334
    memcpy(nRF24L01.rx0Address, &address, 5);

    if (nRF24L01_init(&nRF24L01) == NRF24L01_SUCCESS)
    {
	  printf("nRF24L01 successfully initialized\n");
    }
    else
    {
	  printf("nRF24L01 initialization error\n");
	  while(1);
    }

    ```
- Receive data:
    ```cpp
    while (nRF24L01_available(&nRF24L01))
    {
        uint8_t recBuf[nRF24L01.payloadSize];
        memset(recBuf, 0x00, nRF24L01.payloadSize);
        nRF24L01_read(&nRF24L01, recBuf, nRF24L01.payloadSize);
    }
    ```
- Transmit data:
    ```cpp
    uint8_t txBuffer[nRF24L01.payloadSize];
    uint64_t txAddress = 0xCC23141234;
    nRF24L01_openWritingPipe(nRF24L01, &txAddress);
    nRF24L01_startTransmitter(nRF24L01);

    if (nRF24L01_safeWrite(nRF24L01, txBuffer, nRF24L01.payloadSize, 1, 1, 8))
    {
        //data sent succesfully
    }
    ```