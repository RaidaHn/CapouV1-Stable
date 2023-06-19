/**
 * @file lora.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifdef __cplusplus
extern "C" {
#endif

/********* LoRa module hardware SPI bus features parameters section *************/
#define LORADEFAULT_SPI_FREQUENCY      (8E6) 
#define LORADEFAULT_SPI_MSBFIRST       (1)
#define LORADEFAULT_SPI_MODE           (0)
#define LORADEFAULT_SPI_MOSI_PIN       (27)
#define LORADEFAULT_SPI_MISO_PIN       (19)
#define LORADEFAULT_SPI_SCLK_PIN       (5)
#define LORADEFAULT_SPI_SS_PIN         (18)
#define LORADEFAULT_SPI_RESET_PIN      (23)
#define LORADEFAULT_DIO0_PIN           (26)

#define PA_OUTPUT_RFO_PIN              (0)
#define PA_OUTPUT_PA_BOOST_PIN         (1)

/******* Public Function Prototypes Declaration Section **************************/

/**
 * @brief Reseting and enabling the LoRa module for operation.  
 * 
 * @param lFrequency : The carrier frequency in hertz of LoRa radio communications.
 * 
 * @return int  : 0 in case of success, -1 otherwise.
 */
int   LoRaBegin(long lFrequency);

/**
 * @brief Stop the LoRa module by placing it in sleep state.
 *        No further operation can take place.
 *        Release the SPI bus attached to.
 */
void  LoRaEnd(void);

/**
 * @brief Starts a new Tx packet session.
 * 
 * @param iImplicitHeader : TRUE for implicit header mode.
 *                          FALSE for explicit header mode.
 * 
 * @return int : 0 in case of success, -1 if LoRa module can't open a new Tx packet session at this time.
 *                                     Should in this case, retry laster.
 */
int   LoRaBeginPacket(int iImplicitHeader);

/**
 * @brief Closes the current Tx packet session and proccesses its Tx on LoRa radio.
 * 
 * @param iAsync : TRUE for asynchronous Tx mode: not waiting for transmission completed, function promptly returns
 *                                                should use Tx done event processing
 *                 FALSE for synchronous Tx mode: waiting for transmission completed, function retuns at Tx end
 * 
 * @return int : always 0 i.e. success
 */
int   LoRaEndPacket(int iAsync);

/**
 * @brief Retrieves the byte number of a Rx packet.
 * 
 * @param szSize : >0 for implicit header mode 
 * 
 * @return int : byte number of the Rx packet. 
 */
int   LoRaParsePacket(int szSize);

/**
 * @brief Retrieves the received packet RSSI (Received Signal Strength Indicator).
 * 
 * @return int : packet RSSI value in dBm
 */
int   LoRaPacketRssi(void);

/**
 * @brief Retrieves the received packet SNR (Signal to Noise Rate).
 * 
 * @return float : packet SNR value in dB
 */
float LoRaPacketSnr(void);


/**
 * @brief Retrieves the radio link carrier frequency error drift. 
 * 
 * @return long : link carrier frequency error drift in Hz
 */
long  LoRaPacketFrequencyError(void);


/**
 * @brief Wtite a byte of data in the LoRa Tx fifo.
 * 
 * @param u8Byte : the data byte to write
 * 
 * @return size_t : returns the number of byte written
 */
size_t LoRaWriteByte(uint8_t u8Byte);


/**
 * @brief Wtite a byte array of data in the LoRa Tx fifo.
 * 
 * @param pBuffer : the address of the array data byte to write
 * 
 * @param szSize : the number in bytes of the byte array to write 
 * 
 * @return size_t : returns the number of byte written
 */
size_t LoRaWriteBuf(const uint8_t*pBuffer, size_t szSize);


/**
 * @brief Retrieves the number of available Rx data in the Rx fifo.
 * 
 * @return int : the number in bytes of available Rx data
 */
int   LoRaAvailable(void);


/**
 * @brief Return a byte of Rx data from Rx fifo if available.
 *        fifo address pointer moves to the next location.
 * 
 * @return int : the Rx data byte
 */
int   LoRaRead(void);


/**
 * @brief Return a byte of Rx data from Rx fifo if available.
 *        fifo address pointer is unchanged.
 * 
 * @return int : the Rx data byte
 */
int   LoRaPeek(void);


/**
 * @brief Set the callback function pointer for pre-processing on a SPI
 *        transaction session. Commonly for asserting the SPI slave select pin.
 * 
 * @param callback : the callback function pointer
 *                   if NULL, no pre-processing takes place, SPI slave select pin
 *                   is processed by the LoRa library.
 */
void  LoRaPreCallback(void(*callback)(void*));


/**
 * @brief Set the callback function pointer for post-processing on a SPI
 *        transaction session. Commonly for de-asserting the SPI slave select pin.
 * 
 * @param callback : the callback function pointer
 *                   if NULL, no post-processing takes place, SPI slave select pin
 *                   is processed by the LoRa library.
 */
void  LoRaPostCallback(void(*callback)(void*));


/**
 * @brief Set the callback function pointer for processing the Rx done LoRa event. 
 * 
 * @param callback : the callback function pointer
 *                   if NULL, no callback will be processed.
 */
void  LoRaOnReceive(void(*callback)(int));


/**
 * @brief Set the callback function pointer for processing the Tx done LoRa event. 
 * 
 * @param callback : the callback function pointer
 *                   if NULL, no callback will be processed.
 */
void  LoRaOnTxDone(void(*callback)());


/**
 * @brief Places the LoRa module in the continuous receive mode for a receive session on LoRa radio.
 * 
 * @param szSize : >0 for implicit hearder mode 
 */
void  LoRaReceive(int szSize);


/**
 * @brief Places the LoRa module in idle state.
 * 
 */
void  LoRaIdle(void);


/**
 * @brief Places the LoRa module in sleep state.
 * 
 */
void  LoRaSleep(void);


/**
 * @brief Set the LoRa module Tx power.
 * 
 * @param iLevel : the power level in dBm 
 * 
 * @param iOutputPin : for power option (see Semtech SX127x datasheet) 
 */
void  LoRaSetTxPower(int iLevel, int iOutputPin);


/**
 * @brief Set the LoRa module carrier frequency.
 * 
 * @param lFrequency : the carrier frequency in Hertz
 */
void  LoRaSetFrequency(long lFrequency);


/**
 * @brief Set the LoRa radio modulation spreading factor. 
 * 
 * @param iSf : the modulation spreding factor [6 to 12]
 */
void  LoRaSetSpreadingFactor(int iSf);


/**
 * @brief Set the LoRa radio modulation bandwidth. 
 * 
 * @param lSbw : the modulation bandwidth in Hertz
 */
void  LoRaSetSignalBandwidth(long lSbw);


/**
 * @brief Set the LoRa data coding rate for error detection/correction.
 * 
 * @param iDenominator : the data coding rate [5 to 8]
 */
void  LoRaSetCodingRate4(int iDenominator);


/**
 * @brief Set the LoRa packet premble length.
 * 
 * @param lLength : the premble length in bytes [6 to 65535]
 */
void  LoRaSetPrembleLength(long lLength);


/**
 * @brief Set the LoRa synchronization word (byte)
 * 
 * @param iSw ; value of the sync word (default 0x12)
 *              (0x34 resreved for LoRaWan networks)
 */
void  LoRaSetSyncWord(int iSw);


/**
 * @brief Enabling LoRa CRC generation and checking on the Tx data payload.
 */
void  LoRaEnableCrc(void);


/**
 * @brief Disabling LoRa CRC generation and checking on the Tx data payload.
 */
void  LoRaDisableCrc(void);


/**
 * @brief Enabling invert the I Q data path on transceiver chain.
 */
void  LoRaEnableInvertIQ(void);


/**
 * @brief Disabling invert the I Q data path on transceiver chain.
 */
void  LoRaDisableInvertIQ(void);


/**
 * @brief Parametrize the LoRa module over current protection.
 * 
 * @param u8mA : value of max current in mA
 */
void  LoRaSetOCP(uint8_t u8mA);


/**
 * @brief Set the module Tx power amplifier gain value. 
 * 
 * @param u8Gain : gain factor [0 to 6]
 */
void  LoRaSetGain(uint8_t u8Gain);


/**
 * @brief Gives a random number based on the wideband RSSI measurement.
 * 
 * @return uint8_t : a random number
 */
uint8_t LoRaRandom();


/**
 * @brief Set the LoRa module SPI pin connection to the host controler.
 *        Parameters are gpio pin numbers.
 * 
 * @param iSs    : slave select pin
 * @param iReset : reset pin
 * @param iDio0  : DIO0 pin
 */
void  LoRaSetPins(int iSs, int iReset, int iDio0);


/**
 * @brief Set the SPI bus clock frequency.
 * 
 * @param u32Frequency : the SPI bus frequency in hertz
 */
void  LoRaSetSpiFrequency(uint32_t u32Frequency);


/**
 * @brief Dump the entire LoRa register array.
 */
void  LoRaDumpRegisters(void);


/**
 * @brief Dump the entire LoRa fifo data array.
 */
void  LoRaDumpFifo(void);


#ifdef __cplusplus
}
#endif