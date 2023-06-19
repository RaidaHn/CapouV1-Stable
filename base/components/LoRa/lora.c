/**
 * @file lora.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "loraRegs.h"
#include "lora.h"


/***************** Module TAG string declaration *************************************/
static const char *TAG = "<LoRa Entity>";

/***************** Private Function Prototypes Declaration Section *******************/
void _LoRaExplicitHeaderMode(void);
void _LoRaImplicitHeaderMode(void);

void _LoRaHandleDio0Rise(void);
int  _LoRaIsTransmitting(void);

int  _LoRaGetSpreadingFactor(void);
long _LoRaGetSignalBandwidth(void);

void _LoRaSetLdoFlag(void);

uint8_t _LoRaReadRegister(uint8_t u8Address);
void    _LoRaWriteRegister(uint8_t u8Address, uint8_t u8Value);

 static void IRAM_ATTR _LoRaOnDio0Rise(void*p);


/****************** LoRa Entity Structure Def & LoRa Entity Implementation *************/
static struct s_lora {
    struct s_spiSettings{
        uint32_t    u32Clock;
        uint8_t     u8BitOrder;
        uint8_t     u8DataMode;
    }                   m_spiSettings;
    spi_device_handle_t m_spiDeviceHandle;
    int                 m_iSs;
    int                 m_iReset;
    int                 m_iDio0;
    long                m_lFrequency;
    int                 m_iPacketIndex;
    int                 m_iImplicitHeaderMode;
    uint32_t            m_u32Timeout;
    void                (*m_onReceiveCallback)(int);
    void                (*m_onTxDoneCallback)(void);
    void                (*m_preCallback)(void*);
    void                (*m_postCallback)(void*);
}lora = {
    .m_spiSettings          = { LORADEFAULT_SPI_FREQUENCY,
                                LORADEFAULT_SPI_MSBFIRST,
                                LORADEFAULT_SPI_MODE      },
    .m_spiDeviceHandle      = 0,
    .m_iSs                  = LORADEFAULT_SPI_SS_PIN,
    .m_iReset               = LORADEFAULT_SPI_RESET_PIN,
    .m_iDio0                = LORADEFAULT_DIO0_PIN,
    .m_lFrequency           = 0,
    .m_iPacketIndex         = 0,
    .m_iImplicitHeaderMode  = 0,
    .m_u32Timeout           = 0,
    .m_onReceiveCallback    = NULL,
    .m_onTxDoneCallback     = NULL,
    .m_preCallback          = NULL,
    .m_postCallback         = NULL,
};

/***************** lora entity fields access macros definition **************/
#define m_spiSettings               lora.m_spiSettings
#define m_spiDeviceHandle           lora.m_spiDeviceHandle
#define m_iSs                       lora.m_iSs
#define m_iReset                    lora.m_iReset
#define m_iDio0                     lora.m_iDio0
#define m_lFrequency                lora.m_lFrequency
#define m_iPacketIndex              lora.m_iPacketIndex
#define m_iImplicitHeaderMode       lora.m_iImplicitHeaderMode
#define m_u32Timeout                lora.m_u32Timeout
#define m_onReceiveCallback         lora.m_onReceiveCallback
#define m_onTxDoneCallback          lora.m_onTxDoneCallback
#define m_preCallback               lora.m_preCallback
#define m_postCallback              lora.m_postCallback

/****************** Bits Manipulation Macros ********************************/
#define bitRead(value, bit)             (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)              ((value) |= (1UL << (bit)))
#define bitClear(value, bit)            ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit)           ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue)  ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))


/***************** Public Function Implementation Section *******************/
int   LoRaBegin(long lFrequency){
    m_spiSettings.u32Clock = LORADEFAULT_SPI_FREQUENCY;
    m_spiSettings.u8BitOrder = LORADEFAULT_SPI_MSBFIRST;
    m_spiSettings.u8DataMode = LORADEFAULT_SPI_MODE;
    m_spiDeviceHandle = NULL;
    m_lFrequency = 0;
    m_iPacketIndex = 0;
    m_iImplicitHeaderMode = 0;
    m_u32Timeout = 0;
    m_onReceiveCallback = NULL;
    m_onTxDoneCallback = NULL;
    m_preCallback = NULL;
    m_postCallback = NULL;

    /**** SPI Slave Select Pin Configuration *************************/
    gpio_reset_pin(m_iSs);
    gpio_set_direction(m_iSs, GPIO_MODE_OUTPUT);
    gpio_set_level(m_iSs, 1);

    /**** Reset Pin Configuration and SX127x Reset Signal ************/
    if(m_iReset!=-1){
        gpio_reset_pin(m_iReset);
        gpio_set_direction(m_iReset, GPIO_MODE_OUTPUT);
        gpio_set_level(m_iReset, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(m_iReset, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    /**** Configuring SPI Interface *********************************/
    esp_err_t err;

    spi_bus_config_t bus_config = {
        .mosi_io_num    = LORADEFAULT_SPI_MOSI_PIN,
        .miso_io_num    = LORADEFAULT_SPI_MISO_PIN,
        .sclk_io_num    = LORADEFAULT_SPI_SCLK_PIN,
        .quadwp_io_num  = -1,
        .quadhd_io_num  = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE
    };
    err = spi_bus_initialize(HSPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(err);

    spi_device_interface_config_t device_config = {
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = m_spiSettings.u8DataMode,           /* Mode 0: CPOL=0 CPHA=0 */
        .spics_io_num = m_iSs,
        .clock_speed_hz = m_spiSettings.u32Clock,   /* FSCK max: 10 MHz */
        .queue_size = 1,
        .input_delay_ns = 0,
        .pre_cb = (transaction_cb_t)m_preCallback,
        .post_cb = (transaction_cb_t)m_postCallback
    };
    err = spi_bus_add_device(HSPI_HOST, &device_config, &m_spiDeviceHandle);
    ESP_ERROR_CHECK(err);

    /**** Check version *********************/
    uint8_t u8Version = _LoRaReadRegister(REG_VERSION); 
    if(u8Version!=SEMTECH_SX127X_VERSION) {
        ESP_LOGE(TAG, "\bLoRaBegin(): SX127x bad version: 0x%02X! Must be: 0x%02X.", u8Version, SEMTECH_SX127X_VERSION);
        return -1;
    }

    /**** Put in sleep mode *****************/
    LoRaSleep();

    /**** Set frequency *********************/
    LoRaSetFrequency(lFrequency);

    /**** Set base addresses ****************/
    _LoRaWriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
    _LoRaWriteRegister(REG_FIFO_RX_BASE_ADDR, 0);

    /**** Set LNA boost *********************/
    _LoRaWriteRegister(REG_LNA, _LoRaReadRegister(REG_LNA) | 0x03);

    /**** Set ouput power to 17 dBm *********/
    LoRaSetTxPower(17, PA_OUTPUT_PA_BOOST_PIN);

    /**** Put in stanby mode ****************/
    LoRaIdle();

    return 0;
}

void  LoRaEnd(void){
    /**** Put in sleep mode *****************/
    LoRaSleep();

    /**** Stop SPI **************************/
    spi_bus_remove_device(m_spiDeviceHandle);
    esp_err_t err = spi_bus_free(HSPI_HOST);
    ESP_ERROR_CHECK(err);
}

int   LoRaBeginPacket(int iImplicitHeader){
    if(_LoRaIsTransmitting()) return -1;

    /**** Put in stanby mode ****************/
    LoRaIdle();
    if(iImplicitHeader){
        _LoRaImplicitHeaderMode();
    }
    else{
        _LoRaExplicitHeaderMode();
    }

    /**** Reset FIFO address and payload length **********/
    _LoRaWriteRegister(REG_FIFO_ADDR_PTR, 0);
    _LoRaWriteRegister(REG_PAYLOAD_LENGTH, 0);

    return 0;
}

int   LoRaEndPacket(int iAsync){
    if(iAsync && m_onTxDoneCallback){
        _LoRaWriteRegister(REG_DIO_MAPPING_1, 0x40);    /* DIO0 => TXDONE */
    }

    /**** Put in Tx mode ***************/
    _LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if(!iAsync){
        /**** Wait for TX done ********/
        while((_LoRaReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0){
            /* ToDo: yield() C ANSI equivalent */
        }
        /**** Clear IRQ's **************/
        _LoRaWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 0;
}

int   LoRaParsePacket(int szSize){
    int iPacketLength = 0;
    int iIrqFlags = _LoRaReadRegister(REG_IRQ_FLAGS);

    if(szSize>0){
        _LoRaImplicitHeaderMode();
        _LoRaWriteRegister(REG_PAYLOAD_LENGTH, szSize & 0xff);
    }
    else{
        _LoRaExplicitHeaderMode();
    }

    /**** Clear IRQ's **************/
    _LoRaWriteRegister(REG_IRQ_FLAGS, iIrqFlags);

    if((iIrqFlags & IRQ_RX_DONE_MASK) && (iIrqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0){
        /**** Received packet ******/
        m_iPacketIndex = 0;

        /**** Read packet length ***/
        if(m_iImplicitHeaderMode){
            iPacketLength = _LoRaReadRegister(REG_PAYLOAD_LENGTH);
        }
        else{
            iPacketLength = _LoRaReadRegister(REG_RX_NB_BYTES);
        }

        /**** Set FIFO address to current RX address *********/
        _LoRaWriteRegister(REG_FIFO_ADDR_PTR, _LoRaReadRegister(REG_FIFO_RX_CURRENT_ADDR));

        /**** Put in stanby mode *****************************/
        LoRaIdle();
    }
    else if(_LoRaReadRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
        /**** Not currently in RX mode ****/
        /**** Reset FIFO address **********/
        _LoRaWriteRegister(REG_FIFO_ADDR_PTR, 0);

        /**** Put in single RX mode *******/
        _LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return iPacketLength;
}

int   LoRaPacketRssi(void){
    return (_LoRaReadRegister(REG_PKT_RSSI_VALUE) - (m_lFrequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRaPacketSnr(void){
    return ((int8_t)_LoRaReadRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long  LoRaPacketFrequencyError(void){
    int32_t i32FreqError = 0;
    i32FreqError = (int32_t)(_LoRaReadRegister(REG_FREQ_ERROR_MSB) & 0b111);
    i32FreqError <<= 8L;
    i32FreqError += (int32_t)(_LoRaReadRegister(REG_FREQ_ERROR_MID));
    i32FreqError <<= 8L;
    i32FreqError += (int32_t)(_LoRaReadRegister(REG_FREQ_ERROR_LSB));

    if (_LoRaReadRegister(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
        i32FreqError -= 524288; // B1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = (((float)i32FreqError * (1L << 24)) / fXtal) * (_LoRaGetSignalBandwidth() / 500000.0f); // p. 37

    return (long)fError;
}

size_t LoRaWriteByte(uint8_t u8Byte){
    return LoRaWriteBuf(&u8Byte, sizeof u8Byte);
}

size_t LoRaWriteBuf(const uint8_t*pBuffer, size_t szSize){
    int iCurrentLength = _LoRaReadRegister(REG_PAYLOAD_LENGTH);

    /**** Check size ****************/
    if((iCurrentLength + szSize) > MAX_PKT_LENGTH){
        szSize = MAX_PKT_LENGTH - iCurrentLength; 
    }

    /**** Write data ****************/
    for(size_t k=0; k<szSize; ++k){
        _LoRaWriteRegister(REG_FIFO, pBuffer[k]);
    }

    /**** Update length *************/
    _LoRaWriteRegister(REG_PAYLOAD_LENGTH, iCurrentLength + szSize);

    return szSize;
}

int   LoRaAvailable(void){
    return (_LoRaReadRegister(REG_RX_NB_BYTES) - m_iPacketIndex);
}

int   LoRaRead(void){
    if(!LoRaAvailable()) return -1;
    m_iPacketIndex++;
    return _LoRaReadRegister(REG_FIFO);
}

int   LoRaPeek(void){
    if(!LoRaAvailable()) return -1;

    /**** Store current FIFO address ********/
    int iCurrentAddress = _LoRaReadRegister(REG_FIFO_ADDR_PTR);

    /**** Read ******/
    uint8_t u8Byte = _LoRaReadRegister(REG_FIFO);

    /**** Restore FIFO address **************/
    _LoRaWriteRegister(REG_FIFO_ADDR_PTR, iCurrentAddress);

    return u8Byte;
}

void  LoRaPreCallback(void(*callback)(void*)){
    m_preCallback = callback;
}

void  LoRaPostCallback(void(*callback)(void*)){
    m_postCallback = callback;
}


void  LoRaOnReceive(void(*callback)(int)){
    m_onReceiveCallback = callback;

    if(callback){
        /* ToDo: Configure interrupt
         */
        esp_err_t err;
        gpio_pad_select_gpio(m_iDio0);
        gpio_set_direction(m_iDio0, GPIO_MODE_INPUT);
        gpio_set_pull_mode(m_iDio0, GPIO_PULLUP_ONLY);
        err = gpio_set_intr_type(m_iDio0, GPIO_INTR_POSEDGE);
        ESP_ERROR_CHECK(err);
        err = gpio_isr_handler_add(m_iDio0, _LoRaHandleDio0Rise, NULL);
        ESP_ERROR_CHECK(err);
        gpio_intr_enable(m_iDio0);
    }
    else{
        gpio_intr_disable(m_iDio0);
        gpio_uninstall_isr_service();
        gpio_isr_handler_remove(m_iDio0);
    }
}

void  LoRaOnTxDone(void(*callback)()){
    m_onTxDoneCallback = callback;

    if(callback){
        gpio_set_direction(m_iDio0, GPIO_MODE_INPUT);
        /* ToDo: Configure interrupt
         */
        esp_err_t err;
        gpio_pad_select_gpio(m_iDio0);
        gpio_set_direction(m_iDio0, GPIO_MODE_INPUT);
        gpio_set_pull_mode(m_iDio0, GPIO_PULLUP_ONLY);
        err = gpio_set_intr_type(m_iDio0, GPIO_INTR_POSEDGE);
        ESP_ERROR_CHECK(err);
        err = gpio_isr_handler_add(m_iDio0, _LoRaHandleDio0Rise, NULL);
        ESP_ERROR_CHECK(err);
        gpio_intr_enable(m_iDio0);
    }
    else{
        gpio_intr_disable(m_iDio0);
        gpio_uninstall_isr_service();
        gpio_isr_handler_remove(m_iDio0);
    }
}

void  LoRaReceive(int szSize){
    _LoRaWriteRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE
    if (szSize > 0) {
        _LoRaImplicitHeaderMode();
        _LoRaWriteRegister(REG_PAYLOAD_LENGTH, szSize & 0xff);
    } else {
        _LoRaExplicitHeaderMode();
    }
    _LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void  LoRaIdle(void){
    _LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void  LoRaSleep(void){
    _LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void  LoRaSetTxPower(int iLevel, int iOutputPin){
    if (PA_OUTPUT_RFO_PIN == iOutputPin) {
    // RFO
    if (iLevel < 0) {
        iLevel = 0;
    } else if (iLevel > 14) {
        iLevel = 14;
    }

    _LoRaWriteRegister(REG_PA_CONFIG, 0x70 | iLevel);
    } else {
    // PA BOOST
    if (iLevel > 17) {
        if (iLevel > 20) {
        iLevel = 20;
        }

        // subtract 3 from iLevel, so 18 - 20 maps to 15 - 17
        iLevel -= 3;

        // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
        _LoRaWriteRegister(REG_PA_DAC, 0x87);
        LoRaSetOCP(140);
    } else {
        if (iLevel < 2) {
        iLevel = 2;
        }
        //Default value PA_HF/LF or +17dBm
        _LoRaWriteRegister(REG_PA_DAC, 0x84);
        LoRaSetOCP(100);
    }

    _LoRaWriteRegister(REG_PA_CONFIG, PA_BOOST | (iLevel - 2));
    }    
}

void  LoRaSetFrequency(long lFrequency){
    m_lFrequency = lFrequency;

    uint64_t frf = ((uint64_t)lFrequency << 19) / 32000000;

    _LoRaWriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    _LoRaWriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    _LoRaWriteRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));    
}

void  LoRaSetSpreadingFactor(int iSf){
    if (iSf < 6) {
        iSf = 6;
    } else if (iSf > 12) {
        iSf = 12;
    }

    if (iSf == 6) {
        _LoRaWriteRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        _LoRaWriteRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        _LoRaWriteRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        _LoRaWriteRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    _LoRaWriteRegister(REG_MODEM_CONFIG_2, (_LoRaReadRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((iSf << 4) & 0xf0));
    _LoRaSetLdoFlag();    
}

void  LoRaSetSignalBandwidth(long lSbw){
    int bw;

    if (lSbw <= 7.8E3) {
    bw = 0;
    } else if (lSbw <= 10.4E3) {
    bw = 1;
    } else if (lSbw <= 15.6E3) {
    bw = 2;
    } else if (lSbw <= 20.8E3) {
    bw = 3;
    } else if (lSbw <= 31.25E3) {
    bw = 4;
    } else if (lSbw <= 41.7E3) {
    bw = 5;
    } else if (lSbw <= 62.5E3) {
    bw = 6;
    } else if (lSbw <= 125E3) {
    bw = 7;
    } else if (lSbw <= 250E3) {
    bw = 8;
    } else /*if (lSbw <= 250E3)*/ {
    bw = 9;
    }

    _LoRaWriteRegister(REG_MODEM_CONFIG_1, (_LoRaReadRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    _LoRaSetLdoFlag();
}

void  LoRaSetCodingRate4(int iDenominator){
    if (iDenominator < 5) {
        iDenominator = 5;
    } else if (iDenominator > 8) {
        iDenominator = 8;
    }

    int cr = iDenominator - 4;

    _LoRaWriteRegister(REG_MODEM_CONFIG_1, (_LoRaReadRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void  LoRaSetPrembleLength(long lLength){
    _LoRaWriteRegister(REG_PREAMBLE_MSB, (uint8_t)(lLength >> 8));
    _LoRaWriteRegister(REG_PREAMBLE_LSB, (uint8_t)(lLength >> 0));
}

void  LoRaSetSyncWord(int iSw){
    _LoRaWriteRegister(REG_SYNC_WORD, iSw);
}

void  LoRaEnableCrc(void){
    _LoRaWriteRegister(REG_MODEM_CONFIG_2, _LoRaReadRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void  LoRaDisableCrc(void){
    _LoRaWriteRegister(REG_MODEM_CONFIG_2, _LoRaReadRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void  LoRaEnableInvertIQ(void){
  _LoRaWriteRegister(REG_INVERTIQ,  0x66);
  _LoRaWriteRegister(REG_INVERTIQ2, 0x19);    
}

void  LoRaDisableInvertIQ(void){
  _LoRaWriteRegister(REG_INVERTIQ,  0x27);
  _LoRaWriteRegister(REG_INVERTIQ2, 0x1d);        
}

void  LoRaSetOCP(uint8_t u8mA){
    uint8_t ocpTrim = 27;

    if (u8mA <= 120) {
        ocpTrim = (u8mA - 45) / 5;
    } else if (u8mA <=240) {
        ocpTrim = (u8mA + 30) / 10;
    }

    _LoRaWriteRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));    
}

void  LoRaSetGain(uint8_t u8Gain){
    // check allowed range
    if (u8Gain > 6) {
        u8Gain = 6;
    }

    // set to standby
    LoRaIdle();

    // set u8Gain
    if (u8Gain == 0) {
        // if u8Gain = 0, enable AGC
        _LoRaWriteRegister(REG_MODEM_CONFIG_3, 0x04);
    } else {
        // disable AGC
        _LoRaWriteRegister(REG_MODEM_CONFIG_3, 0x00);

        // clear Gain and set LNA boost
        _LoRaWriteRegister(REG_LNA, 0x03);

        // set Gain
        _LoRaWriteRegister(REG_LNA, _LoRaReadRegister(REG_LNA) | (u8Gain << 5));
    }    
}

uint8_t LoRaRandom(){
    return _LoRaReadRegister(REG_RSSI_WIDEBAND);
}

void  LoRaSetPins(int iSs, int iReset, int iDio0){
    m_iSs = iSs;
    m_iReset = iReset;
    m_iDio0 = iDio0;
}

void  LoRaSetSpiFrequency(uint32_t u32Frequency){
    m_spiSettings.u32Clock = u32Frequency;
    m_spiSettings.u8BitOrder = LORADEFAULT_SPI_MSBFIRST;
    m_spiSettings.u8DataMode = LORADEFAULT_SPI_MODE;
}

void  LoRaDumpRegisters(void){
    uint8_t rxBuffer[128];

    rxBuffer[0] = _LoRaReadRegister(0);

    spi_transaction_t t = {
        .addr = 1,
        .length = 8 * 127,
        .rx_buffer = rxBuffer + 1,
    };
    esp_err_t err = spi_device_polling_transmit(m_spiDeviceHandle, &t);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "LoraDumpRegisters() ERROR FAILED!");
        return;
    }
    printf("----------Dumping LoRa module registers----------\n  ");
    uint32_t k;
    for (k = 0; k < 16; ++k)
    {
        printf(" %X ", k);
    }
    for (k = 0; k < 128; ++k)
    {
        if (k % 16 == 0)
            printf("\n%X ", k / 16);
        printf("%02X ", rxBuffer[k]);
    }
    printf("\n-------------------------------------------------\n");    
}

void  LoRaDumpFifo(void){
    uint8_t rxBuffer[256];
    _LoRaWriteRegister( REG_FIFO_ADDR_PTR, 0x00);

    spi_transaction_t t = {
        .addr = 0,
        .length = 8 * 256,
        .rx_buffer = rxBuffer,
    };
    esp_err_t err = spi_device_polling_transmit(m_spiDeviceHandle, &t);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "LoraDumpFifo() ERROR FAILED!");
        return;
    }

    printf("------------Dumping LoRa fifo data---------------  ----------------\n  ");
    uint32_t k;
    for (k = 0; k < 16; ++k)
    {
        printf(" %X ", k);
    }
    printf("\tASCII");
    for (k = 0; k < 256; ++k)
    {
        if (k % 16 == 0)
            printf("\n%X ", k / 16);
        printf("%02X ", rxBuffer[k]);
        if ((k & 0xF) == 0xF)
        {
            printf(" ");
            for (int m = 16; m > 0; --m)
            {
                printf("%c", (rxBuffer[k - m + 1] >= 0x20 && rxBuffer[k - m + 1] <= 0x7F) ? rxBuffer[k - m + 1] : '.');
            }
        }
    }
    printf("\n-------------------------------------------------  ----------------\n");
}


/******************Private Function Implementation Section *******************/
void _LoRaExplicitHeaderMode(void){
    m_iImplicitHeaderMode = 0;
    _LoRaWriteRegister(REG_MODEM_CONFIG_1, _LoRaReadRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void _LoRaImplicitHeaderMode(void){
    m_iImplicitHeaderMode = 1;
    _LoRaWriteRegister(REG_MODEM_CONFIG_1, _LoRaReadRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void _LoRaHandleDio0Rise(void){
    int irqFlags = _LoRaReadRegister(REG_IRQ_FLAGS);

    // clear IRQ's
    _LoRaWriteRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
            // received a packet
            m_iPacketIndex = 0;

            // read packet length
            int iPacketLength = m_iImplicitHeaderMode ? _LoRaReadRegister(REG_PAYLOAD_LENGTH) : _LoRaReadRegister(REG_RX_NB_BYTES);

            // set FIFO address to current RX address
            _LoRaWriteRegister(REG_FIFO_ADDR_PTR, _LoRaReadRegister(REG_FIFO_RX_CURRENT_ADDR));

            if (m_onReceiveCallback) {
                m_onReceiveCallback(iPacketLength);
            }
        }
        else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
            if (m_onTxDoneCallback) {
                m_onTxDoneCallback();
            }
        }
    }
}

int _LoRaIsTransmitting(void){
    if((_LoRaReadRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) return 1;

    if(_LoRaReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK){
        /**** Clear IRQ's *********/
        _LoRaWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 0;
}

int  _LoRaGetSpreadingFactor(void){
    return _LoRaReadRegister(REG_MODEM_CONFIG_2) >> 4;
}

long _LoRaGetSignalBandwidth(void){
    uint8_t u8Bw = (_LoRaReadRegister(REG_MODEM_CONFIG_1) >> 4);

    switch (u8Bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
    }

    return -1;
}

void _LoRaSetLdoFlag(void){
    // Section 4.1.1.5
    long lSymbolDuration = 1000 / ( _LoRaGetSignalBandwidth() / (1L << _LoRaGetSpreadingFactor()) ) ;

    // Section 4.1.1.6
    uint8_t ldoOn = lSymbolDuration > 16;

    uint8_t config3 = _LoRaReadRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    _LoRaWriteRegister(REG_MODEM_CONFIG_3, config3);
}

uint8_t _LoRaReadRegister(uint8_t u8Address){
    spi_transaction_t t = {
        .addr = u8Address & 0x7F, /* For read access */
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    esp_err_t err = spi_device_polling_transmit(m_spiDeviceHandle, &t);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "_LoRaReadRegister() ERROR FAILED!");
    }
    return t.rx_data[0];
}

void    _LoRaWriteRegister(uint8_t u8Address, uint8_t u8Value){
    spi_transaction_t t = {
        .addr = u8Address | 0x80, /* For write access */
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data[0] = u8Value,
    };
    esp_err_t err = spi_device_polling_transmit(m_spiDeviceHandle, &t);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "_LoRaWriteRegister() ERROR FAILED!");
    }
}

static void IRAM_ATTR _LoRaOnDio0Rise(void*p){
    _LoRaHandleDio0Rise();
}