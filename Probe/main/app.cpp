/**
 * @file app.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/soc.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "HardwareSerial.h"

#include <../libs/TinyGPS++/src/TinyGPS++.h>
#include <..\libs\SHT2x\SHT2x.h>
#include <..\libs\OneWire\OneWire.h>
#include <..\libs\DS18B20\src\DS18B20.h>
#include <..\libs\Adafruit_BusIO\Adafruit_I2CDevice.h>
#include <..\libs\Adafruit_BusIO\Adafruit_I2CRegister.h>
#include <..\libs\AXP202X\src\axp20x.h>
#include <..\libs\BMP180/src/BMP180.h>


#include "lora.h"
#include "app.h"

#define mBitsSet(f,m)       ((f)|=(m))
#define mBitsClr(f,m)       ((f)&=(~(m)))
#define mBitsTgl(f,m)       ((f)^=(m))
#define mBitsMsk(f,m)       ((f)& (m))
#define mIsBitsSet(f,m)     (((f)&(m))==(m))
#define mIsBitsClr(f,m)     (((~(f))&(m))==(m))

#define VALVE_9V_PIN        ((gpio_num_t)32)
#define VALVE_BOOST_PIN     ((gpio_num_t)33)
#define VALVE_INVERT_PIN    ((gpio_num_t)25)
#define LED_VIE             ((gpio_num_t)2)

static const char *TAG = APP_TAG_STR;

enum e_statusMask{
    ST_ALL_CLEARED                      = 0x00000000,
    ST_ALL_SET                          = 0xffffffff,
    ST_LORA_MODULE_IS_IN_TX_MODE        = 0x00000001,
    ST_LORA_MODULE_TX_DONE_TRIGGERED    = 0x00000002, 
    ST_LORA_MODULE_RX_DONE_TRIGGERED    = 0x00000004,

    ST_LORA_FUNCTION_LINK_ADDRESS       = 0x00000010,   /* Protocole d'apparaillage */
    ST_LORA_MODULE_RX                   = 0x00000020,   /* Activation/désactivation du mode de réception*/
    ST_LORA_MODULE_TX                   = 0x00000040,   /* Activation/désactivation du mode de transmition*/
    ST_LORA_FUNCTION_SENSORS_DATA       = 0x00000080,   /* Protocole collecte données capteurs*/
    ST_VALVE_STATE                      = 0x00000100,
    ST_VALVE_IS_OPEN                    = 0x00000200,
    ST_VALVE_IS_CLOSED                  = 0x00000400,

};

struct s_app{
    uint32_t    m_uStatus;
    uint8_t     m_u8HostAddress;
    float       m_Temperature;
    float       m_HumidityAir;
    float       m_HumidityGnd;
    float       m_longitude;
    float       m_latitude;
}app={
    .m_uStatus          = ST_ALL_CLEARED,
    .m_u8HostAddress    = APP_LORA_HOST_ADDRESS,
    /********************GPS*****************************/
    .m_longitude =   1.309588,
    .m_latitude  =  44.033261
};



SHT2x sht;
TinyGPSPlus gps;
HardwareSerial  mySerial(1);

BMP180 myBMP(BMP180_ULTRAHIGHRES);

//const char *gpsStream 
/***** Private Function Prototype Declaration/Implementation Section ***************************/

/************************************************************************************************
 * @brief Callback function called on LoRa Rx done event. 
 * 
 * @param iPa Not used.
 */
void _AppLoRaRxCallback(int iPa){
    (void) iPa;
    mBitsSet(app.m_uStatus, ST_LORA_MODULE_RX_DONE_TRIGGERED);
}

/************************************************************************************************
 * @brief Callback function called on LoRa Tx done event.
 */
void _AppLoRaTxCallback(void){
    mBitsSet(app.m_uStatus, ST_LORA_MODULE_TX_DONE_TRIGGERED);
}

/************************************************************************************************
 * @brief Signaling the App state for Tx mode status. Set the Idle state
 *        of the LoRa module.  
 */
void _AppLoRaSetTxMode(void){
    if(mIsBitsSet(app.m_uStatus, ST_LORA_MODULE_IS_IN_TX_MODE)) return;
    ESP_LOGI(TAG, "Setting Tx Mode");
    LoRaIdle();
    mBitsSet(app.m_uStatus, ST_LORA_MODULE_IS_IN_TX_MODE);
}

/************************************************************************************************
 * @brief Signaling the App state for Rx mode status. Set the Rx state
 *        of the LoRa module for receiving incomming data on LoRa radio.
 */
void _AppLoRaSetRxMode(void){
    if(mIsBitsClr(app.m_uStatus, ST_LORA_MODULE_IS_IN_TX_MODE)) return;
    ESP_LOGI(TAG, "Setting Rx Mode");
    LoRaReceive(0);
    mBitsClr(app.m_uStatus, ST_LORA_MODULE_IS_IN_TX_MODE);
}

/************************************************************************************************
 * @brief 
 *     
 */
void _AppLoraSendData(uint8_t destAddr, const char * strMessage){
    uint8_t msgLen;
    _AppLoRaSetTxMode();                            /* signaling the App Tx status and setting LoRa module for Tx action    */

    LoRaWriteByte(destAddr);     /* write the module destination address to LoRa Tx FIFO             */
    LoRaWriteByte(app.m_u8HostAddress);                    /* write the module source address to LoRa Tx FIFO                  */
    LoRaWriteByte(msgLen=(uint8_t)strnlen(strMessage, 250));   /* write the data message string length to the Tx FIFO      */
    for(int k=0; k<msgLen; ++k){                        /* loop for...                                              */
        LoRaWriteByte(strMessage[k]);                          /* ...writing the data message string bytes to the Tx FIFO  */
    }                                                   /*                                                          */
    LoRaWriteByte('\0');        /* write the null string terminator to the LoRa Tx FIFO                             */
    LoRaEndPacket(TRUE);        /* ending the Tx data packet session, triggering LoRa Tx data packet on radio       */
    ESP_LOGI(TAG, "Sent data to 0x%02x [%s]", APP_LORA_REMOTE_ADDRESS, strMessage);
    _AppLoRaSetRxMode();
}

/************************************************************************************************
 * @brief 
 *     
 */
void _AppOpenValve(){
    ESP_LOGI(TAG, "Starting Valve...");
    gpio_set_level(LED_VIE,1);
	//Boost Converter
    gpio_set_level(VALVE_BOOST_PIN, 1);
    gpio_set_level(VALVE_INVERT_PIN, 0);
    gpio_set_level(VALVE_9V_PIN, 0);
    delay(100);

	gpio_set_level(VALVE_9V_PIN, 1);
	delay(200);
    gpio_set_level(VALVE_9V_PIN, 0);

	delay(10);
    gpio_set_level(VALVE_BOOST_PIN, 0);

    gpio_set_level(LED_VIE,0);
}


/************************************************************************************************
 * @brief 
 *     
 */
void _AppCloseValve(){
    ESP_LOGI(TAG, "Stopping Valve...");
    gpio_set_level(LED_VIE,1);


    //Boost Converter
	gpio_set_level(VALVE_BOOST_PIN, 1);
	gpio_set_level(VALVE_INVERT_PIN, 1);		// pour avoir du -9V
	delay(100);

	gpio_set_level(VALVE_9V_PIN, 1);
    delay(200);
	gpio_set_level(VALVE_9V_PIN, 0);
    delay(10);
    gpio_set_level(VALVE_INVERT_PIN, 0);
    gpio_set_level(VALVE_BOOST_PIN, 0);

    gpio_set_level(LED_VIE,0);
}

/************************************************************************************************
 * @brief The freeRTOS task function for LoRa processing Tx and Rx mechanisms.
 * 
 * @param pV Not used.
 */
extern "C" void _AppLoRaTask(void*pV){
    (void)pV;
    const char*msg;
    static char buf[128];
    static char dataJson[256];

    unsigned long lBaseTime = 0;
    unsigned long lElapsedTime;
    unsigned long lCurrentTime;


    mBitsSet(app.m_uStatus, ST_LORA_FUNCTION_LINK_ADDRESS);
    mBitsSet(app.m_uStatus, ST_LORA_MODULE_TX);
    mBitsClr(app.m_uStatus, ST_VALVE_STATE);


    /****************************************************/

    ESP_LOGI(TAG, "----------- ENTERING _AppLoRaTask() ------------");

    for(;;){  /******** freeRTOS task perpetual loop **************************************************************/
   
        vTaskDelay(100 / portTICK_PERIOD_MS);    /* the task takes place every 100 ms i.e. task sleeps most of the time         */
        lCurrentTime = (unsigned long)(esp_timer_get_time() / 1000ULL); /* get the current kernel time in milliseconds          */
        lElapsedTime =  lCurrentTime - lBaseTime;                  /* processing the elapsed time since the last task execution */
        
        /******** Tx Task Processing Section ************************************************************************************/
        // if(lElapsedTime>=5000)                           /* if it's time to process sending task...                              */

        /******** Tx Task Processing Section ************************************************************************************/
        if(mIsBitsSet(app.m_uStatus, ST_LORA_MODULE_TX)) {
            if(LoRaBeginPacket(FALSE)==0){              /* if LoRa module is enabled to process a new Tx packect...             */

                if(mIsBitsSet(app.m_uStatus ,ST_LORA_FUNCTION_LINK_ADDRESS)) {  /*if lora module is enable to config its address*/
                    msg = APP_SENDING_MESSAGE_LINK;
                    sprintf(buf, "%s", msg);             /* building the message string to send over LoRa radio              */
                    _AppLoraSendData(APP_LORA_REMOTE_ADDRESS, buf);
                    mBitsClr(app.m_uStatus, ST_LORA_MODULE_TX);
                }
                else if (mIsBitsSet(app.m_uStatus ,ST_LORA_FUNCTION_SENSORS_DATA)){
                    sht.read();
                    app.m_HumidityGnd = 4096.0f-analogRead(36);                        /*get humidity*/ 
                    app.m_HumidityGnd = map(app.m_HumidityGnd, 430, 3800, 0, 100);        /*convert humidity in %*/

                    // if(gps.location.lng()!=0) { app.m_longitude = gps.location.lng(); }
                    // if(gps.location.lat()!=0) { app.m_latitude = gps.location.lat(); }

                    sprintf(dataJson, "{\"node_id\": \"0x%02X\", \"longitude\": \"%f\", \"latitude\": \"%f\", \"internal_humidity\": \"%.0f\", \"external_humidity\": \"%.0f\", \"internal_temperature\": \"0\", \"external_temperature\": \"%.0f\"}",
                        app.m_u8HostAddress,
                        app.m_longitude,
                        app.m_latitude,
                        app.m_HumidityGnd,
                        app.m_HumidityAir = sht.getHumidity(),
                        app.m_Temperature = sht.getTemperature()  
                    );
                    printf("%s\n", dataJson);

                    _AppLoraSendData(APP_LORA_REMOTE_ADDRESS, dataJson);
                    mBitsClr(app.m_uStatus, ST_LORA_FUNCTION_SENSORS_DATA);
                }
                else if (mIsBitsSet(app.m_uStatus, ST_VALVE_STATE)) {
                    if(mIsBitsSet(app.m_uStatus, ST_VALVE_IS_OPEN)) {
                        msg = "2";                            /*************Send New state Valve********************/
                        sprintf(buf, "%s", msg);               /**/
                        _AppLoraSendData(APP_LORA_REMOTE_ADDRESS, buf);
                    }
                    else if (mIsBitsSet(app.m_uStatus, ST_VALVE_IS_CLOSED)) {
                        msg = "3";                            /*************Send New state Valve********************/
                        sprintf(buf, "%s", msg);               /**/
                        _AppLoraSendData(APP_LORA_REMOTE_ADDRESS, buf);
                    }
                }
            }   /*                                                                                                           */
            mBitsClr(app.m_uStatus, ST_LORA_MODULE_TX);
            mBitsSet(app.m_uStatus, ST_LORA_MODULE_TX_DONE_TRIGGERED);
        }
        /************************************************************************************************************************/

        /******* LoRa Tx done event processing **********************************************************************************/
        if(mIsBitsSet(app.m_uStatus, ST_LORA_MODULE_TX_DONE_TRIGGERED)) {   /* if LoRa Tx done event has occurred...            */
            mBitsClr(app.m_uStatus, ST_LORA_MODULE_TX_DONE_TRIGGERED);      /* acknowledging this event...                      */
            _AppLoRaSetRxMode();                                            /* and set the App state to Rx state and set the    */
            mBitsSet(app.m_uStatus, ST_LORA_MODULE_RX);
        }                                                                   /* the LoRa module in Rx state too.                 */
        /************************************************************************************************************************/

        /******* LoRa Rx done event processing **********************************************************************************/
        if(mIsBitsSet(app.m_uStatus, ST_LORA_MODULE_RX)){                   /* if LoRa Rx done event has occurred...            */
            uint8_t u8DstAddr = LoRaRead();                                          /* get the module destination address   */
            //delay(5000);


            if(u8DstAddr==app.m_u8HostAddress) {
                // if(u8DstAddr!=app.m_u8HostAddress && u8DstAddr!=APP_LORA_BCAST_ADDRESS){    /* checking destination address...      */
                // //ESP_LOGI(TAG, "TTGO has received a data frame not for this station!");  /* address not matching host one        */
                // }

                ESP_LOGI(TAG, "Received LoRa data: RSSI:%d\tSNR:%f", LoRaPacketRssi(), LoRaPacketSnr()); /* displaying some Rx stats*/
                uint8_t u8SrcAddr = LoRaRead();             /* retrieves the source host station address                        */
                uint8_t u8SzData  = LoRaRead();             /* retrieves the data length                                        */
                char data[u8SzData+1];                      /* allocates a character array for data storage                     */
                for(int k=0; k<u8SzData; ++k){              /* loop for data retrieving...                                      */
                    data[k] = LoRaRead();                   /* peeks byte of data from LoRa Rx FIFO                             */
                }   /*                                                                                                          */
                data[u8SzData]='\0';                        /* placing the null character string terminator                     */

                    /************************************************ Link **************************************************************/
                    if((mIsBitsSet(app.m_uStatus ,ST_LORA_FUNCTION_LINK_ADDRESS)) && u8SrcAddr== APP_LORA_REMOTE_ADDRESS) {
                        ESP_LOGI(TAG, "dstAddr: 0x%02X srcAddr: 0x%02X Raw message content: \"%s\"", u8DstAddr, u8SrcAddr, data);
                        ESP_LOGI(TAG,"Configuration new address");
                        app.m_u8HostAddress = strtol(data, NULL, 10 );
                        ESP_LOGI(TAG,"New address :0x%02x", app.m_u8HostAddress);

                        mBitsClr(app.m_uStatus ,ST_LORA_FUNCTION_LINK_ADDRESS);
                    }/********************************************END Link **************************************************************/
                    else if (mIsBitsClr(app.m_uStatus, ST_LORA_FUNCTION_LINK_ADDRESS)) {
                        ESP_LOGI(TAG, "dstAddr: 0x%02X srcAddr: 0x%02X Raw message content: \"%s\"", u8DstAddr, u8SrcAddr, data);

                        switch (atoi(data)) {
                        case 10:            /* Code 10 = Envoie des données temp / hum / gps...*/
                            mBitsSet(app.m_uStatus, ST_LORA_FUNCTION_SENSORS_DATA);
                            mBitsSet(app.m_uStatus, ST_LORA_MODULE_TX);

                            while(mySerial.available()>0){
                                gps.encode(mySerial.read());
                            }
                            break;
                        case 15:            /* Code 15 = Reception ordre fermer la vanne*/
                            mBitsClr(app.m_uStatus, ST_VALVE_IS_OPEN);
                            _AppCloseValve();
                            mBitsSet(app.m_uStatus, ST_VALVE_IS_CLOSED);
                            mBitsSet(app.m_uStatus, ST_VALVE_STATE);
                            mBitsSet(app.m_uStatus, ST_LORA_MODULE_TX);
                            break;
                        case 20:            /* Code 20 = Reception ordre ouvrir la vanne*/
                            mBitsClr(app.m_uStatus, ST_VALVE_IS_CLOSED);
                            _AppOpenValve();
                            mBitsSet(app.m_uStatus, ST_VALVE_IS_OPEN);
                            mBitsSet(app.m_uStatus, ST_VALVE_STATE);
                            mBitsSet(app.m_uStatus, ST_LORA_MODULE_TX);
                            break;
                        default:
                            break;
                        }
                    }
                    
            }
        }   /*                                                                                                              */
        mBitsClr(app.m_uStatus, ST_LORA_MODULE_RX_DONE_TRIGGERED);  /* acknowledging the Rx done event                      */            
    }/*                                                                                                                     */
    /************************************************************************************************************************/

} /* []end of the perpetual loop    */



/************************************************************************************************
 * @brief Initializes the application App entity
 */
void AppInit(void){
    ESP_LOGI(TAG, "AppInit()");
    esp_err_t err;
    err = gpio_install_isr_service(0);
    ESP_ERROR_CHECK(err);

    /******* Flash led initializing ***************************/
    gpio_reset_pin(APP_FLASH_LED_PIN);
    gpio_set_direction(APP_FLASH_LED_PIN, GPIO_MODE_OUTPUT);


    /******* LoRa module parametrization ************************/
    LoRaBegin(APP_LORA_CARRIER_FREQUENCY_HZ);           /*      */
    LoRaSetSpreadingFactor(APP_LORA_SPREADING_FACTOR);  /*      */
    LoRaSetSignalBandwidth(APP_LORA_BANDWIDTH_FREQUENCY);   /*  */
    LoRaSetCodingRate4(APP_LORA_CODING_RATE);           /*      */
    LoRaOnReceive(_AppLoRaRxCallback);                  /*      */
    LoRaOnTxDone(_AppLoRaTxCallback);                   /*      */
    LoRaEnableCrc();                                    /*      */
    /************************************************************/
    Serial.begin(115200);

    /***** GPS LAT/LNG ***********/
    delay(200);
    mySerial.begin(4800, SERIAL_8N1, 34, 12);
    while(mySerial.available()>0){
        gps.encode(mySerial.read());
    }

    ESP_LOGI(TAG, "long, lat : %.6f,%.6f",gps.location.lng(), gps.location.lat());

    /*********SENSOR BMP**********/
    // myBMP.begin();
    // Serial.println(F("Bosch BMP180 sensor is OK"));

    /*********SENSOR SHT**********/
    sht.begin(21, 22);
    uint8_t stat = sht.getStatus();
    Serial.print(stat, HEX);
    Serial.println();

    /*******valve pin reset + mode***********/
    gpio_reset_pin(VALVE_9V_PIN);
    gpio_set_direction(VALVE_9V_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(VALVE_BOOST_PIN);
    gpio_set_direction(VALVE_BOOST_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(VALVE_INVERT_PIN);
    gpio_set_direction(VALVE_INVERT_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_VIE);
    gpio_set_direction(LED_VIE, GPIO_MODE_OUTPUT);
}


/************************************************************************************************************
 * @brief The main application task, contains the main perpetual task loop
 */
void AppRun(void){
    ESP_LOGI(TAG, "AppRun()");

    LoRaDumpRegisters();    /* displays the content of the entire LoRa Semtech SX127x register bytes array  */
    LoRaDumpFifo();         /* displays the content of the entire LoRa Semtech SX127x fifo data bytes array */

    xTaskCreate(_AppLoRaTask, NULL, 4096, NULL, 5, NULL);   /* creates the freeRTOS LoRa processing task    */

    for(uint32_t k=-1;;){   /* the main perpetual task loop */
        /***** Doing the flashing led processor activity *****************************/
        static const uint32_t ledSeq[]={40,90,40,3500,0};
        if(ledSeq[++k]==0) k=0;
        gpio_set_level(APP_FLASH_LED_PIN, k&0x01);
        vTaskDelay(ledSeq[k] / portTICK_PERIOD_MS);
    }   /* []end of the perpetual main task loop    */
}
/************************************************************************************************************/