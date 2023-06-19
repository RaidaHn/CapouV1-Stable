/**
 * @file app.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifdef __cplusplus
extern "C" {
#endif

/*********** Application parameters *********************************************/
#define APP_FLASH_LED_PIN               ((gpio_num_t)(4))
#define APP_SENDING_INTERVAL_VALVE      (6333)
#define APP_SENDING_INTERVAL_DATA       (15000)
#define APP_SENDING_MESSAGE_STR         ("LoRa Base Station")
#define APP_TAG_STR                     ("[$BASE-APP$]:")

/*********** LoRa host station addresses definition *****************************/
#define APP_LORA_HOST_ADDRESS           (0x01)  /* address of this module       */
#define APP_LORA_REMOTE_ADDRESS         (0x00)  /* address of the remote module */
#define APP_LORA_BCAST_ADDRESS          (0xff)  /* address for broadcasting     */
#define APP_LORA_PROBE_NB               (50)    /* number of probes             */

/*********** LoRa module parameters *********************************************/
#define APP_LORA_CARRIER_FREQUENCY_HZ   (868E6)
#define APP_LORA_SPREADING_FACTOR       (8)
#define APP_LORA_BANDWIDTH_FREQUENCY    (62.5E3)
#define APP_LORA_CODING_RATE            (6)

/*********** LoRa module requests ***********************************************/
#define APP_LORA_REQUEST_ADDRESS        (15)

/*********** WiFi parameters ****************************************************/
#define APP_WIFI_SSID                       ("WF-DMZ")
#define APP_WIFI_PASSWORD                   ("WF-DMZ2023@)
#define APP_WIFI_CONNECTION_TIMEOUT         (10)    /* time before reconnection */
/********************************************************************************/

typedef enum e_bool{
    FALSE = 0,
    TRUE
}t_bool;


void AppInit(void);
void AppRun(void);


#ifdef __cplusplus
}
#endif