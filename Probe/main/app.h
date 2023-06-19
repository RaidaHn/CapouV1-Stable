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
#define APP_FLASH_LED_PIN               ((gpio_num_t)(CONFIG_FLASH_LED_GPIO))
#define APP_SENDING_INTERVAL_MS         (5000)
#define APP_SENDING_MESSAGE_LINK        ("250") /*link protocol = n°250         */
#define APP_TAG_STR                     ("[$PROBE-APP$]:")

/*********** LoRa host station addresses definition *****************************/
#define APP_LORA_HOST_ADDRESS           (0x00)  /* address of this module       */
#define APP_LORA_REMOTE_ADDRESS         (0x01)  /* address of the remote module */
#define APP_LORA_BCAST_ADDRESS          (0xFF)  /* address for broadcasting     */

/*********** LoRa module parameters *********************************************/
#define APP_LORA_CARRIER_FREQUENCY_HZ   (868E6)
#define APP_LORA_SPREADING_FACTOR       (8)
#define APP_LORA_BANDWIDTH_FREQUENCY    (62.5E3)
#define APP_LORA_CODING_RATE            (6)
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