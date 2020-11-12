/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* Copyright 2016-2019 NXP
* All rights reserved.
*
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*******************************************************************************
 * Includes
 ******************************************************************************/
//#include "lwip/tcpip.h"
//#include "lwip/apps/lwiperf.h"
#include "board.h"
//#include "wwd.h"
//#include "wwd_wiced.h"

#include "fsl_debug_console.h"


#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_common.h"

#include "kv_api.h"
#include "flexspi_hyper_flash_ops.h"

#include "ewmain.h"

#include "esp8266_at.h"
#include "at_wrapper.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define WIFI_SSID "7K"
#define WIFI_PASS "11111111"
//#define AP_SEC WICED_SECURITY_WPA2_AES_PSK

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

//extern void test_join(void);
//extern wwd_result_t test_scan();
//extern wiced_result_t wiced_wlan_connectivity_init(void);
//extern void add_wlan_interface(void);

//extern int linkkit_example_solo(int argc, char **argv);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

//static void BOARD_USDHCClockConfiguration(void)
//{
//    /*configure system pll PFD2 fractional divider to 24*/
//    CLOCK_InitSysPfd(kCLOCK_Pfd2, 24U);
//    /* Configure USDHC clock source and divider */
//    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
//    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 0U);
//}


void Enter_ErrorMode(uint8_t mode)
{
	while(1)
	{
		switch(mode){
			case 0:HAL_Printf("ESP8266 Initialized Failed.\r\n");
                        break;
			case 1:HAL_Printf("Wifi Connecting Failed.\r\n");
                        break;
			default:HAL_Printf("Nothing\r\n");
                        break;
		}
		vTaskDelay(500);
	}
}


/*!
 * @brief The main function containing client thread.
 */
static void demo_task(void *arg)
{
    PRINTF("\r\n************************************************\r\n");
    PRINTF(" CSDK Demo task example\r\n");
    PRINTF("************************************************\r\n");
    
//    BOARD_InitNetwork();
    vTaskDelay(400);
    
    
#if (DEMO_OPTION == DEMO_DIM_LIGHT)
	PRINTF("Run Dimmable Light Demo...\r\n");
        lighting_run(NULL,NULL);
#elif (DEMO_OPTION == DEMO_RGB_LIGHT)
	PRINTF("Run RGB Lighting Demo...\r\n");
	rgb_light_run(NULL,NULL);
#elif (DEMO_OPTION == DEMO_WASHING_MACHINE)
	PRINTF("Run Washing Machine Demo...\r\n");
        wm_run(NULL, NULL);
#endif
}

/*******************************************************************************
* FUNCTION:
*   GuiThread
*
* DESCRIPTION:
*   The EwThread processes the Embeded Wizard application.
*
* ARGUMENTS:
*   arg - not used.
*
* RETURN VALUE:
*   None.
*
*******************************************************************************/
static void GuiThread( void* arg )
{
  /* initialize Embedded Wizard application */
  if ( EwInit() == 0 )
    return;

  EwPrintSystemInfo();

  /* process the Embedded Wizard main loop */
  while( EwProcess())
    ;

  /* de-initialize Embedded Wizard application */
  EwDone();
}

int wifi_init(void)
{
    uart_dev_t AT_Uart = {
     .port = 2,     
    };
    HAL_AT_Uart_Init(&AT_Uart);
    uint8_t status = 0;
    
    if(ESP8266_Init()){
       HAL_Printf("ESP8266 Initialized Successfully.\r\n");
       status++;
    }
    else Enter_ErrorMode(0);
    
    if(status == 1){
      if(ESP8266_ConnectAP(WIFI_SSID,WIFI_PASS)){
         HAL_Printf("Wifi connecting Successfully.\r\n");
         status++;
      }
    else Enter_ErrorMode(1);
    }
}

#if 0
static uint8_t inited = 0;
static uint8_t gotip = 0;
int at_wifi_join(char *ssid, char *pwd)
{
	if(!inited){

		return -1;
	}

	char conn_str[100]= {0};
//    char out[20] = {0};
    HAL_Snprintf(conn_str, 100, "AT+CWJAP=%s,%s", ssid, pwd);

    if (at_send_no_reply(conn_str, strlen(conn_str), true) < 0){
        return -1;
    }

    return 0;
}


int HAL_Wifi_Connected(void ){

	return gotip;
}

int at_wifi_factory_new(void ){
	if(!inited){

		return -1;
	}
	char cmd[32]={0};
        char out[32] = {0};
	HAL_Snprintf(cmd, 32, "AT+FACTORY");
	if (at_send_no_reply(cmd, strlen(cmd), true) < 0){
		return -1;
	}

	return 0;
}

void app_wait_wifi_connect(void ){

	char wifi_ssid[40]={0};
	char wifi_key[40] = {0};
	int ssid_len = 40;
	int key_len = 40;
//	wwd_result_t err = WWD_SUCCESS;
//	int cnt = 0x0e;
        if(HAL_Kv_Get("wifi_ssid", wifi_ssid, &ssid_len) == 0){
          if(ssid_len != 1 || wifi_ssid[0] != 0xff){


            if(HAL_Kv_Get("wifi_key", wifi_key, &key_len) == 0){
               if(key_len != 1){
                  at_wifi_join(wifi_ssid,wifi_key);
                  HAL_Printf("join wifi:%s....\r\n",wifi_ssid);
                  HAL_SleepMs(2000);
               }
            }
          }
	}
	if(!HAL_Wifi_Connected()){
	    HAL_Printf("Wifi not connected, join the AP first\r\n");
	    HAL_SleepMs(1000);
	    while(!HAL_Wifi_Connected()){
	      HAL_SleepMs(500);
	    }
	}
}

static uint8_t app_wifi_ib_same(char *ssid, char *key){
	char wifi_ssid[40]={0};
	char wifi_key[40] = {0};
	int ssid_len = 40;
	int key_len = 40;
	if((HAL_Kv_Get("wifi_ssid", wifi_ssid, &ssid_len) == 0) && (strncmp(ssid,wifi_ssid,strlen(wifi_ssid)) == 0)){
	    if((HAL_Kv_Get("wifi_key", wifi_key, &key_len) == 0) &&(strncmp(key,wifi_key,strlen(wifi_key)) == 0)){
	 
			HAL_Printf("Same WiFi IB inputed\r\n");
			return 1;

	    }
	}
	return 0;


}

void app_process_wifi_config(char *ssid, char *key){
	if((ssid == NULL) && (key == NULL)){
		uint8_t value_invalid = 0xff;
		HAL_Kv_Set("wifi_ssid", &value_invalid, 1, 0);
		HAL_Kv_Set("wifi_key", &value_invalid, 1, 0);
//		  wwd_wifi_leave(WWD_STA_INTERFACE);
                at_wifi_factory_new();  //mk3060.c 

	}else if(app_wifi_ib_same(ssid,key) == 0){
		HAL_Kv_Set("wifi_ssid", ssid, strlen(ssid), 0);
		HAL_Kv_Set("wifi_key", key, strlen(key), 0);
	}

}

/*
void app_process_recive_cmd(char *buff, uint8_t len){
  uint8_t ptr = 2;
  uint8_t i = 0;
  if(buff[0] == 'c'){//connect wifi
		char wifi_ssid[40]={0};
		char wifi_key[40] = {0};
		if(buff[1] == ' '){
			while(buff[ptr] != ' '){
				wifi_ssid[i++] = buff[ptr++];
			}
			ptr++;
			i=0;
			while(buff[ptr] != '\r' && (ptr<len)){
				wifi_key[i++] = buff[ptr++];
			}
			if(app_wifi_ib_same(wifi_ssid,wifi_key) == 0){
				HAL_Kv_Set("wifi_ssid", wifi_ssid, strlen(wifi_ssid), 0);
				HAL_Kv_Set("wifi_key", wifi_key, strlen(wifi_key), 0);
			}
                        wiced_ssid_t ap_ssid = {0};

                        ap_ssid.length = strlen(wifi_ssid);
                        memcpy(ap_ssid.value,wifi_ssid,ap_ssid.length);
                        wwd_wifi_join(&ap_ssid, AP_SEC, (uint8_t *)wifi_key, strlen(wifi_key), NULL, WWD_STA_INTERFACE);
                        HAL_Printf("join wifi:%s....\r\n",wifi_ssid);
		}

  }else if(buff[0] == 'f'){//factory new module
  	uint8_t value_invalid = 0xff;
	HAL_Kv_Set("wifi_ssid", &value_invalid, 1, 0);
	HAL_Kv_Set("wifi_key", &value_invalid, 1, 0);
	wwd_wifi_leave(WWD_STA_INTERFACE);
        HAL_Printf("Factory wifi module....\r\n");
  }else if(buff[0] == 'w'){
  	wm_cli_process(buff+1, len - 1);
  	

  }
  else
  {
  	HAL_Printf("Unknown command\r\n");
  }
  
}
*/
#endif

/*!
 * @brief Main function.
 */
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
//    BOARD_USDHCClockConfiguration();
    BOARD_InitDebugConsole();
    PRINTF("1.\r\n");
//    ShellInit();

    flexspi_hyper_flash_init();
    kv_init();
    
   
//    tcpip_init(NULL, NULL);
	
#if (DEMO_OPTION == DEMO_WASHING_MACHINE)
	BOARD_InitI2C1Pins();
	BOARD_InitSemcPins();
    /* create thread that drives the Embedded Wizard GUI application... */
    EwPrint( "Create UI thread...                          " );
    xTaskCreate( GuiThread, "EmWi_Task", 2048, NULL, 2, NULL );
    EwPrint( "[OK]\n" );
#endif

    if (xTaskCreate(demo_task, "demo_task", 2048, NULL, 3, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1);
    }
	
    vTaskStartScheduler();

    return 0;
}
