#include "esp8266_at.h"

uint8_t uart3_txbuf[256];
uint8_t uart3_rxbuf[512];
uint8_t uart3_rxone[1];
uint8_t uart3_rxcounter;

lpuart_rtos_handle_t uart3;


void ESP8266_ATSendString(char* str)
{
        uart3.base = LPUART3;
        HAL_AT_Uart_Send(&uart3,(uint8_t *)str,strlen(str),0xFFFF);

}

//退出透传
void ESP8266_ExitUnvarnishedTrans(void)
{
        ESP8266_ATSendString("+++");
        vTaskDelay(500);
        ESP8266_ATSendString("+++");
        vTaskDelay(500);	
}

//查找字符串中是否包含另一个字符串
uint8_t FindStr(char* dest,char* src,uint16_t retry_nms)
{
	retry_nms/=10;                   //超时时间

	while(strstr(dest,src)==0 && retry_nms--)//等待串口接收完毕或超时退出
	{		
		vTaskDelay(500);
	}

	if(retry_nms) return 1;                       

	return 0; 
}

/**
 * 功能：检查ESP8266是否正常
 * 参数：None
 * 返回值：ESP8266返回状态
 *        非0 ESP8266正常
 *        0 ESP8266有问题  
 */

uint8_t ESP8266_Check(void)
{
	uint8_t check_cnt=5;
	while(check_cnt--)
	{
		ESP8266_ATSendString("AT\r\n");               //发送AT握手指令	
		if(FindStr((char*)uart3_rxbuf,"OK",200) != 0)
		{
			return 1;
		}
	}
	return 0;
}

/**
 * 功能：初始化ESP8266
 * 参数：None
 * 返回值：初始化结果，非0为初始化成功,0为失败
 */
uint8_t ESP8266_Init(void)
{
	
	ESP8266_ExitUnvarnishedTrans();		//退出透传
	vTaskDelay(500);
	ESP8266_ATSendString("AT+RST\r\n");
	vTaskDelay(500);

     	if(ESP8266_Check()==0)              
	{
	   return 0;
	}
        vTaskDelay(500);
	memset(uart3_rxbuf,0,sizeof(uart3_rxbuf));    //清空接受缓冲
	//ESP8266_ATSendString("ATE0\r\n");     	//关闭回显 
	if(FindStr((char*)uart3_rxbuf,"OK",200)==0)    //设置不成功
	{
		return 0;      
	}
	return 1;                         //设置成功
}

/**
 * 功能：连接热点
 * 参数：
 *         ssid:热点名
 *         pwd:热点密码
 * 返回值：
 *         连接结果,非0连接成功,0连接失败
 * 说明： 
 *         失败的原因有以下几种(UART通信和ESP8266正常情况下)
 *         1. WIFI名和密码不正确
 *         2. 路由器连接设备太多,未能给ESP8266分配IP
 */
uint8_t ESP8266_ConnectAP(char* ssid,char* pass)
{
	uint8_t cnt=5;
	while(cnt--)
	{
		memset(uart3_rxbuf,0,sizeof(uart3_rxbuf));     
		ESP8266_ATSendString("AT+CWMODE=1\r\n");              //设置为STA模式
                vTaskDelay(500);
	        ESP8266_ATSendString("AT+RST\r\n");                
		if(FindStr((char*)uart3_rxbuf,"OK",200) != 0)
		{
			break;
		}             		
	}
	if(cnt == 0)
		return 0;

	cnt=2;
	while(cnt--)
	{                    
		memset(uart3_txbuf,0,sizeof(uart3_txbuf));//清空发送缓冲
		memset(uart3_rxbuf,0,sizeof(uart3_rxbuf));//清空接收缓冲
		sprintf((char*)uart3_txbuf,"AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,pass);//连接目标AP AT+CWJAP+_CUR
		ESP8266_ATSendString((char*)uart3_txbuf);	
		if(FindStr((char*)uart3_rxbuf,"OK",200)!=0)                      //连接成功且分配到IP
		{
			return 1;
		}
	}
	return 0;
}
