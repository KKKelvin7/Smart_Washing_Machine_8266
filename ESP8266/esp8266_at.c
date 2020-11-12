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

//�˳�͸��
void ESP8266_ExitUnvarnishedTrans(void)
{
        ESP8266_ATSendString("+++");
        vTaskDelay(500);
        ESP8266_ATSendString("+++");
        vTaskDelay(500);	
}

//�����ַ������Ƿ������һ���ַ���
uint8_t FindStr(char* dest,char* src,uint16_t retry_nms)
{
	retry_nms/=10;                   //��ʱʱ��

	while(strstr(dest,src)==0 && retry_nms--)//�ȴ����ڽ�����ϻ�ʱ�˳�
	{		
		vTaskDelay(500);
	}

	if(retry_nms) return 1;                       

	return 0; 
}

/**
 * ���ܣ����ESP8266�Ƿ�����
 * ������None
 * ����ֵ��ESP8266����״̬
 *        ��0 ESP8266����
 *        0 ESP8266������  
 */

uint8_t ESP8266_Check(void)
{
	uint8_t check_cnt=5;
	while(check_cnt--)
	{
		ESP8266_ATSendString("AT\r\n");               //����AT����ָ��	
		if(FindStr((char*)uart3_rxbuf,"OK",200) != 0)
		{
			return 1;
		}
	}
	return 0;
}

/**
 * ���ܣ���ʼ��ESP8266
 * ������None
 * ����ֵ����ʼ���������0Ϊ��ʼ���ɹ�,0Ϊʧ��
 */
uint8_t ESP8266_Init(void)
{
	
	ESP8266_ExitUnvarnishedTrans();		//�˳�͸��
	vTaskDelay(500);
	ESP8266_ATSendString("AT+RST\r\n");
	vTaskDelay(500);

     	if(ESP8266_Check()==0)              
	{
	   return 0;
	}
        vTaskDelay(500);
	memset(uart3_rxbuf,0,sizeof(uart3_rxbuf));    //��ս��ܻ���
	//ESP8266_ATSendString("ATE0\r\n");     	//�رջ��� 
	if(FindStr((char*)uart3_rxbuf,"OK",200)==0)    //���ò��ɹ�
	{
		return 0;      
	}
	return 1;                         //���óɹ�
}

/**
 * ���ܣ������ȵ�
 * ������
 *         ssid:�ȵ���
 *         pwd:�ȵ�����
 * ����ֵ��
 *         ���ӽ��,��0���ӳɹ�,0����ʧ��
 * ˵���� 
 *         ʧ�ܵ�ԭ�������¼���(UARTͨ�ź�ESP8266���������)
 *         1. WIFI�������벻��ȷ
 *         2. ·���������豸̫��,δ�ܸ�ESP8266����IP
 */
uint8_t ESP8266_ConnectAP(char* ssid,char* pass)
{
	uint8_t cnt=5;
	while(cnt--)
	{
		memset(uart3_rxbuf,0,sizeof(uart3_rxbuf));     
		ESP8266_ATSendString("AT+CWMODE=1\r\n");              //����ΪSTAģʽ
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
		memset(uart3_txbuf,0,sizeof(uart3_txbuf));//��շ��ͻ���
		memset(uart3_rxbuf,0,sizeof(uart3_rxbuf));//��ս��ջ���
		sprintf((char*)uart3_txbuf,"AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,pass);//����Ŀ��AP AT+CWJAP+_CUR
		ESP8266_ATSendString((char*)uart3_txbuf);	
		if(FindStr((char*)uart3_rxbuf,"OK",200)!=0)                      //���ӳɹ��ҷ��䵽IP
		{
			return 1;
		}
	}
	return 0;
}
