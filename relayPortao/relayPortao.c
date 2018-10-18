/* The classic "blink" example
 *
 * This sample code is in the public domain.
 */
#include <stdlib.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include <queue.h>

#include <string.h>
#include <dhcpserver.h>
#include <lwip/api.h>




#define AP_SSID "ITRIADIOT"
#define AP_PSK "itriadIot"
#define TELNET_PORT 23




int gpio = 0;
int stateTo=0;


u_int portArray[]={16,5,4,0,2,14,12,13};//D0, D1, D2, D3, D4, D5, D6, D7 lolin esp8266

u_int sensorInput = 15;
static QueueHandle_t tsqueue;
scanTime = 100;
bool toChangePortState = false;





void blinkenTask(void *pvParameters)
{
int i;
for(i = 0; i < 8; i ++)    gpio_enable(portArray[i], GPIO_OUTPUT);
int lastState = 0;
for(int j = 0; j < 8; j ++) gpio_write(portArray[j], 1);
i=0;



    while(1)
    {

	
	vTaskDelay(portTICK_PERIOD_MS); 

//i++;
	//gpio = portArray[i%8];


	if(toChangePortState)
	{
		toChangePortState = false;
		gpio_write(gpio, stateTo);
		lastState = lastState==0?1:0;

	}

/*
printf("seting gpio: [%d]\r\n",gpio);

        gpio_write(gpio, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_write(gpio, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
*/

/*



printf("seting gpio: [%d]\r\n",gpio);

       for(int j = 0; j < 8; j ++) gpio_write(portArray[j], 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        for(int j = 0; j < 8; j ++) gpio_write(portArray[j], 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

*/
    }
}



void uartReceivData(void *pvParameters)
{
int charNumber = 0;
uart_rxfifo_wait(0,1);

char cmdReceived, cmdBuff[10];
u_short indiceCmd = 0;
memset(cmdBuff,0,10);

	for(;;)
	{
		charNumber = uart_getc_nowait(0);
		vTaskDelay(10 / portTICK_PERIOD_MS);

		if(charNumber != -1)
		{	
	


			char received = (char) charNumber;

			

			printf("%c",received);
//toChangePortState = true;

	
			cmdBuff[indiceCmd] = received;
			indiceCmd++;
			//printf("cmdBuff:[%s]\n",cmdBuff);



			if(indiceCmd > 5)
			{
//toChangePortState = true;
		 		if(memcmp(cmdBuff,"rele",4) == 0 )
				{
					char releNumber = cmdBuff[4];
					char stateCmd = cmdBuff[5];

					if( (releNumber > '8') || (releNumber < '1') )
					{
						printf("parser error releNumber [%c]\n",releNumber);
					}

					else if( (stateCmd != '0') && (stateCmd != '1') )
					{
						printf("parser error state invalid [%c]\n",stateCmd);
					}
					else
					{
						gpio = portArray[releNumber - '0' - 1];
						stateTo=stateCmd - '0';
						if(stateTo == 0) stateTo = 1;
						else stateTo = 0;
						toChangePortState = true;
						printf("OK",gpio,stateTo);
					}
					
				}
				else
				{
					printf("parser error [%s]\n",cmdBuff);
				}

				memset(cmdBuff,0,10);
				indiceCmd = 0;
			}

			


			//if(charNumber == 102) toChangePortState = true;
		}
		
	}

}




void monitoraSensor(void *pvParameters)
{
	int32_t now = xTaskGetTickCount();
	int32_t nextEventTime = now;
	gpio_enable(sensorInput, GPIO_INPUT);
	uint8_t scan = 0, scanPrev=0;

	while(1)	
	{
		vTaskDelay(scanTime/portTICK_PERIOD_MS);
		scan = gpio_read(sensorInput);
		if(scan != scanPrev)
		{
			now = xTaskGetTickCount();
			printf("switch detectec at: [%d]ms\n",now);
		}
		scanPrev = scan;
	}



}



static void telnetTask(void *pvParameters)
{
    ip_addr_t first_client_ip;
    IP4_ADDR(&first_client_ip, 172, 16, 0, 2);
    dhcpserver_start(&first_client_ip, 4);

    struct netconn *nc = netconn_new(NETCONN_TCP);
    if (!nc)
    {
        printf("Status monitor: Failed to allocate socket.\r\n");
        return;
    }
    netconn_bind(nc, IP_ANY_TYPE, TELNET_PORT);
    netconn_listen(nc);

    while (1)
    {
        struct netconn *client = NULL;
        err_t err = netconn_accept(nc, &client);

        if (err != ERR_OK)
        {
            if (client)
                netconn_delete(client);
            continue;
        }

        ip_addr_t client_addr;
        uint16_t port_ignore;
        netconn_peer(client, &client_addr, &port_ignore);

        char buf[80];
        snprintf(buf, sizeof(buf), "Uptime %d seconds\r\n", xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
        netconn_write(client, buf, strlen(buf), NETCONN_COPY);
        snprintf(buf, sizeof(buf), "Free heap %d bytes\r\n", (int) xPortGetFreeHeapSize());
        netconn_write(client, buf, strlen(buf), NETCONN_COPY);
        char abuf[40];
        snprintf(buf, sizeof(buf), "Your address is %s\r\n\r\n", ipaddr_ntoa_r(&client_addr, abuf, sizeof(abuf)));
        netconn_write(client, buf, strlen(buf), NETCONN_COPY);
        netconn_delete(client);
    }
}




void user_init(void)
{
     uart_set_baud(0, 115200);
	tsqueue = xQueueCreate(10, sizeof(char));

//    printf("SDK version:%s\n", sdk_system_get_sdk_version());
	xTaskCreate(blinkenTask, "blinkenTask", 256, NULL, 2, NULL);
	xTaskCreate(uartReceivData, "uartReceivData", 256, NULL, 1, NULL);

	xTaskCreate(monitoraSensor, "monitoraSensor", 256, NULL, 1, NULL);



    sdk_wifi_set_opmode(SOFTAP_MODE);
    struct ip_info ap_ip;
    IP4_ADDR(&ap_ip.ip, 172, 16, 0, 1);
    IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 0, 0);
    sdk_wifi_set_ip_info(1, &ap_ip);

    struct sdk_softap_config ap_config = { .ssid = AP_SSID, .ssid_hidden = 0, .channel = 3, .ssid_len = strlen(AP_SSID), .authmode =
            AUTH_WPA_WPA2_PSK, .password = AP_PSK, .max_connection = 3, .beacon_interval = 100, };
    sdk_wifi_softap_set_config(&ap_config);

    xTaskCreate(telnetTask, "telnetTask", 512, NULL, 2, NULL);


}


