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

int gpio = 0;
int stateTo=0;


u_int portArray[]={16,5,4,0,2,14,12,13};//D0, D1, D2, D3, D4, D5, D6, D7 lolin esp8266

static QueueHandle_t tsqueue;



/* This task uses the high level GPIO API (esp_gpio.h) to blink an LED.
 *
 */


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
		vTaskDelay(1 / portTICK_PERIOD_MS);

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


void user_init(void)
{
     uart_set_baud(0, 115200);
	tsqueue = xQueueCreate(10, sizeof(char));

//    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    xTaskCreate(blinkenTask, "blinkenTask", 256, NULL, 2, NULL);



xTaskCreate(uartReceivData, "uartReceivData", 256, NULL, 1, NULL);





}


