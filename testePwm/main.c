#include <espressif/esp_common.h>
#include <espressif/esp_system.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <string.h>
#include <ssd1306/ssd1306.h>
#include <pwm.h>
#include <pcf8574/pcf8574.h>


/* pin config */
const int INTERRUPT_PCF8574_PIN = 14;   /* gpio 15 usually has "PROGRAM" button attached */
#define PWM1_PIN 14

uint16_t adcInput = 0;
uint16_t lastAdcImput = 0;
int32_t tempAdc = 0;


#define DELTA_MIN_ADC 1 // adc - 0 - 1024 (1,024V) - variacao minima para ajustar o pwm - util para reduzir ruido
#define OFFSET_INF  0 //% do duty cicle a ser adicionada para inicio de controle

float INCLINACAO = 1.0;

uint32_t valorPwmAtual = 0;



void taskPwm(void *pvParameters)
{
//    printf("Hello from taskPwm!\r\n");
    uint32_t const init_count = 0;
    uint32_t count = init_count;
    while(1)
    {
        vTaskDelay(10);

        tempAdc = (int32_t)lastAdcImput - (int32_t)adcInput;

        if(tempAdc < 0) tempAdc*=-1;

     /*   if(tempAdc < DELTA_MIN_ADC) 
        {
            printf("adcDelta min: %d\n",tempAdc);
            lastAdcImput = adcInput;
            continue;
        }
*/
        valorPwmAtual = (UINT16_MAX/100)*OFFSET_INF + (UINT16_MAX - INCLINACAO*(UINT16_MAX*adcInput)/1024); // soma offset
         if (valorPwmAtual < 0)
         {
            valorPwmAtual = 0;
            pwm_set_duty(valorPwmAtual);
            printf("saturou zero\n");
            lastAdcImput = adcInput;
            continue; 
         }

         if (valorPwmAtual > UINT16_MAX)
         {
            valorPwmAtual = UINT16_MAX;
            pwm_set_duty(valorPwmAtual);
            printf("saturou max\n");
            lastAdcImput = adcInput;
            continue; 
         }



         //se
         //printf("seting new Pwm Value %d\n", valorPwmAtual);
         pwm_set_duty(valorPwmAtual);

  //      printf("duty cycle set to %d/UINT16_MAX%%\r\n", count);
        
     /*   count += UINT16_MAX/17;
        if (count > UINT16_MAX)
            count = init_count;*/
         lastAdcImput = adcInput;
    }
}

void taskAdcRead()
{
 //   printf("Hello from taskAdcRead!\r\n");
    

    while(1)
    {
        vTaskDelay(10);
        adcInput = sdk_system_adc_read();


        printf("adcRead: %u\r\n", adcInput);
 
    }
}



void user_init(void)
{
    //uncomment to test with CPU overclocked
    //sdk_system_update_cpu_freq(160);
    // Setup HW
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

   
   /* font_timer_handle = xTimerCreate("font_timer", 1 * SECOND, pdTRUE, NULL, font_timer);
    xTimerStart(font_timer_handle, 0);*/

   /*pwm setings*/
    uint8_t pins[1];
    printf("pwm_init(1, [16])\n");
    pins[0] = PWM1_PIN;
    pwm_init(1, pins, false);
    printf("pwm_set_freq(1000)     # 1 kHz\n");
    pwm_set_freq(1000);
    printf("pwm_set_duty(UINT16_MAX/2)     # 50%%\n");
    pwm_set_duty(UINT16_MAX/2);
    printf("pwm_start()\n");
    pwm_start();
    xTaskCreate(taskPwm, "taskPwm", 256, NULL, 2, NULL);
    xTaskCreate(taskAdcRead, "taskAdcRead", 256, NULL, 2, NULL);
   

}

