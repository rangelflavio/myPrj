#include <espressif/esp_common.h>
#include <espressif/esp_system.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <string.h>
#include <pwm.h>
#include <semphr.h>

 QueueHandle_t publish_queue;
#define PUB_MSG_LEN 64


/* Remove this line if your display connected by SPI */
#define I2C_CONNECTION
#include <i2c/i2c.h>


    #define I2C_BUS  0
  //  #define SCL_PIN  5
  //  #define SDA_PIN  4
    #define SCL_PIN  13   //d7
    #define SDA_PIN  12    //d6

#define PWM1_PIN 16

//param addr I2C register address (0b0100<A2><A1><A0> for PCF8574)
#define ADD_PCF8574_ctrl  0x20  //endereco correto sem o bit de leitura/escrita (a0 a1 a2 em GND) //100000
#define ADD_PCF8574_motor 0x27  //endereco correto sem o bit de leitura/escrita (a0 a1 a2 em GND) //100111

static const i2c_dev_t devPcf8574_ctr = {
    .bus      = I2C_BUS,   //mesmo barramento do display
    .addr     = ADD_PCF8574_ctrl
};

static const i2c_dev_t devPcf8574_mot = {
    .bus      = I2C_BUS,   //mesmo barramento do display
    .addr     = ADD_PCF8574_motor
};


#define SECOND (1000 / portTICK_PERIOD_MS)





bool empilharMensagem(char *msg)
{
         if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) 
        {
            printf("Publish queue overflow.\r\n");
            return false;
        }
  return true;
}


static void handleI2cDevices(void *pvParameters)
{
    printf("%s: Started user interface task\n", __FUNCTION__);
    vTaskDelay(SECOND);

    while (1)
    {

        vTaskDelay(SECOND/10);
        pcf8574_port_write(&devPcf8574_ctr, 0x00);
        pcf8574_port_write(&devPcf8574_mot, 0xff);

    }

}




void taskPwm(void *pvParameters)
{
/*
    printf("Hello from taskPwm!\r\n");
    uint32_t const init_count = 0;
    uint32_t count = init_count;
    while(1)
    {
        vTaskDelay(100);
  //      printf("duty cycle set to %d/UINT16_MAX%%\r\n", count);
        pwm_set_duty(count);
        count += UINT16_MAX/17;
        if (count > UINT16_MAX)
            count = init_count;
    }
    */
}

void taskAdcRead()
{
    /*
 //   printf("Hello from taskAdcRead!\r\n");
    uint16_t adcInput = 0;

    while(1)
    {
        vTaskDelay(100);
        adcInput = sdk_system_adc_read();
  //      printf("adcRead: %u\r\n", adcInput);
 
    }*/
}

/*
const int active = 1; // active == 0 for active low 
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;

static QueueHandle_t tsqueue;
void gpio_intr_handler(uint8_t gpio_num)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now, NULL);
}
void buttonIntTask(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", INTERRUPT_PCF8574_PIN);
    uint8_t botaoApertado;
    QueueHandle_t *tsqueue = (QueueHandle_t *)pvParameters;
    gpio_set_interrupt(INTERRUPT_PCF8574_PIN, int_type, gpio_intr_handler);
    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_PERIOD_MS;
        if(last < button_ts-200) {
        
            //botaoApertado = pcf8574_port_read(&devPcf8574);
            //printf("Button interrupt fired at %dms, value[%u]\r\n", button_ts,botaoApertado);
            keyToGet = true;
            printf("Button interrupt fired at %dms\r\n", button_ts);
            last = button_ts;

        }
    }
}*/

void user_init(void)
{
    //uncomment to test with CPU overclocked
    //sdk_system_update_cpu_freq(160);
    // Setup HW
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_400K);

 

    xTaskCreate(handleI2cDevices, "handleI2cDevices", 256, NULL, 2, NULL);


   /*pwm setings*/
   /* uint8_t pins[1];
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
*/
//    xTaskCreate(taskAdcRead, "taskAdcRead", 256, NULL, 2, NULL);


}

