#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <esp/hwrand.h>
#include <pwm.h>
//#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

/*#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <semphr.h>
*/

/* You can use http://test.mosquitto.org/ to test mqtt_client instead
 * of setting up your own MQTT server */

/*
#define WIFI_SSID "TCTSmartPhones"
#define WIFI_PASS "TCTSMARTPHONES"
#define MQTT_HOST ("192.168.23.31")
#define MQTT_PORT 1883
#define MQTT_USER NULL
#define MQTT_PASS NULL
*/

/*
SemaphoreHandle_t wifi_alive;
QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

*/

//sensor keyence para feira
//u_int sensorInput = 15;//d8
//u_int scanTime = 100;
//u_int portArray[]={16,5,4,0,2,14,12,13,15};//D0, D1, D2, D3, D4, D5, D6, D7 lolin esp8266

#define MAX_INPUT_NUMBER 5
u_int portArray[]={16,5,4,14,13}; //d0 d1 d2 d5

#define PWM1_PIN 12 //d6 pin ouput


#define linkLedPin 15


#define BOTAO_PRODUCAO 1
#define BOTAO_MANUTENCAO 2
#define BOTAO_PECAS_DEFEITO 0
#define SENSOR_PRODUCAO 3
#define ON_OFF_BT 4



//d0 d1 d2 d5  ProdutoErro, fimProducao/reset, paradaManutencao, contadorProduto



#define topic1Good "pieceProduction,%d,good\n"
#define topic1Bad "pieceProduction,%d,bad\n"


//state/timeIni,timeFin,
#define topic2PauseStart "state,%d,initMaintenance\n"
#define topic2PauseStop "state,%d,endMaintenance\n"



#define topic3InitProd "startStatus,%d,init\n"
#define topic3EndProd "startStatus,%d,end\n"





#define DEBUG printf



uint8_t toMove = 0;


typedef enum
{
    PRODUZINDO = 0,//produzindo, naoManutencao, pecaOk
    PARADO//padado, em Manutencao, pecaDefeituosa
} STATE_MAIN;


STATE_MAIN producao = PARADO;

STATE_MAIN manutencao = PRODUZINDO;

STATE_MAIN pecaSemDefeito = PRODUZINDO;

static void  beat_task(void *pvParameters)
{

    TickType_t xLastWakeTime = xTaskGetTickCount();
   // char msg[PUB_MSG_LEN];
    int count = 0;
    int32_t now = xTaskGetTickCount(), tempoParada = xTaskGetTickCount(), tempoIniciouProducao = xTaskGetTickCount();
    int32_t nextEventTime = now;
    int i, n=0; 
    uint8_t scan[MAX_INPUT_NUMBER], scanPrev[MAX_INPUT_NUMBER];



    gpio_enable(linkLedPin, GPIO_OUTPUT);
    uint8_t estado = 0;
    gpio_write(linkLedPin, estado);







    vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);//limpar entradas temporárias
   /* for(i=0; i < MAX_INPUT_NUMBER; i++)
    {
        scan[i] = gpio_read(portArray[i]);
        scanPrev[i] = scan[i];
    }
*/


    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);

        //now = xTaskGetTickCount();
        for(i=0; i < MAX_INPUT_NUMBER; i++)
        {
            scan[i] = gpio_read(portArray[i]);
            now = xTaskGetTickCount();




            if(scan[i] != scanPrev[i])
            {

 /*               if(scan[ON_OFF_BT] == 1)//se botao ligado ok
                {
*/
                    scanPrev[i] = scan[i];
                  
                   // printf("switch [%u] d[%d] detectec at: [%d]ms\n",scan[i],i,now);

                    if(  (scan[BOTAO_PRODUCAO] == 1)  && (scan[i] == 1) ) 
                    {
                       // if(manutencao == PRODUZINDO)
                       // {
                            if(producao == PARADO) 
                            {
                                    producao = PRODUZINDO;

                                    if(manutencao == PRODUZINDO ) printf(topic3InitProd,now*10);

                                    tempoIniciouProducao = xTaskGetTickCount();
                            }
                            else 
                            {
                                producao = PARADO;
                                 if(manutencao == PRODUZINDO ) printf(topic3EndProd,now*10);
                            }
                       // }
                        //printf("mudou estado\n");
                    }



                    if( i == BOTAO_MANUTENCAO ) //tratar somente botao manutencao
                    {
                        if(scan[BOTAO_MANUTENCAO] == 1) //se apertou comeca a contagem de parada
                        {
                                manutencao = PARADO;
                               // producao = PARADO;
                                
                                printf(topic2PauseStart,now*10);
                         //       DEBUG("entrou em parada manutencao\n");
                        }
                        else 
                        {
                                manutencao = PRODUZINDO;//finalizar a contagem de parda e enviar o evento
                                printf(topic2PauseStop,now*10);
                           //      DEBUG("terminou manutencao\n");
                                
                        }
                    }

                    
                    if( i == BOTAO_PECAS_DEFEITO )//para definir o tipo de peca a se preduzir
                    {

                        if(scan[BOTAO_PECAS_DEFEITO] == 1) pecaSemDefeito = PARADO;
                        else pecaSemDefeito = PRODUZINDO;



                    }
                    

                    if( i == SENSOR_PRODUCAO )
                    {

                        if(producao == PRODUZINDO)  //somente notificar pecas se o estado for produzindo
                        if(scan[SENSOR_PRODUCAO] == 1) //se houver novo produto
                        {
                            if(pecaSemDefeito == PRODUZINDO)   printf(topic1Good,now*10);
                            else  printf(topic1Bad,now*10);

                        }


                    }

/*                }
*/


/*                snprintf(msg, PUB_MSG_LEN, "Beat %d,%d\r\n", i,scan[i]);

                if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) 
                {
                    printf("Publish queue overflow.\r\n");

                }

*/

            }
            
        }
        

/*        if(scan[ON_OFF_BT] == 1)
        {
*/
            if((producao == PRODUZINDO) && (manutencao != PARADO))
            {
                       
                    toMove = 1; 
                   
               
            }
            else
            {

                toMove = 0;
            }


            if(producao == PRODUZINDO)
            {
                if(  (xTaskGetTickCount() - tempoIniciouProducao) > 479*100)
                {


                    if(manutencao == PARADO) printf(topic2PauseStop,now*10);
                        
                   


                     //producao = PARADO;
                     printf(topic3EndProd,now*10);
                      tempoIniciouProducao = xTaskGetTickCount();
                      printf(topic3InitProd,now*10);

                    if(manutencao == PARADO) printf(topic2PauseStart,now*10);


                }

            }

        
            n++;
            if(n%10 == 0)
            {
                estado = !estado;
                gpio_write(linkLedPin, estado);
            }

 /*       }
        else toMove = 0;
*/
    }
}
/*
static void  topic_received(mqtt_message_data_t *md)
{
    int i;
    mqtt_message_t *message = md->message;
    printf("Received: ");
    for( i = 0; i < md->topic->lenstring.len; ++i)
        printf("%c", md->topic->lenstring.data[ i ]);

    printf(" = ");
    for( i = 0; i < (int)message->payloadlen; ++i)
        printf("%c", ((char *)(message->payload))[i]);

    printf("\r\n");
}

static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static void  mqtt_task(void *pvParameters)
{
    int ret         = 0;
    struct mqtt_network network;
    mqtt_client_t client   = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
               MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100,
                      mqtt_readbuf, 100);

        data.willFlag       = 0;
        data.MQTTVersion    = 3;
        data.clientID.cstring   = mqtt_client_id;
        data.username.cstring   = MQTT_USER;
        data.password.cstring   = MQTT_PASS;
        data.keepAliveInterval  = 10;
        data.cleansession   = 0;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\r\n");
        mqtt_subscribe(&client, "/esptopic", MQTT_QOS1, topic_received);
        xQueueReset(publish_queue);

        while(1){

            char msg[PUB_MSG_LEN - 1] = "\0";
            while(xQueueReceive(publish_queue, (void *)msg, 0) ==
                  pdTRUE){
                printf("got message to publish[%s]\r\n", msg);
                mqtt_message_t message;
                message.payload = msg;
                message.payloadlen = PUB_MSG_LEN;
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, "/beat", &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing message: %d\n", ret );
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n\r");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }
}

static void  wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("WiFi: connecting to WiFi\n\r");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status );
            if( status == STATION_WRONG_PASSWORD ){
                printf("WiFi: wrong password\n\r");
                break;
            } else if( status == STATION_NO_AP_FOUND ) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if( status == STATION_CONNECT_FAIL ) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        }
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        printf("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

*/




void taskPwm(void *pvParameters)
{
//    printf("Hello from taskPwm!\r\n");
    uint32_t const init_count = 1441;
    uint32_t count = init_count;



    uint8_t pins[1];
    //printf("pwm_init(1, [%d])\n",PWM1_PIN);
    pins[0] = PWM1_PIN;
    pwm_init(1, pins, false);
   // printf("pwm_set_freq(1000)     # 1 kHz\n");
    pwm_set_freq(50);
   // printf("pwm_set_duty(UINT16_MAX/2)     # 50%%\n");
    pwm_set_duty(UINT16_MAX/2);
   // printf("pwm_start()\n");
    pwm_start();

    uint8_t state=0;





    while(1)
    {


        uint32_t randTimer = hwrand();
     //deixar variando entre 400 e 900

        randTimer%=500;



    //    vTaskDelay(500 / portTICK_PERIOD_MS);
            vTaskDelay((400+ randTimer)/ portTICK_PERIOD_MS);
  //      printf("duty cycle set to %d/UINT16_MAX%%\r\n", count);


        if(toMove)
        {
            pwm_set_duty(count);
     //       printf("to move %u\n", count);

            count += UINT16_MAX/50;
            if (count > 3930) count = init_count;


/*
to move 0
to move 1310
to move 2620
to move 3930
to move 5240
to move 6550
to move 7860

*/
/*
            if(state == 0){  count = 0; state = 1; continue;}

           if(state == 1){  count =  UINT16_MAX/2; state = 2; continue;}

            if(state == 1){  count = 1310; state = 0; continue;}
*/
        


            
            
        }


        //usar d7 e d8 - d7 botao liga/desliga(combinar com led) e d8 saida do led de rede


       



    }
}





void user_init(void)
{



    uart_set_baud(0, 115200);
//    printf("SDK version:%s\n", sdk_system_get_sdk_version());



 sdk_wifi_set_opmode(NULL_MODE);
 /*
sdk_wifi_softap_stop();
*/
 sdk_wDevDisableRx();
    sdk_wDev_DisableTransmit();
    sdk_phy_disable_agc();


for(int i = 0; i < MAX_INPUT_NUMBER; i++) //primeiras 4 portas como entrada
        gpio_enable(portArray[i], GPIO_INPUT);



/*
    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);

*/

 //xTaskCreate(monitoraSensor, "monitoraSensor", 256, NULL, 1, NULL);




//    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&beat_task, "beat_task", 256, NULL, 3, NULL);
//    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);

xTaskCreate(taskPwm, "taskPwm", 256, NULL, 2, NULL);

   

}
