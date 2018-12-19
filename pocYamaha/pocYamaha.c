

#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <semphr.h>

#include <lwip/api.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"





/* You can use http://test.mosquitto.org/ to test mqtt_client instead
 * of setting up your own MQTT server */


//#define MQTT_HOST ("192.168.23.31")
#define MQTT_HOST ("172.20.4.190")

#define MQTT_PORT 25000


#define WIFI_SSID "yma"
#define WIFI_PASS "ymaitriad0011"


/*
#define WIFI_SSID "TCTSmartPhones"
#define WIFI_PASS "TCTSMARTPHONES"
*/


#define MAX_INPUT_NUMBER 5
u_int portArray[]={16,5,4,14,13}; //d0 d1 d2 d5
void configIo(void)
{
for(int i = 0; i < MAX_INPUT_NUMBER; i++) //primeiras 4 portas como entrada
        gpio_enable(portArray[i], GPIO_INPUT);

}



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




SemaphoreHandle_t wifi_alive;
QueueHandle_t publish_queue;
#define PUB_MSG_LEN 64


bool empilharMensagem(char *msg)
{
         if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) 
        {
            printf("Publish queue overflow.\r\n");
            return false;
        }
  return true;
}

char buffTemp[32];

static void  beat_task(void *pvParameters)
{

    TickType_t xLastWakeTime = xTaskGetTickCount();
  
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

                                    if(manutencao == PRODUZINDO ) 
                                        {
                                            sprintf(buffTemp,topic3InitProd,now*10);
                                            empilharMensagem(buffTemp);
                                           // printf(topic3InitProd,now*10);
                                        }

                                    tempoIniciouProducao = xTaskGetTickCount();
                            }
                            else 
                            {
                                producao = PARADO;
                                if(manutencao == PRODUZINDO )
                                {

                                    sprintf(buffTemp,topic3EndProd,now*10);
                                    empilharMensagem(buffTemp);
                                    //printf(topic3EndProd,now*10);
                                }
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
                                sprintf(buffTemp,topic2PauseStart,now*10);
                                empilharMensagem(buffTemp);
                                //printf(topic2PauseStart,now*10);
                        }
                        else 
                        {
                                manutencao = PRODUZINDO;//finalizar a contagem de parda e enviar o evento
                                sprintf(buffTemp,topic2PauseStop,now*10);
                                empilharMensagem(buffTemp);
                                //printf(topic2PauseStop,now*10);
                                
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
                            if(pecaSemDefeito == PRODUZINDO)
                            {
                                sprintf(buffTemp,topic1Good,now*10);
                                empilharMensagem(buffTemp);
                                //printf(topic1Good,now*10);
                            }
                            else
                            {
                                sprintf(buffTemp,topic1Bad,now*10);
                                empilharMensagem(buffTemp);
                                //printf(topic1Bad,now*10);
                            }
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


                    if(manutencao == PARADO)
                    {

                        sprintf(buffTemp,topic2PauseStop,now*10);
                        empilharMensagem(buffTemp);
                        //printf(topic2PauseStop,now*10);
                    }

                    sprintf(buffTemp,topic3EndProd,now*10);
                    empilharMensagem(buffTemp);
                    //printf(topic3EndProd,now*10);
                    

                    tempoIniciouProducao = xTaskGetTickCount();
                    

                    sprintf(buffTemp,topic3InitProd,now*10);
                    empilharMensagem(buffTemp);
                    //printf(topic3InitProd,now*10);

                    if(manutencao == PARADO)
                    {
                        sprintf(buffTemp,topic2PauseStart,now*10);
                        empilharMensagem(buffTemp);
                        //printf(topic2PauseStart,now*10);
                    }

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
static void  beat_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];
    int count = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
        printf(".\n");
        snprintf(msg, PUB_MSG_LEN, "Beat %d\r\n", count++);
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
            printf("Publish queue overflow.\r\n");
        }
    }
}

*/
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

    printf("mcadd:[%s]\n",my_id);

    return my_id;
}



static int  host2addr(const char *hostname , struct in_addr *in)
{
    struct addrinfo hints, *servinfo, *p;
    struct sockaddr_in *h;
    int rv;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    rv = getaddrinfo(hostname, 0 , &hints , &servinfo);
    if (rv != 0)
    {
        return rv;
    }

    // loop through all the results and get the first resolve
    for (p = servinfo; p != 0; p = p->ai_next)
    {
        h = (struct sockaddr_in *)p->ai_addr;
        in->s_addr = h->sin_addr.s_addr;
    }
    freeaddrinfo(servinfo); // all done with this structure
    return 0;
}

int  network_connect(int * mysocket, const char* host, int port)
{
    struct sockaddr_in addr;
    int ret;

    if (host2addr(host, &(addr.sin_addr)) != 0)
    {
        return -1;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    *mysocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(*mysocket < 0 )
    {
        // error
        return -1;
    }
    ret = connect(*mysocket, ( struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    if( ret < 0 )
    {
        // error
        close(*mysocket);
        return ret;
    }

    return ret;
}

int  network_disconnect(int * mysocket)
{
    close(*mysocket);
    *mysocket = -1;
    return 0;
}



int  network_write(int * mySocket, unsigned char* buffer, int len, int timeout_ms)
{
    struct timeval tv;
    fd_set fdset;
    int rc = 0;

    FD_ZERO(&fdset);
    FD_SET(*mySocket, &fdset);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    rc = select(*mySocket + 1, 0, &fdset, 0, &tv);
    if ((rc > 0) && (FD_ISSET(*mySocket, &fdset)))
    {
        rc = send(*mySocket, buffer, len, 0);
    }
    else
    {
        // select fail
        return -1;
    }
    return rc;
}


/*todo Read*/

char msg[PUB_MSG_LEN - 1] = "\0";

static void  mqtt_task(void *pvParameters)
{

    int ret         = 0;
    int mySocket = -1;
       
    xQueueReset(publish_queue);//fila de eventos a serem descarregados


    while(1) 
    {
        vTaskDelay( 100 / portTICK_PERIOD_MS );
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to server %s ... ",__func__,
               MQTT_HOST);
        ret = network_connect(&mySocket, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("conectou no socket\n\r");

        ret = 1;
        while(ret>0){

            int temItens = 1;
            while(temItens)
            {
                if( xQueuePeek(publish_queue, (void *)msg, 0) ==   pdTRUE)
                {
                    ret = network_write(&mySocket, msg, strlen(msg),1000);
                    if(ret > 0) 
                    {
                        xQueueReceive(publish_queue, (void *)msg, 0);
                        printf("ok [%s]desempilhando\n",msg);
                    }
                    else printf("erro comunicacao - ret:[%d] \n",ret);
                }
                else temItens = 0;

                vTaskDelay( 100 / portTICK_PERIOD_MS );
                if(ret < 0) temItens = 0;

            }

            vTaskDelay( 100 / portTICK_PERIOD_MS );
            //taskYIELD();
            
            

        }
        printf("Connection dropped, request restart\n\r");
        network_disconnect(&mySocket);
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
        vTaskDelay((400+ randTimer)/ portTICK_PERIOD_MS);
        if(toMove)
        {
            pwm_set_duty(count);
              count += UINT16_MAX/50;
            if (count > 3930) count = init_count;      
            
        }


    }
}



void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());


    configIo();

    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(10, PUB_MSG_LEN);
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&beat_task, "beat_task", 256, NULL, 3, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
    xTaskCreate(taskPwm, "taskPwm", 256, NULL, 2, NULL);

}