

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
#define MQTT_HOST ("192.168.23.31")
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




#define MQTT_USER NULL
#define MQTT_PASS NULL

SemaphoreHandle_t wifi_alive;
QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

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
                    printf("ret:[%d] \n",ret);
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

/*
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
                printf("got message to publish\r\n");
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




*/
    
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
}