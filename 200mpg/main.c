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
#define LOAD_ICON_X 54
#define LOAD_ICON_Y 42
#define LOAD_ICON_SIZE 20
#define CIRCLE_COUNT_ICON_X 100
#define CIRCLE_COUNT_ICON_Y 52
/* Remove this line if your display connected by SPI */
#define I2C_CONNECTION
#include <i2c/i2c.h>
#include "fonts/fonts.h"
/* Change this according to you schematics and display size */
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64
    #define PROTOCOL SSD1306_PROTO_I2C
    #define ADDR_DISPLAY     SSD1306_I2C_ADDR_0
    #define I2C_BUS  0
  //  #define SCL_PIN  5
  //  #define SDA_PIN  4
    #define SCL_PIN  13
    #define SDA_PIN  12

/* pin config */
const int INTERRUPT_PCF8574_PIN = 14;   /* gpio 15 usually has "PROGRAM" button attached */
#define PWM1_PIN 16
#define ADD_PCF8574  0x20  //endereco correto sem o bit de leitura/escrita (a0 a1 a2 em GND)


#define DEFAULT_FONT FONT_FACE_TERMINUS_6X12_ISO8859_1
/* Declare device descriptor */
static const ssd1306_t dev = {
    .protocol = PROTOCOL,
    .i2c_dev.bus      = I2C_BUS,
    .i2c_dev.addr     = ADDR_DISPLAY,
    .width    = DISPLAY_WIDTH,
    .height   = DISPLAY_HEIGHT
};
static const i2c_dev_t devPcf8574 = {
    .bus      = I2C_BUS,   //mesmo barramento do display
    .addr     = ADD_PCF8574
};

/* Local frame buffer */
static uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];


TimerHandle_t fps_timer_handle = NULL; // Timer handler
TimerHandle_t font_timer_handle = NULL;


uint8_t frame_done = 0; // number of frame send.
uint8_t fps = 0; // image per second.
const font_info_t *font = NULL; // current font
font_face_t font_face = 0;
#define SECOND (1000 / portTICK_PERIOD_MS)

uint8_t itemIndex;
typedef struct
{
char *linha;
int16_t valor;
char *descritor;
}menuLine;
//todo: dynamic
menuLine menuArray[] =
{
{"tensao pico", 500,"mv"},
{"tempo decrescer", 8000,"ms"},
{"tensao vale", 100,"mv"},
{"tempo crescer", 7000,"ms"},
{"numero passo", 0,"steps"},
{"testegrd", 0,"xx"},
/*{"textLinha6", 0},
{"textLinha7", 0},
{"textLinha8", 0},
{"textLinha9", 0},
{"textLinha10", 0},*/
};
#define MAX_ITENS_MENU sizeof(menuArray)/sizeof(menuLine)

menuLine * selected;
/*
menuLine *menuShow[4];
menuShow[0] = &menuArray[0];
menuShow[1] = &menuArray[1];
menuShow[2] = &menuArray[2];
menuShow[3] = &menuArray[3];
*/
int8_t indexShowing[4] = {0 , 1, 2, 3};
uint8_t menuSelected = 2;

typedef enum
{
    MENU_OPTIONS=0,
    MENU_EDITING,
    INPUTS,
    IDLE
} FRAMESHOWING;
FRAMESHOWING frameShowing = MENU_OPTIONS;
uint8_t keyToGet = false;

typedef enum // mapear aqui as teclas de acordo com o endereco
{

TECLA_NONE        =0,
TECLA_UP          =251,
TECLA_DOWN        =191,
TECLA_DIREITA     =254,
TECLA_ESQUERDA    =223,
TECLA_MENOS       =239,
TECLA_MAIS      =253,
TECLA_CANCEL   =247,
TECLA_ENTER   =127
}KEY_MAP;

KEY_MAP teclaCorrente;


static void handleI2cDevices(void *pvParameters)
{
    printf("%s: Started user interface task\n", __FUNCTION__);
    vTaskDelay(SECOND);
    ssd1306_set_whole_display_lighting(&dev, false);
    char text[20];
    uint8_t i;
    uint16_t count = 0;
    ssd1306_set_contrast (&dev, 0xff);
    
    uint botaoApertado;

while (1)
{
        vTaskDelay(SECOND/10);
        //ssd1306_fill_rectangle(&dev, buffer, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT/2, OLED_COLOR_BLACK);
        ssd1306_fill_rectangle(&dev, buffer, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, OLED_COLOR_BLACK);
        switch(frameShowing)//telas atuais
        {
            case MENU_OPTIONS:
            font = font_builtin_fonts[5];
            sprintf(text, "%s: %d", menuArray[indexShowing[0]].linha, menuArray[indexShowing[0]].valor);
            ssd1306_draw_string(&dev, buffer, font_builtin_fonts[DEFAULT_FONT], 0, 0, text, menuSelected == 0? OLED_COLOR_BLACK:OLED_COLOR_WHITE, menuSelected == 0? OLED_COLOR_WHITE:OLED_COLOR_BLACK);
            sprintf(text, "%s: %d", menuArray[indexShowing[1]].linha, menuArray[indexShowing[1]].valor);
            ssd1306_draw_string(&dev, buffer, font_builtin_fonts[DEFAULT_FONT], 0, 15,text, menuSelected == 1? OLED_COLOR_BLACK:OLED_COLOR_WHITE, menuSelected == 1? OLED_COLOR_WHITE:OLED_COLOR_BLACK);
            sprintf(text, "%s: %d", menuArray[indexShowing[2]].linha, menuArray[indexShowing[2]].valor);
            ssd1306_draw_string(&dev, buffer, font_builtin_fonts[DEFAULT_FONT], 0, 30,text, menuSelected == 2? OLED_COLOR_BLACK:OLED_COLOR_WHITE, menuSelected == 2? OLED_COLOR_WHITE:OLED_COLOR_BLACK);
            sprintf(text, "%s: %d", menuArray[indexShowing[3]].linha, menuArray[indexShowing[3]].valor);
            ssd1306_draw_string(&dev, buffer, font_builtin_fonts[DEFAULT_FONT], 0, 45,text, menuSelected == 3? OLED_COLOR_BLACK:OLED_COLOR_WHITE, menuSelected == 3? OLED_COLOR_WHITE:OLED_COLOR_BLACK);
        
            if (ssd1306_load_frame_buffer(&dev, buffer))
                goto error_loop;
            frameShowing = IDLE;
            break;
    
        case MENU_EDITING:
            font = font_builtin_fonts[4];
                
            sprintf(text, "  %s", menuArray[indexShowing[menuSelected]].linha);
            ssd1306_draw_string(&dev, buffer, font_builtin_fonts[DEFAULT_FONT], 0, 0, text, OLED_COLOR_BLACK,OLED_COLOR_WHITE);
            font = font_builtin_fonts[9];
            sprintf(text, "  %d", menuArray[indexShowing[menuSelected]].valor);
            ssd1306_draw_string(&dev, buffer, font_builtin_fonts[DEFAULT_FONT], 0,30, text, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
           // sprintf(text, "%s: %u", menuArray[indexShowing[0]].linha, menuArray[indexShowing[0]].valor);

            //ssd1306_draw_string(&dev, buffer, font_builtin_fonts[DEFAULT_FONT], 0, 0, text, menuSelected == 0? OLED_COLOR_BLACK:OLED_COLOR_WHITE, menuSelected == 0? OLED_COLOR_WHITE:OLED_COLOR_BLACK);
           
            if (ssd1306_load_frame_buffer(&dev, buffer))
                goto error_loop;
            frameShowing = IDLE;
        break;



        case IDLE:

        break;
        };
      

            botaoApertado = pcf8574_port_read(&devPcf8574);//pooling
            if( botaoApertado!= 255)
            {
                printf("Button test value: [%u]\r\n",botaoApertado);
                //envileirar evento de botão
                teclaCorrente = botaoApertado;
            }


        frame_done++;
        
}
error_loop:
    printf("%s: error while loading framebuffer into SSD1306\n", __func__);
    for (;;) {
        vTaskDelay(2 * SECOND);
        printf("%s: error loop\n", __FUNCTION__);
    }
}

void menuNext()
{
    uint8_t menuItem =  menuSelected;
    menuItem++;
    if(menuItem >= 4)
    {
        if(indexShowing[3] > (MAX_ITENS_MENU-2) )//rotacionar para o primeiro
        {
            printf("indexShowing[3] > MAX_ITENS_MENU: [%u][%u]\n",indexShowing[3],MAX_ITENS_MENU);
            menuItem = 0;
            indexShowing[0]=0;
            indexShowing[1]=1;
            indexShowing[2]=2;
            indexShowing[3]=3;
           
        }
        else//apenas incrementar o array de ídices
        {
            indexShowing[0]++;
            indexShowing[1]++;
            indexShowing[2]++;
            indexShowing[3]++;
            menuItem = 3;
        }
        
    }
    menuSelected = menuItem;
    
    frameShowing = MENU_OPTIONS;
     printf("menuSelected: [%u]\n",menuSelected);
}
void menuPrev()
{
  int8_t menuItem =  menuSelected;
    menuItem--;
    if(menuItem < 0)
    {
            
            indexShowing[0]--;
            indexShowing[1]--;
            indexShowing[2]--;
            indexShowing[3]--;
            menuItem = 0;
            printf("xxx,menuItem %d\n",menuItem);
        if(indexShowing[0] < 0 )//rotacionar para o primeiro
        {
           
            menuItem = 3;
            indexShowing[0]=MAX_ITENS_MENU-4;
            indexShowing[1]=MAX_ITENS_MENU-3;
            indexShowing[2]=MAX_ITENS_MENU-2;
            indexShowing[3]=MAX_ITENS_MENU-1;
           printf("indexShowing[0] < MAX_ITENS_MENU: [%d][%u]\n",indexShowing[0],menuItem);
        }
    }
    menuSelected = menuItem;
    frameShowing = MENU_OPTIONS;
    
     printf("menuSelected: %u, indexShowing[%d]\n",menuSelected,indexShowing[0]);
}




void taskMenuControl(void *pvParameters)
{

typedef enum // mapear aqui as teclas de acordo com o endereco
{
    rolagemMenu=0,
    edicaoMenu
} APP_STATE;
APP_STATE state = rolagemMenu;
   //frameShowing = INPUTS;
   while (1)
   {
  
    vTaskDelay(SECOND*0.1);


    switch (state)
    {
        case rolagemMenu:

            if(teclaCorrente == TECLA_UP){ menuPrev(); teclaCorrente = TECLA_NONE;}
            if(teclaCorrente == TECLA_DOWN){ menuNext(); teclaCorrente = TECLA_NONE;}
         
             if(teclaCorrente == TECLA_DIREITA)
             { 
                frameShowing = MENU_EDITING;
                teclaCorrente = TECLA_NONE;
                state = edicaoMenu;
             }
        break;

        case edicaoMenu:

             if(teclaCorrente == TECLA_ESQUERDA)//retornar ao menu
             { 
                state = rolagemMenu;
                teclaCorrente = TECLA_NONE;
                frameShowing = MENU_OPTIONS;
             }
             if(teclaCorrente ==  TECLA_MAIS)
             {
                menuArray[indexShowing[menuSelected]].valor++;
                teclaCorrente = TECLA_NONE;
                frameShowing = MENU_EDITING;
             }
             if(teclaCorrente ==  TECLA_MENOS)
             {
                menuArray[indexShowing[menuSelected]].valor--;
                teclaCorrente = TECLA_NONE;
                frameShowing = MENU_EDITING;
             }

        break;
    }
    
    //menuPrev();
   }
}

void fps_timer(TimerHandle_t h)
{
    fps = frame_done; // Save number of frame already send to screen
    frame_done = 0;
}
/*
void font_timer(TimerHandle_t h)
{
    do {
        if (++font_face >= font_builtin_fonts_count)
            font_face = 0;
        font = font_builtin_fonts[font_face];
    } while (!font);
    printf("Selected builtin font %d\n", font_face);
}
*/

void taskPwm(void *pvParameters)
{
//    printf("Hello from taskPwm!\r\n");
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
}

void taskAdcRead()
{
 //   printf("Hello from taskAdcRead!\r\n");
    uint16_t adcInput = 0;

    while(1)
    {
        vTaskDelay(100);
        adcInput = sdk_system_adc_read();
  //      printf("adcRead: %u\r\n", adcInput);
 
    }
}

void taskKeyboardRead()
{

    uint8_t teste = pcf8574_port_read(&devPcf8574);
}

const int active = 1; /* active == 0 for active low */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;
/* This task configures the GPIO interrupt and uses it to tell
   when the button is pressed.
   The interrupt handler communicates the exact button press time to
   the task via a queue.
   This is a better example of how to wait for button input!
*/
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
}

void user_init(void)
{
    //uncomment to test with CPU overclocked
    //sdk_system_update_cpu_freq(160);
    // Setup HW
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);

    while (ssd1306_init(&dev) != 0) {
        printf("%s: failed to init SSD1306 lcd\n", __func__);
        vTaskDelay(SECOND);
    }
   // vTaskDelay(SECOND);
    ssd1306_set_whole_display_lighting(&dev, true);
   // vTaskDelay(SECOND);

    font = font_builtin_fonts[font_face];
    // Create user interface task
    xTaskCreate(handleI2cDevices, "handleI2cDevices", 256, NULL, 2, NULL);
    fps_timer_handle = xTimerCreate("fps_timer", SECOND, pdTRUE, NULL, fps_timer);
    xTimerStart(fps_timer_handle, 0);
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
   xTaskCreate(taskMenuControl, "taskAdcRead", 256, NULL, 2, NULL);

  //  gpio_enable(INTERRUPT_PCF8574_PIN, GPIO_INPUT);
  //  tsqueue = xQueueCreate(2, sizeof(uint32_t));
  //  xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
 
 //   xTaskCreate(buttonPollTask, "buttonPollTask", 256, NULL, 1, NULL);


}

