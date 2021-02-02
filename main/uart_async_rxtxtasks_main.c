/*  PID
    https://blog.csdn.net/kilotwo/article/details/79828201
    PID控制通过积分作用消除误差，而微分控制可缩小超调量、加快系统响应，是综合了PI控制和PD控制长处并去除其短处的控制。
*/

/*
    CRC not activated yet
    
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"
#include "string.h"

static const int RX_BUF_SIZE = 1024;

//IMU HI229 UART in
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define LM    18        //left motor
#define RM    19        //right motor
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<LM) | (1ULL<<RM))


/*定义PID结构体*/
//https://blog.csdn.net/liangbin414/article/details/88596320
typedef struct
{
    float setpoint;       //设定值
    float Kp;     //比例系数
    float Ki;      //积分系数
    float Kd;    //微分系数
    float lasterror;     //前一拍偏差
    float preerror;     //前两拍偏差
    float result; //输出值
}PID;


typedef struct
{
uint8_t tag; /* data packet tag */
uint8_t id;
uint8_t rev[6]; /* reserved */
uint32_t ts; /* timestamp */
float acc[3];
float gyr[3];
float mag[3];
float eul[3]; /* eular angles: Roll,Pitch,Yaw */
float quat[4]; /* quaternion */
}id0x91_t;


float PIDRegulation(PID *vPID, float processValue)
{
    float thisError;
    float increment;
    float pError,dError,iError;
    
    thisError=vPID->setpoint-processValue; //当前误差等于设定值减去当前值
    //计算公式中除系数外的三个 乘数
    pError = thisError - vPID->lasterror; //两次偏差差值err(k)-err(k-1)
    iError = thisError;
    dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

    increment = vPID->Kp*pError + vPID->Ki*iError + vPID->Kd * dError;   //增量计算

    vPID->preerror=vPID->lasterror;  //存放偏差用于下次运算
    vPID->lasterror=thisError;
    
    vPID->result+=increment;    //结果当然是上次结果 加上本次增量

    return increment;
}


void init() {
    
    //D:\Coding\ESP\IDF323\examples\peripherals\gpio\main\gpio_example_main.c  62:74
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //put 'em low
    gpio_set_level(LM, 0);
    gpio_set_level(RM, 0);

    /* Configure UART */
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /*  Definition from D:\Coding\ESP\IDF323\components\freemodbus\port\portserial.c :233

    Install UART driver, and get the queue.
    uart_driver_install(ucUartNumber, MB_SERIAL_BUF_SIZE, MB_SERIAL_BUF_SIZE, MB_QUEUE_LENGTH, &xMbUartQueue, ESP_INTR_FLAG_LOWMED);
    */
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, RX_BUF_SIZE, 0, NULL, 0);
    
    /* Setting up HI229 by AT instructions
    AT+SETPTL=A0,B0,C0,D0,D1 */
    const char* odr = "AT+ODR=0\r\n";
    const char* setup = "AT+SETPTL=A0,B0,C0,D0,D1\r\n";
    
    int len = strlen(odr);
    uart_write_bytes(UART_NUM_1, odr, len);
    
    //vTaskDelay(50 / portTICK_RATE_MS);

    len = strlen(setup);
    uart_write_bytes(UART_NUM_1, setup, len);
}


static void Process_HI229_data()
{
    static const char *HI229_TAG = "HI229";
    esp_log_level_set(HI229_TAG, ESP_LOG_INFO);

    const char* trg = "AT+TRG\r\n";
    const int len = strlen(trg);
    int16_t pitch, roll, yaw;
    float pidout;
    //uint16_t payload_len;
    //uint16_t crc;
    //float qw, qx;
    PID aPID = {
        .setpoint = 1800,
        .Kp = 1,
        .Ki = 1,
        .Kd = 1,
    };
    
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        uart_write_bytes(UART_NUM_1, trg, len);
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 200 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //ESP_LOGI(HI229_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(HI229_TAG, data, rxBytes, ESP_LOG_INFO);
            //printf("%02X:%02X:%02X:%02X\n", data[0], data[1], data[2], data[3]);
            //crc = (data[5]<<8) + data[4] ;

            //eul cal
            pitch = (int16_t)((data[29] << 8) + data[28]) / 100;
            roll = (int16_t)((data[31] << 8) + data[30]) / 100;
            yaw = (int16_t)((data[33] << 8) + data[32]) + 1799;
            printf("pitch: %d\nroll: %d\nyaw: %d\n", pitch, roll, yaw);

            //quaternion
            //qw = (float)(data[]<<24)

            //PID control
            
            pidout = PIDRegulation(&aPID, yaw);
            printf("PIDout: %f\n", pidout);
        }
    }
    free(data);
}

void app_main()
{
    init();
    xTaskCreate(Process_HI229_data, "uart_Process", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
