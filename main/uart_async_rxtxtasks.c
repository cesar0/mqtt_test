/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <freertos/queue.h>
#include "uart_async_rxtxtasks.h"

static const char *TAG = "uart_example";


#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_5)

static const int RX_BUF_SIZE = 1024;


static QueueHandle_t uart_queue;
uart_packet_t  m_uart_packet;               //保存当前标签数据

bt_state_t m_bt_state;
uint32_t m_bt_state_timer;


void init(void)
{
    m_bt_state = BT_IDLE;
    memset(&m_uart_packet, 0, sizeof(m_uart_packet));
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 创建队列
    uart_queue = xQueueCreate(10, sizeof(uart_packet_t));
}


// 串口任务发送函数
void uart_push_queue(const char *pcmd,const char *psn, const uint8_t *data, size_t length, uint8_t retry) {
    uart_packet_t packet;

    if(0 != memcmp(pcmd, "wb", 2))    return;                  //只处理wb
    
    // 分配内存保存数据
    packet.data = (uint8_t *)malloc(length);
    if (packet.data == NULL) {
        ESP_LOGI(TAG, "uart_push_queue data is NULL");
        return;
    }
    if (length == 0) {
        ESP_LOGI(TAG, "uart_push_queue LENGTH is 0");
        return;
    }

    // 复制数据
    memcpy(packet.cmd, pcmd, strlen(pcmd)+1);
    memcpy(packet.sn, psn, strlen(psn)+1);
    memcpy(packet.data, data, length);
    packet.retry = retry;
    packet.length = length;
    ESP_LOGI(TAG, "uart_queue %s %s len=%d\n",packet.cmd, packet.sn, length);
    if('A' == data[0]  && 'T' == data[1]){
        ESP_LOGI(TAG, "%s\n",data);
    }

    // 检查队列剩余空间
    UBaseType_t spacesAvailable = uxQueueSpacesAvailable(uart_queue);
    ESP_LOGI(TAG,"Queue spaces available: %lu\n", (unsigned long)spacesAvailable);
    
    // 将数据包发送到队列
    xQueueSend(uart_queue, &packet, 0);
}

// UART 发送任务
void uart_tx_task(void *pvParameters) {
    uart_packet_t packet;

    while (1) {
        // 等待从队列中接收数据
        if (xQueuePeek(uart_queue, &packet, portMAX_DELAY)) {
            switch(m_bt_state){
            case BT_IDLE:
            case BT_READY:
            case BT_DISCONN:
                //发送连接指令
                // char *at_conn_dev = "AT+CESL=85AC00093535\r";
                // char *at_conn_dev = "AT+CESL=665544332212\r";
                char at_conn_dev[32];
                memset(at_conn_dev, 0, sizeof(at_conn_dev));
                memcpy(at_conn_dev,"AT+CESL=", strlen("AT+CESL="));
                memcpy(at_conn_dev + strlen("AT+CESL="),packet.sn, 12);
                memcpy(at_conn_dev + strlen("AT+CESL=") + 12 ,"\r", 1);
                uart_write_bytes(UART_NUM_2,(const uint8_t *)at_conn_dev, strlen(at_conn_dev));
                m_bt_state = BT_CONNING;
                m_bt_state_timer = xTaskGetTickCount() * portTICK_PERIOD_MS;
                ESP_LOGI(TAG, "uart_write_bytes len=%d: %s\n",strlen(at_conn_dev),at_conn_dev);
                break;
            case BT_CONNING:
                ESP_LOGI(TAG,"conning %s\n", packet.sn);

                 if(xTaskGetTickCount() * portTICK_PERIOD_MS - m_bt_state_timer > 25*1000){
                    ESP_LOGE(TAG,"uart bt connect timeout.....\n");
                    do{
                        if (xQueueReceive(uart_queue, &packet, 0) == pdPASS) {      
                            ESP_LOGE(TAG,"BT_CONNTIMEOUT consumer msg: %s %s...\n",packet.cmd, packet.sn);
                        }else{
                            break;
                        }
                    }while(memcmp(packet.cmd, "wb", 2));

                    m_bt_state = BT_DISCONN;
                }else{
                    vTaskDelay(pdMS_TO_TICKS(2000));  // 1000毫秒 = 1秒
                }  
                break;
            case BT_CONN:
                if (xQueueReceive(uart_queue, &packet, 0) == pdPASS) {
                    // ESP_LOGI(TAG, "uart_tx_task=%.*s", packet.length, (const char *)packet.data);
                    // 发送数据到串口
                    if(0 == memcmp(packet.cmd, "wb", 2)){ 
                        uart_write_bytes(UART_NUM_2, (const char *)packet.data, packet.length);
                        ESP_LOGI(TAG, "uart_write_bytes %s %s len=%d.....\n",packet.cmd, packet.sn, packet.length);

                        // 发送完成后等待断连连接再释放数据
                        m_bt_state_timer = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        m_uart_packet = packet;
                        m_bt_state = BT_CONN_WRITE_END;
                    }
                    
                }
                break;
            case BT_CONN_WRITE_END:
                if(xTaskGetTickCount() * portTICK_PERIOD_MS - m_bt_state_timer > 10*1000){
                    ESP_LOGE(TAG,"uart bt disconnect timeout.....\n");
                    free(m_uart_packet.data);
                    memset(&m_uart_packet, 0, sizeof(m_uart_packet));
                    m_bt_state = BT_DISCONN;
                }else{
                    vTaskDelay(pdMS_TO_TICKS(100));
                }  

                break;
            default:
                vTaskDelay(pdMS_TO_TICKS(10));  // 1000毫秒 = 1秒
            break;
            }
        }
    }
}


// IM_DISC,0001 REASON:13
char* uart_decode_value(char* buf, int len)
{
    int index=0;
    char* uart_msg[5];
    char *datp;

    datp = strtok(buf, ":");
    while(datp){
        uart_msg[index] = datp;
        // ESP_LOGI("\t%s", datp);
        index++;
        datp = strtok(NULL,":");
    }

    // ESP_LOGI(TAG,"uart_msg: <cmd>%s\n <val>%s\n",uart_msg[0],uart_msg[1]);
    return uart_msg[1];
}

static void uart_rx_task(void *arg)
{   
    uint8_t* data;
    uint8_t* raw_data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    if(NULL == raw_data){
         ESP_LOGI(TAG, "rx_task malloc NULL.");
    }
    while (1) {
        int rxBytes = uart_read_bytes(UART_NUM_2, raw_data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            raw_data[rxBytes] = 0;
            data = raw_data;

            if(('\r' == data[0]) || ('\n' == data[0])){
                data++;
                rxBytes--;
                if (0 == rxBytes) continue;
                if(('\r' == data[0]) || ('\n' == data[0])){
                    data++;
                    rxBytes--;
                }
            }
            if (0 == rxBytes) continue;
            ESP_LOGI(TAG, "Read %d bytes: '%s'", rxBytes, data);
            
            if(strlen("IM_REAY") <= rxBytes && 
                ((0 == memcmp("IM_REAY", data, strlen("IM_REAY")))
                || (0 == memcmp("IM_READY", data, strlen("IM_READY"))))){
                m_bt_state = BT_READY;
                ESP_LOGI(TAG, "BT_READY...");
            }else if(strlen("IM_CONN ") <= rxBytes &&
                ((0 == memcmp("\r\nIM_CONN", data, strlen("\r\nIM_CONN")))
                || (0 == memcmp("IM_CONN", data, strlen("IM_CONN"))))){
                vTaskDelay(pdMS_TO_TICKS(20));  // 20毫秒延时后再设置conn
                m_bt_state = BT_CONN;
                ESP_LOGI(TAG, "BT_CONN...");
            }else if(strlen("IM_DISC") <= rxBytes && 
                 ((0 == memcmp("\r\nIM_DISC", data, strlen("\r\nIM_DISC")))
                || (0 == memcmp("IM_DISC", data, strlen("IM_DISC"))))){
                m_bt_state = BT_DISCONN;
                char* reson_str = uart_decode_value((char*)data, rxBytes);
                ESP_LOGI(TAG, "BT_DISCONN sn=%s retry=%d reson=%s,...",m_uart_packet.sn,m_uart_packet.retry,reson_str);
                if(memcmp("13", reson_str, strlen("13"))){
                    if(m_uart_packet.retry--){
                        ESP_LOGE(TAG, "ERROR retry sn=%s!",m_uart_packet.sn);
                        uart_push_queue(m_uart_packet.cmd,m_uart_packet.sn, m_uart_packet.data, m_uart_packet.length, m_uart_packet.retry);
                    }
                }
                free(m_uart_packet.data);
                memset(&m_uart_packet, 0, sizeof(m_uart_packet));
            }
        }
    }
    free(raw_data);
}


// UART 发送任务
void uart_task(void *pvParameters) {
    uart_packet_t packet;

    while (1) {
        switch(m_bt_state){
            case BT_IDLE:
                ESP_LOGI(TAG,"m_bt_state=%d BT_IDLE\n",m_bt_state);
                break;
            case BT_READY:
                ESP_LOGI(TAG,"m_bt_state=%d BT_READY\n",m_bt_state);
                break;
            case BT_DISCONN:
                ESP_LOGI(TAG,"m_bt_state=%d BT_DISCONN\n",m_bt_state);
                break;
            case BT_CONNING:
                ESP_LOGI(TAG,"m_bt_state=%d BT_CONNING\n",m_bt_state);
                break;
            case BT_CONN:
                ESP_LOGI(TAG,"m_bt_state=%d BT_CONN\n",m_bt_state);
                break;
            case BT_CONN_WRITE_END:
                ESP_LOGI(TAG,"m_bt_state=%d BT_CONN_WRITE_END\n",m_bt_state);
                break;
            default:
                break;
        }
        // 等待1秒
        vTaskDelay(pdMS_TO_TICKS(10000));  // 1000毫秒 = 1秒
    }
}


void app_uart_start(void)
{
    init();
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    
    // 创建 UART 发送任务
    xTaskCreate(uart_tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES-5, NULL);

    // xTaskCreate(uart_task, "uart_task", 1024 * 2, NULL, configMAX_PRIORITIES-10, NULL);
}
