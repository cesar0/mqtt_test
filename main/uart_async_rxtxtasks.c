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


static const int RX_BUF_SIZE = 1024;
static QueueHandle_t uart_queue;


#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_5)


bt_state_t m_bt_state;

void init(void)
{
    m_bt_state = BT_IDLE;
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
void uart_send_data(const uint8_t *data, size_t length) {
    uart_packet_t packet;

    // 分配内存保存数据
    packet.data = (uint8_t *)malloc(length);
    if (packet.data == NULL) {
        ESP_LOGI(TAG, "uart_send_data data is NULL");
        return;
    }
    if (length == 0) {
        ESP_LOGI(TAG, "uart_send_data LENGTH is 0");
        return;
    }

    // 复制数据
    memcpy(packet.data, data, length);
    packet.length = length;
    ESP_LOGI(TAG, "uart_send_data len=%d\n",length);

    // if (portCHECK_IF_IN_ISR()) {
    //     // 当前在中断上下文中
    //     ESP_LOGI(TAG, "In ISR,,,,,,,,,,,,,,,,\n");
    // } else {
    //     // 当前不在中断上下文中
    //     ESP_LOGI(TAG, "Not in ISR,,,,,,,,,,,,,,,\n");
    // }

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
        if (xQueueReceive(uart_queue, &packet, portMAX_DELAY)) {
            // ESP_LOGI(TAG, "uart_tx_task=%.*s", packet.length, (const char *)packet.data);
            // 发送数据到串口
            uart_write_bytes(UART_NUM_2, (const char *)packet.data, packet.length);
            // 发送完成后释放内存
            free(packet.data);
        }
    }
}


static void rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    if(NULL == data){
         ESP_LOGI(TAG, "rx_task malloc NULL.");
    }
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(TAG, "Read %d bytes: '%s'", rxBytes, data);
            
            if(strlen("\r\nIM_READY") <= rxBytes && 0 == memcmp("\r\nIM_READY", data, strlen("\r\nIM_READY"))){
                m_bt_state = BT_READY;
                //发送连接指令
                char *at_conn_dev = "AT+CESL=85AC00093535\r";
                uart_send_data((const uint8_t *)at_conn_dev, strlen(at_conn_dev));

            }else if(strlen("\r\nIM_CONN") <= rxBytes && 0 == memcmp("\r\nIM_CONN", data, strlen("\r\nIM_CONN"))){
                m_bt_state = BT_CONN;
            }else if(strlen("\r\nIM_DISC") <= rxBytes && 0 == memcmp("\r\nIM_DISC", data, strlen("\r\nIM_DISC"))){
                m_bt_state = BT_DISCONN;
            }
        }
    }
    free(data);
}

void app_uart_start(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    
    // 创建 UART 发送任务
    xTaskCreate(uart_tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES-2, NULL);

    // 示例发送数据
    uint8_t example_data[] = {0x48, 0x65, 0x6C, 0x6C, 0x6F}; // "Hello" in ASCII
    uart_send_data(example_data, sizeof(example_data));
}
