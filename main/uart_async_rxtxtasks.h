/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_system.h"


typedef enum {
    BT_IDLE = 0,
    BT_READY,
    BT_DISCONN,
    BT_CONNING,
    BT_CONN,
    BT_CONN_WRITE_END,
} bt_state_t;

// 定义一个数据结构来保存要发送的数据
typedef struct {
    uint8_t *data;
    size_t length;
    char sn[32];         //str
    char cmd[32];        //str
    uint8_t retry;
} uart_packet_t;


extern bt_state_t m_bt_state;

void uart_push_queue(const char *pcmd, const char *psn, const uint8_t *data, size_t length, uint8_t retry);
extern void app_uart_start(void);
