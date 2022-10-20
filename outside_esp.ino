#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define ECHO_TEST_TXD 17 //(CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD 16 //(CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS 10 //(UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS 11 //(UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM 2     //(CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE 115200    //(CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE 0   //(CONFIG_EXAMPLE_TASK_STACK_SIZE)

static const char *TAG = "UART TEST";
int Enable_pin = 4;
#define BUF_SIZE (1024)
uart_config_t uart_config = {
  .baud_rate = ECHO_UART_BAUD_RATE,
  .data_bits = UART_DATA_8_BITS,
  .parity    = UART_PARITY_DISABLE,
  .stop_bits = UART_STOP_BITS_1,
  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,

};

void setup() {
  // put your setup code here, to run once:
  int intr_alloc_flags = 0;
  pinMode(Enable_pin, OUTPUT);
  digitalWrite(Enable_pin, LOW);        //  (LOW to receive value from Master)
  ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, intr_alloc_flags));
  //ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
  //uart_write_bytes(UART_NUM_0, (const char*)"Driver installed", strlen("Driver installed"));
}

void loop() {

  uint8_t data[128];
  int length = 0;

  ESP_ERROR_CHECK(uart_get_buffered_data_len(ECHO_UART_PORT_NUM, (size_t*)&length));
  length = uart_read_bytes(ECHO_UART_PORT_NUM, data, length, 100);

  //  char len1 = length + '0';
  //  uart_write_bytes(UART_NUM_0, (const char*)"Buffered Data length: ", strlen("Buffered Data length: "));
  //  uart_write_bytes(UART_NUM_0, &len1 , 1);
  //  uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
  //  if (length != 0) {
  //
  //    char len2 = length + '0';
  //    uart_write_bytes(UART_NUM_0, (const char*)"length read: ", strlen("length read: "));
  //    uart_write_bytes(UART_NUM_0, &len2 , 1);
  //    uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
  //    uart_flush(ECHO_UART_PORT_NUM);
  //    for (int j = 0; j < 3; j++) {
  //      char j1 = j + '0';
  //      uart_write_bytes(UART_NUM_0, (const char*)"j: ", strlen("j: "));
  //      uart_write_bytes(UART_NUM_0, &j1 , 1);
  //      uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
  //
  //      int k = 0;
  //      //m[0]= (const char *)data;
  //      for (int i = 3; i >= 0; i--) {
  //        char i1 = i + '0';
  //        uart_write_bytes(UART_NUM_0, (const char*)"i: ", strlen("i: "));
  //        uart_write_bytes(UART_NUM_0, &i1 , 1);
  //        uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
  //
  //        k = (k << 8) | data[i + (j * 4)];
  //      }
  //      float* m = (float*)&k;
  //      uart_write_bytes(UART_NUM_0, (const char*)"float value: ", strlen("float value: "));
  //      uart_write_bytes(UART_NUM_0, (char*)m, sizeof(float));
  //      uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
  //
  //      //float m1 = float
  //      if (*m == 53.34) {
  //        uart_write_bytes(UART_NUM_0, (const char*)"correct", strlen("correct"));
  //        uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
  //      }
  //      else {
  //        uart_write_bytes(UART_NUM_0, (const char*)"wrong", strlen("wrong"));
  //        uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
  //      }
  //
  //    }


//converting to string and reading again - worked properly
String str = String((const char *)data);
uart_write_bytes(UART_NUM_0, str.c_str(), length);


// Write data back to the UART

//uart_write_bytes(UART_NUM_0, (const char *) data, length);
delay(100);

}
