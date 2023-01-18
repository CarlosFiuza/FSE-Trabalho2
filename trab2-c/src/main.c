#include <stdio.h>
#include <stdlib.h>

#define pin_res 23
#define pin_cooler 24
#define dev_path_i2c "/dev/i2c-1"
#define dev_path_uart "/dev/serial/0"

#define esp32_add 0x01
#define code_req 0x23
#define code_send 0x16

#define subcode_req_ti 0xC1
#define subcode_req_tr 0xC2
#define subcode_req_comm 0xC3

#define subcode_send_ctrl 0xD1
#define subcode_send_sign_ref 0xD2
#define subcode_send_sys_state 0xD3
#define subcode_send_temp_ctrl 0xD4
#define subcode_send_wrk_state 0xD5
#define subcode_send_ta 0xD6

#define usr_on_oven 0xA1
#define usr_off_oven 0xA2
#define usr_on_warming 0xA3
#define usr_off_warming 0xA4
#define usr_menu 0xA5

//global variables
int system_stt;
int warming_stt;
int temp_mode_stt;


int init_GPIO(pin_gpio1, pin_gpio2) {
  return 0;
}

int init_I2C(device_path) {
  return 0;
}

int init_UART(device_path) {
  return 0;
}

int uart_read() {
  return 0;
}

int uart_write() {
  return 0;
}

int uart_read_usr_commands() {
  return 0;
}

int uart_turn_on_oven() {
  //have to turn on led
  return 0;
}

int uart_turn_off_oven() {
  //have to turn off led
  return 0;
}

int uart_turn_on_warming() {
  //have to look oven mode
  //have to turn on led
  //have to init warming
  return 0;
}

int uart_turn_off_warming() {
  //have to turn off led
  //have to stop warming
  return 0;
}

int uart_change_oven_mode() {
  //have to update oven mode
  //have to send command to esp32
  return 0;
}

void init_system_state() {
  system_stt = 0;
  warming_stt = 0;
  temp_mode_stt = 0;

  //send states to esp32
  return 0;
}

void main_loop() {
  int status, command;

  while (1) {
    //read commands from uart
    status = uart_read_usr_commands(&command);
    if (status != 0) {
      fprintf(stderr, "Falha em ler comandos da uart\n");
    }

    switch (command) {
      case usr_on_oven:
        status = uart_turn_on_oven();
        if (status != 0) {
          fprintf(stderr, "Falha em ligar forno\n");
        }
        break;

      case usr_off_oven:
        status = uart_turn_off_oven();
        if (status != 0) {
          fprintf(stderr, "Falha em desligar forno\n");
        }
        break;

      case usr_on_warming:
        status = uart_turn_on_warming();
        if (status != 0) {
          fprintf(stderr, "Falha em iniciar aquecimento\n");
        }
        break;

      case usr_off_warming:
        status = uart_turn_off_warming();
        if (status != 0) {
          fprintf(stderr, "Falha em parar aquecimento\n");
        }
        break;
    
      case usr_menu:
        status = uart_change_oven_mode();
        if (status != 0) {
          fprintf(stderr, "Falha em mudar modo do forno\n");
        }
        break;

      default:
        fprintf(stderr, "Comando não reconhecido\n");
        break;
    }
    
    sleep(1000);
  }
}

int main (int argc, char **argv) {

  int status;

  //init connection with raspberry
  status = init_GPIO(pin_res, pin_cooler);
  if (status != 0) {
    fprintf(stderr, "Falha em iniciar conexão com GPIO\n");
    exit(EXIT_FAILURE);
  }

  //init serial connection I2C with sensor BME280
  status = init_I2C(dev_path_i2c);
  if (status != 0) {
    fprintf(stderr, "Falha em iniciar conexão com I2C\n");
    exit(EXIT_FAILURE);
  }

  //init serial connection UART with ESCP32 in MODBUS protocol
  status = init_UART(dev_path_uart);
  if (status != 0) {
    fprintf(stderr, "Falha em iniciar conexão com ESP32\n");
    exit(EXIT_FAILURE);
  }

  //init system state
  init_system_state();

  //loop principal
  main_loop();

  //close connections

  return 0;
}