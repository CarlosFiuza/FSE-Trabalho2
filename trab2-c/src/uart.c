#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "crc16.h"
#include "uart.h"
#include <time.h>

#define dev_path_uart "/dev/serial0"

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

#define mat_1 6
#define mat_2 8
#define mat_3 4
#define mat_4 3

int uart_fs;

int init_UART()
{
  uart_fs = open(dev_path_uart, O_RDWR | O_NOCTTY | O_NDELAY);
  if (uart_fs == -1)
  {
    return uart_fs;
  }

  struct termios options;

  tcgetattr(uart_fs, &options);
  options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(uart_fs, TCIFLUSH);
  tcsetattr(uart_fs, TCSANOW, &options);

  return 0;
}

void sleep_to_read()
{
  sleep(0.25);
}

int uart_read(unsigned char *buffer, char sub_code)
{
  int bytes_read = read(uart_fs, &buffer[0], 3);

  if (bytes_read < 0)
  {
    fprintf(stderr, "Falha na leitura UART\n");
    return -1;
  }
  else if (bytes_read == 0)
  {
    // fprintf(stderr, "Nenhum dado na leitura UART\n");
    return -2;
  }
  // verifica sub_code
  char sub_code_read = buffer[2];

  if (sub_code != '0' && sub_code_read != sub_code)
  {
    // fprintf(stderr, "Sub-code recebido invalido\n");
    return -3;
  }

  // calcula CRC para conferir dados
  short crc_read;

  if (sub_code_read == subcode_send_ctrl || sub_code_read == subcode_send_sign_ref)
  {
    return 0;
  }

  bytes_read += read(uart_fs, &buffer[3], 6);
  memcpy(&crc_read, &buffer[7], 2);

  short crc = 0;
  crc = CRC16(crc, buffer[0]);
  crc = CRC16(crc, buffer[1]);
  crc = calcula_CRC(crc, &buffer[2], 5);

  if (crc != crc_read)
  {
    fprintf(stderr, "Crc invalido\n");
    tcflush(uart_fs, TCIFLUSH);
    return -4;
  }

  return 0;
}

int uart_write(char addr, char code, unsigned char *data, int size)
{
  short crc = 0;
  int bytes_sended = 0;

  crc = CRC16(crc, addr);
  crc = CRC16(crc, code);
  crc = calcula_CRC(crc, data, size);

  bytes_sended += write(uart_fs, &addr, 1) +
                  write(uart_fs, &code, 1) +
                  write(uart_fs, data, size) +
                  write(uart_fs, &crc, 2);

  return bytes_sended;
}

int uart_read_usr_commands(int *command)
{
  // printf("Lendo comandos\n");
  unsigned char data[5];
  data[0] = subcode_req_comm;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  int bytes = uart_write(esp32_add, code_req, data, 5);

  if (bytes != 9)
  {
    fprintf(stderr, "Falha ao requisitar comandos usuario\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, '0');

  if (status < 0)
  {
    memset(buffer, 0, 255);
    status = uart_read(buffer, '0');
  }

  if (status < 0)
  {
    // printf("Falha na leitura de comandos do usr\n");
    return -1;
  }

  memcpy(command, &buffer[3], 4);

  return 0;
}

int uart_turn_on_oven(int *system_stt)
{
  // have to turn on led
  unsigned char data[6];
  int state = 1;
  data[0] = subcode_send_sys_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 1;
  int bytes = uart_write(esp32_add, code_send, data, 6);

  if (bytes != 10)
  {
    fprintf(stderr, "Falha ao ligar sistema\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_sys_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);
  printf("state_read uart_turn_on_oven: %d\n", state_read);

  *system_stt = 1;

  if (status != 0 || state_read != state)
  {
    // fprintf(stderr, "Falha ao ligar sistema\n");
    return -1;
  }

  return 0;
}

int uart_turn_off_oven(int *system_stt)
{
  // have to turn off led
  unsigned char data[6];
  int state = 0;
  data[0] = subcode_send_sys_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 0;

  int bytes = uart_write(esp32_add, code_send, data, 6);

  if (bytes != 10)
  {
    fprintf(stderr, "Falha ao desligar sistema\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_sys_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);

  *system_stt = 0;

  if (status != 0 || state_read != state)
  {
    //fprintf(stderr, "Falha ao desligar sistema\n");
    return -1;
  }

  return 0;
}

int uart_turn_on_warming(int *warming_stt, int temp_mode_stt, clock_t *time_curve_mode)
{
  // have to look oven mode
  // have to turn on led
  // have to init warming
  unsigned char data[6];
  int state = 1;
  data[0] = subcode_send_wrk_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 1;

  int bytes = uart_write(esp32_add, code_send, data, 6);

  if (bytes != 10)
  {
    fprintf(stderr, "Falha ao ligar aquecimento\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_wrk_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);

  *warming_stt = 1;

  if (temp_mode_stt == 1)
  {
    *time_curve_mode = clock();
  }

  if (status != 0 || state_read != state)
  {
    // fprintf(stderr, "Falha ao ligar aquecimento\n");
    return -1;
  }

  return 0;
}

int uart_turn_off_warming(int *warming_stt)
{
  // have to turn off led
  // have to stop warming
  unsigned char data[6];
  int state = 0;
  data[0] = subcode_send_wrk_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 0;
  int bytes = uart_write(esp32_add, code_send, data, 6);

  if (bytes != 10)
  {
    // fprintf(stderr, "Falha ao desligar aquecimento\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_wrk_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);

  *warming_stt = 0;
  
  if (status != 0 || state_read != state)
  {
    // fprintf(stderr, "Falha ao desligar aquecimento\n");
    return -1;
  }

  return 0;
}

int uart_change_oven_mode(int *temp_mode_stt)
{
  // have to update oven mode
  // have to send command to esp32

  if (*temp_mode_stt == 1)
  {
    *temp_mode_stt = 0;
  }
  else
  {
    *temp_mode_stt = 1;
  }

  unsigned char data[6];
  data[0] = subcode_send_temp_ctrl;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = *temp_mode_stt;
  int bytes = uart_write(esp32_add, code_send, data, 6);

  if (bytes != 10)
  {
    // fprintf(stderr, "Falha ao mudar modo forno\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_wrk_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);

  if (status != 0 || state_read != *temp_mode_stt)
  {
    // fprintf(stderr, "Falha ao mudar modo forno\n");
    return -1;
  }

  return 0;
}

int uart_send_ta_temp(float ta)
{
  // have to send temp ta
  unsigned char data[9];
  data[0] = subcode_send_ta;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  memcpy(&data[5], &ta, 4);
  int bytes = uart_write(esp32_add, code_send, data, 9);

  if (bytes != 13)
  {
    fprintf(stderr, "Falha ao enviar temp ta\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_ta);

  float temp_read;
  memcpy(&temp_read, &buffer[3], 4);

  if (status != 0 || temp_read != ta)
  {
    // fprintf(stderr, "Falha ao enviar temp ta, enviado: %f; recebido: %f\n", ta, temp_read);
    return -1;
  }

  return 0;
}

int uart_read_ti(float *ti)
{
  // have to request temp ti
  unsigned char data[5];
  data[0] = subcode_req_ti;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  int bytes = uart_write(esp32_add, code_send, data, 5);

  if (bytes != 9)
  {
    fprintf(stderr, "Falha ao requisitar temp ti\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_req_ti);

  float temp_read;
  memcpy(&temp_read, &buffer[3], 4);

  if (status != 0)
  {
    // fprintf(stderr, "Falha ao requisitar temp ti\n");
    return -1;
  }

  *ti = temp_read;

  return 0;
}

int uart_read_tr(float *tr)
{
  // have to request temp tr
  unsigned char data[5];
  data[0] = subcode_req_tr;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  int bytes = uart_write(esp32_add, code_send, data, 5);

  if (bytes != 9)
  {
    fprintf(stderr, "Falha ao requisitar temp tr\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_req_tr);

  float temp_read;
  memcpy(&temp_read, &buffer[3], 4);

  if (status != 0)
  {
    // fprintf(stderr, "Falha ao requisitar temp tr\n");
    return -1;
  }

  *tr = temp_read;

  return 0;
}

int uart_send_ctrl_signal(int signal)
{
  // have to send control signal
  unsigned char data[9];
  data[0] = subcode_send_ctrl;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  memcpy(&data[5], &signal, 4);
  int bytes = uart_write(esp32_add, code_send, data, 9);

  if (bytes != 13)
  {
    fprintf(stderr, "Falha ao enviar sinal de controle\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_ctrl);

  if (status != 0)
  {
    // fprintf(stderr, "Falha ao enviar sinal de controle\n");
    return -1;
  }

  return 0;
}

int uart_send_ref_signal(float signal)
{
  // have to send control signal
  unsigned char data[9];
  data[0] = subcode_send_sign_ref;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  memcpy(&data[5], &signal, 4);
  int bytes = uart_write(esp32_add, code_send, data, 9);

  if (bytes != 13)
  {
    fprintf(stderr, "Falha ao enviar sinal de referencia\n");
    return -1;
  }

  sleep_to_read();

  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_sign_ref);

  if (status != 0)
  {
    fprintf(stderr, "Falha ao enviar sinal de referencia\n");
    return -1;
  }

  return 0;
}

void uart_init_system_state(int *system_stt, int *warming_stt, int *temp_mode_stt)
{
  *system_stt = 0;
  *warming_stt = 0;
  *temp_mode_stt = 1;

  uart_turn_off_oven(system_stt);
  uart_turn_off_warming(warming_stt);
  uart_change_oven_mode(temp_mode_stt);

  *temp_mode_stt = 0;
}

int uart_terminate(int *system_stt, int *warming_stt, int *temp_mode_stt)
{
  uart_init_system_state(system_stt, warming_stt, temp_mode_stt);
  int status = close(uart_fs);

  return status;
}
