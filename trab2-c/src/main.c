#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <signal.h>
#include "bme280.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include "crc16.h"

#define pin_res 23
#define pin_cooler 24
#define dev_path_i2c "/dev/i2c-1"
#define i2c_add 0x76
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

#define usr_on_oven 0xA1
#define usr_off_oven 0xA2
#define usr_on_warming 0xA3
#define usr_off_warming 0xA4
#define usr_menu 0xA5

// global variables
int system_stt;
int warming_stt;
int temp_mode_stt;
int uart_fs;
struct bme280_dev *dev1;
int8_t rslt = BME280_OK;
struct bme280_data comp_data;
uint32_t req_delay;

int init_GPIO(int pin_gpio1, int pin_gpio2)
{
  return 0;
}

struct identifier
{
  /* Variable to hold device address */
  uint8_t dev_addr;

  /* Variable that contains file descriptor */
  int8_t fd;
};

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
  struct identifier id;

  id = *((struct identifier *)intf_ptr);

  write(id.fd, &reg_addr, 1);
  read(id.fd, data, len);

  return 0;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
  uint8_t *buf;
  struct identifier id;

  id = *((struct identifier *)intf_ptr);

  buf = malloc(len + 1);
  buf[0] = reg_addr;
  memcpy(buf + 1, data, len);
  if (write(id.fd, buf, len + 1) < (uint16_t)len)
  {
    return BME280_E_COMM_FAIL;
  }

  free(buf);

  return BME280_OK;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
  usleep(period);
}

int8_t set_stream_sensor_data_forced_mode()
{
    /* Variable to define the result */
    rslt = BME280_OK;

    /* Variable to define the selecting sensors */
    uint8_t settings_sel = 0;

    /* Variable to store minimum wait time between consecutive measurement in force mode */

    /* Recommended mode of operation: Indoor navigation */
    dev1->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev1->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev1->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev1->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    /* Set the sensor settings */
    rslt = bme280_set_sensor_settings(settings_sel, dev1);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);

        return rslt;
    }

    printf("Temperature, Pressure, Humidity\n");

    /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&dev1->settings);

    return rslt;
}

int init_I2C(char *device_path)
{
  struct bme280_dev dev;
  struct identifier id;

  int8_t rslt = BME280_OK;

  if ((id.fd = open(dev_path_i2c, O_RDWR)) < 0)
  {
    fprintf(stderr, "Falha em i2c bus\n");
    return -1;
  }

  if (ioctl(id.fd, I2C_SLAVE, i2c_add) < 0)
  {
    fprintf(stderr, "Falha endereco i2c\n");
    return -2;
  }

  id.dev_addr = BME280_I2C_ADDR_PRIM;

  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_us = user_delay_us;

  dev.intf_ptr = &id;

  rslt = bme280_init(&dev);
  if (rslt != BME280_OK)
  {
    fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
    exit(1);
  }

  rslt = set_stream_sensor_data_forced_mode(&dev);
  if (rslt != BME280_OK)
  {
    fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
    exit(1);
  }
  return 0;
}

int init_UART(char *device_path)
{
  uart_fs = open(device_path, O_RDWR | O_NOCTTY | O_NDELAY);
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

char verify_sub_code(char sub_code_read)
{
  switch (sub_code_read)
  {
  case subcode_req_ti:
    return subcode_req_ti;
  case subcode_req_tr:
    return subcode_req_tr;
  case subcode_req_comm:
    return subcode_req_comm;
  case subcode_send_ctrl:
    return subcode_send_ctrl;
  case subcode_send_wrk_state:
    return subcode_send_wrk_state;
  case subcode_send_ta:
    return subcode_send_ta;
  default:
    return -1;
  }
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
    fprintf(stderr, "Sub-code recebido invalido\n");
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

  printf("Bytes enviados: %d\n", bytes_sended);

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

  sleep(0.03);

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

int uart_turn_on_oven()
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

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_sys_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);
  printf("state_read uart_turn_on_oven: %d\n", state_read);

  if (status != 0 || state_read != state)
  {
    fprintf(stderr, "Falha ao ligar sistema\n");
    return -1;
  }
  system_stt = 1;

  return 0;
}

int uart_turn_off_oven()
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

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_sys_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);
  printf("state_read uart_turn_off_oven: %d\n", state_read);

  if (status != 0 || state_read != state)
  {
    fprintf(stderr, "Falha ao desligar sistema\n");
    return -1;
  }

  system_stt = 0;
  return 0;
}

int uart_turn_on_warming()
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

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_wrk_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);
  printf("state_read uart_turn_on_warming: %d\n", state_read);

  if (status != 0 || state_read != state)
  {
    fprintf(stderr, "Falha ao ligar aquecimento\n");
    return -1;
  }

  warming_stt = 1;

  return 0;
}

int uart_turn_off_warming()
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
    fprintf(stderr, "Falha ao ligar aquecimento\n");
    return -1;
  }

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_wrk_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);
  printf("state_read uart_turn_off_warming: %d\n", state_read);

  if (status != 0 || state_read != state)
  {
    fprintf(stderr, "Falha ao ligar aquecimento\n");
    return -1;
  }
  warming_stt = 0;

  return 0;
}

int uart_change_oven_mode()
{
  // have to update oven mode
  // have to send command to esp32
  int aux = temp_mode_stt == 1 ? 0 : 1;
  unsigned char data[6];
  data[0] = subcode_send_temp_ctrl;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = temp_mode_stt;
  int bytes = uart_write(esp32_add, code_send, data, 6);

  if (bytes != 10)
  {
    fprintf(stderr, "Falha ao ligar aquecimento\n");
    return -1;
  }

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_wrk_state);

  int state_read;
  memcpy(&state_read, &buffer[3], 1);

  if (status != 0 || state_read != aux)
  {
    fprintf(stderr, "Falha ao ligar aquecimento\n");
    return -1;
  }

  temp_mode_stt = aux;

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

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_send_ta);

  float temp_read;
  memcpy(&temp_read, &buffer[3], 4);

  if (status != 0 || temp_read != ta)
  {
    fprintf(stderr, "Falha ao enviar temp ta\n");
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

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_req_ti);

  float temp_read;
  memcpy(&temp_read, &buffer[3], 4);

  if (status != 0)
  {
    fprintf(stderr, "Falha ao requisitar temp ti\n");
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

  sleep(0.03);
  unsigned char buffer[10];
  int status = uart_read(buffer, subcode_req_tr);

  float temp_read;
  memcpy(&temp_read, &buffer[3], 4);

  if (status != 0)
  {
    fprintf(stderr, "Falha ao requisitar temp tr\n");
    return -1;
  }

  *tr = temp_read;

  return 0;
}

int i2c_read_ta(float *ta)
{
  *ta = 24.2f;
  return 0;
  

  // int i = 0;
  // while (i < 4)
  // {
  //   /* Set the sensor to forced mode */
  //   rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev1);
  //   if (rslt != BME280_OK)
  //   {
  //       fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
  //       break;
  //   }

  //   /* Wait for the measurement to complete and print data */
  //   dev1->delay_us(req_delay, dev1->intf_ptr);
  //   rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev1);
  //   if (rslt != BME280_OK)
  //   {
  //       fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
  //       break;
  //   }

  //   float temp, press, hum;

  //   temp = comp_data.temperature;
  //   press = 0.01 * comp_data.pressure;
  //   hum = comp_data.humidity;  

  //   printf("temp: %2.f; press: %2.f; hum: %2.f\n", temp, press, hum);
  // }

  // return 0;
}

int temperature_control()
{
  float ta, ti, tr;
  int status;

  status = i2c_read_ta(&ta);
  if (status != 0) {
    ta = 24.2f;
  } else {
    printf("TA: %f\n", ta);
  }
  
  status = uart_send_ta_temp(ta);

  status = uart_read_ti(&ti);
  if (status != 0) {
    ti = 30.0f;
  } else {
    printf("TI: %f\n", ti);
  }

  status = uart_read_tr(&tr);
  if (status != 0) {
    tr = 30.0f;
  } else {
    printf("TR: %f\n", tr);
  }

  fflush(stdout);

  return 0;
}

void init_system_state()
{
  system_stt = 0;
  warming_stt = 0;
  temp_mode_stt = 1;

  uart_turn_off_oven();
  uart_turn_off_warming();
  uart_change_oven_mode();
}

int terminate_uart()
{
  int status = close(uart_fs);

  return status;
}

void main_loop()
{
  int status, command;

  while (1)
  {
    // read commands from uart
    status = uart_read_usr_commands(&command);
    if (status != 0)
    {
      printf("Falha em ler comandos da uart\n");
    }

    switch (command)
    {
    case usr_on_oven:
      status = uart_turn_on_oven();
      if (status != 0)
      {
        fprintf(stderr, "Falha em ligar forno\n");
      }
      break;

    case usr_off_oven:
      status = uart_turn_off_oven();
      if (status != 0)
      {
        fprintf(stderr, "Falha em desligar forno\n");
      }
      break;

    case usr_on_warming:
      status = uart_turn_on_warming();
      if (status != 0)
      {
        fprintf(stderr, "Falha em iniciar aquecimento\n");
      }
      break;

    case usr_off_warming:
      status = uart_turn_off_warming();
      if (status != 0)
      {
        fprintf(stderr, "Falha em parar aquecimento\n");
      }
      break;

    case usr_menu:
      status = uart_change_oven_mode();
      if (status != 0)
      {
        fprintf(stderr, "Falha em mudar modo do forno\n");
      }
      break;

    default:
      // printf("Comando não reconhecido\n");
      break;
    }

    temperature_control();

    sleep(3);
  }
}

void terminate_prog(int d)
{
  printf("\nCtrl-c pressed\n");
  fflush(stdout);

  terminate_uart();

  exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGINT, terminate_prog);

  int status;

  // init connection with raspberry
  status = init_GPIO(pin_res, pin_cooler);
  if (status != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com GPIO\n");
    exit(EXIT_FAILURE);
  }

  // init serial connection I2C with sensor BME280
  status = 0;//init_I2C(dev_path_i2c);
  if (status != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com I2C\n");
    exit(EXIT_FAILURE);
  }

  // init serial connection UART with ESCP32 in MODBUS protocol
  status = init_UART(dev_path_uart);
  if (status != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com ESP32\n");
    exit(EXIT_FAILURE);
  }

  // init system state
  init_system_state();

  // loop principal
  main_loop();

  // close connections
  terminate_uart();

  return 0;
}