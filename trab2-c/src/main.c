#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <signal.h>

short CRC16(short crc, char data)
{
    const short tbl[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};
    return ((crc & 0xFF00) >> 8) ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)];
}

short calcula_CRC(short crc, unsigned char *commands, int size) {
	int i;
	for(i=0;i<size;i++) {
		crc = CRC16(crc, commands[i]);
	}
	return crc;
}

#define pin_res 23
#define pin_cooler 24
#define dev_path_i2c "/dev/i2c-1"
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

//global variables
int system_stt;
int warming_stt;
int temp_mode_stt;
int uart_fs;


int init_GPIO(int pin_gpio1, int pin_gpio2) {
  return 0;
}

int init_I2C(char *device_path) {
  return 0;
}

int init_UART(char *device_path) {
  uart_fs = open(device_path, O_RDWR | O_NOCTTY | O_NDELAY );
  if (uart_fs == -1) {
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

char verify_sub_code(char sub_code_read) {
  switch (sub_code_read) {
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

int uart_read(unsigned char *buffer) {
  int bytes_read = read(uart_fs, buffer, 3);

  if (bytes_read < 0) {
    fprintf(stderr, "Falha na leitura UART\n");
    return -1;
  }
  else if (bytes_read == 0) {
    fprintf(stderr, "Nenhum dado na leitura UART\n");
    return -2;
  }
  //verifica sub_code
  char sub_code_read = buffer[2];
  printf("sub_code_read %c\n", sub_code_read);

  char sub_code_verified = verify_sub_code(sub_code_read);
  if (sub_code_verified == -1) {
    fprintf(stderr, "Sub-code recebido invalido\n");
    return -3;
  }

  //calcula CRC para conferir dados
  short crc_read;

  unsigned char data[5];
  if (sub_code_verified == subcode_send_ctrl || sub_code_verified == subcode_send_sign_ref) {
    memcpy(&buffer[2], &data, 1);
    memcpy(&buffer[3], &crc_read, 2);
  }
  else {
    bytes_read += read(uart_fs, &buffer[2], 5);
    memcpy(&buffer[2], &data, 5);
    memcpy(&buffer[7], &crc_read, 2);
  }

  short crc = 0;
  crc = CRC16(crc, buffer[0]);
  crc = CRC16(crc, buffer[1]);
  crc = calcula_CRC(crc, data, 5);

  if (crc != crc_read) {
    fprintf(stderr, "Crc invalido\n");
    return -4;
  }

  return 0;
}

int uart_write(char addr, char code, unsigned char* data, int size) {
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

  return 0;
}

int uart_read_usr_commands(int *command) {
  printf("Lendo comandos\n");
  unsigned char buffer[255];
  int status = uart_read(buffer);

  if (status < 0) {
    memset(buffer, 0, 255);
    status = uart_read(buffer);
  }

  if (status < 0) {
    //printf("Falha na leitura de comandos do usr\n");
    return -1;
  }

  memcpy(&buffer[3], command, 4);

  return 0;
}

int uart_turn_on_oven() {
  //have to turn on led
  unsigned char data[6];
  data[0] = subcode_send_sys_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 1;
  int status = uart_write(esp32_add, code_send, data, 6);
  system_stt = 1;

  return 0;
}

int uart_turn_off_oven() {
  //have to turn off led
  unsigned char data[6];
  data[0] = subcode_send_sys_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 0;
  int status = uart_write(esp32_add, code_send, data, 6);
  system_stt = 0;
  
  return 0;
}

int uart_turn_on_warming() {
  //have to look oven mode
  //have to turn on led
  //have to init warming
  unsigned char data[6];
  data[0] = subcode_send_wrk_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 1;
  int status = uart_write(esp32_add, code_send, data, 6);
  warming_stt = 1;
  
  return 0;
}

int uart_turn_off_warming() {
  //have to turn off led
  //have to stop warming
  unsigned char data[6];
  data[0] = subcode_send_wrk_state;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = 0;
  int status = uart_write(esp32_add, code_send, data, 6);
  warming_stt = 0;
  
  return 0;
}

int uart_change_oven_mode() {
  //have to update oven mode
  //have to send command to esp32
  temp_mode_stt = temp_mode_stt == 1 ? 0 : 1;
  unsigned char data[6];
  data[0] = subcode_send_temp_ctrl;
  data[1] = mat_1;
  data[2] = mat_2;
  data[3] = mat_3;
  data[4] = mat_4;
  data[5] = temp_mode_stt;
  int status = uart_write(esp32_add, code_send, data, 6);
  warming_stt = 0;
  return 0;
}

void init_system_state() {
  system_stt = 0;
  warming_stt = 0;
  temp_mode_stt = 1;

  uart_turn_off_oven();
  uart_turn_off_warming();
  uart_change_oven_mode();
}

int terminate_uart() {
  int status = close(uart_fs);

  return status;
}

void main_loop() {
  int status, command;

  while (1) {
    //read commands from uart
    status = uart_read_usr_commands(&command);
    if (status != 0) {
      printf("Falha em ler comandos da uart\n");
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
        //printf("Comando não reconhecido\n");
        break;
    }
    
    sleep(0.5);
  }
}

void terminate_prog(int d) {
  printf("\nCtrl-c pressed\n");
  fflush(stdout);

  terminate_uart();

  exit(1);
}

int main (int argc, char **argv) {
  signal(SIGINT, terminate_prog);

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
  terminate_uart();

  return 0;
}