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
#include "pid.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <time.h>
#include "uart.h"

#define pin_res 4
#define pin_cooler 5
#define dev_path_i2c "/dev/i2c-1"
#define i2c_add 0x76

#define usr_on_oven 0xA1
#define usr_off_oven 0xA2
#define usr_on_warming 0xA3
#define usr_off_warming 0xA4
#define usr_menu 0xA5

// global variables
int system_stt;
int warming_stt;
int temp_mode_stt;
struct bme280_dev *dev1;
int8_t rslt = BME280_OK;
struct bme280_data comp_data;
uint32_t req_delay;
clock_t time_curve_mode;
float last_tr = 30.0f;
float last_ti = 25.0f;

int init_GPIO(int pin_gpio1, int pin_gpio2)
{
  if (wiringPiSetup() == -1)
  {
    fprintf(stderr, "Falha em configurar wiringPi\n");
    return -1;
  }

  pinMode(pin_gpio1, OUTPUT);
  pinMode(pin_gpio2, OUTPUT);

  softPwmCreate(pin_gpio1, 0, 100);
  softPwmCreate(pin_gpio2, 0, 100);

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

float get_tr_by_curve(double seconds)
{
  if (seconds < 60.0)
  {
    return 25.0f;
  }
  else if (seconds < 120)
  {
    return 38.0f;
  }
  else if (seconds < 240)
  {
    return 46.0f;
  }
  else if (seconds < 260)
  {
    return 54.0f;
  }
  else if (seconds < 300)
  {
    return 57.0f;
  }
  else if (seconds < 360)
  {
    return 61.0f;
  }
  else if (seconds < 420)
  {
    return 63.0f;
  }
  else if (seconds < 480)
  {
    return 54.0f;
  }
  else if (seconds < 600)
  {
    return 33.0f;
  }
  else
  {
    return 25.0f;
  }
}

int gpio_update_pwm(int signal)
{
  if (signal > 0)
  {
    softPwmWrite(pin_res, signal);
    softPwmWrite(pin_cooler, 0);
  }
  else if (signal < 0)
  {
    softPwmWrite(pin_cooler, signal);
    softPwmWrite(pin_res, 0);
  }
  else
  {
    softPwmWrite(pin_cooler, 0);
    softPwmWrite(pin_res, 0);
  }

  return 0;
}

int temperature_control()
{
  float ta, ti, tr;
  int status;

  status = i2c_read_ta(&ta);
  if (status != 0)
  {
    ta = 24.2f;
  }

  status = uart_send_ta_temp(ta);

  status = uart_read_ti(&ti);
  if (status != 0)
  {
    status = uart_read_ti(&ti);
    if (status != 0)
    {
      ti = last_ti;
    }
  }
  if (status == 0)
  {
    last_ti = ti;
  }

  if (temp_mode_stt == 0)
  {
    status = uart_read_tr(&tr);
    if (status != 0)
    {
      status = uart_read_tr(&tr);
      if (status != 0)
      {
        tr = last_tr;
      }
    }
    if (status == 0)
    {
      last_tr = tr;
    }
  }
  else if (warming_stt == 1 && temp_mode_stt == 1)
  {
    // catch reference temperatura from curve
    // send temperature to uart
    clock_t time_now = clock();
    double diff_time = ((double)time_now - time_curve_mode) / CLOCKS_PER_SEC;
    tr = get_tr_by_curve(diff_time);

    status = uart_send_ref_signal(tr);
  }

  if (warming_stt == 1)
  {
    pid_atualiza_referencia(tr);

    double signal_ctrl;
    signal_ctrl = pid_controle((double)ti);

    status = uart_send_ctrl_signal((int)signal_ctrl);

    status = gpio_update_pwm((int)signal_ctrl);
  }

  fflush(stdout);

  return 0;
}

void main_loop()
{
  int status, command, count_sleep = 0;

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
      status = uart_turn_on_oven(&system_stt);
      if (status != 0)
      {
        fprintf(stderr, "Falha em ligar forno\n");
      }
      break;

    case usr_off_oven:
      status = uart_turn_off_oven(&system_stt);
      if (status != 0)
      {
        fprintf(stderr, "Falha em desligar forno\n");
      }
      break;

    case usr_on_warming:
      status = uart_turn_on_warming(&warming_stt, temp_mode_stt, &time_curve_mode);
      if (status != 0)
      {
        fprintf(stderr, "Falha em iniciar aquecimento\n");
      }
      break;

    case usr_off_warming:
      status = uart_turn_off_warming(&warming_stt);
      if (status != 0)
      {
        fprintf(stderr, "Falha em parar aquecimento\n");
      }
      break;

    case usr_menu:
      status = uart_change_oven_mode(&temp_mode_stt);
      if (status != 0)
      {
        fprintf(stderr, "Falha em mudar modo do forno\n");
      }
      break;

    default:
      // printf("Comando não reconhecido\n");
      break;
    }

    // to read usr comm in 500ms and control temp in 1s
    if (count_sleep == 2)
    {
      printf("1 sys %d warm %d mode %d\n", system_stt, warming_stt, temp_mode_stt);
      temperature_control();
      printf("2 sys %d warm %d mode %d\n", system_stt, warming_stt, temp_mode_stt);
      count_sleep = 0;
    }

    count_sleep++;

    sleep(1);
  }
}

void terminate_prog(int d)
{
  printf("\nCtrl-c pressed\n");
  fflush(stdout);

  uart_terminate(&system_stt, &warming_stt, &temp_mode_stt);

  gpio_update_pwm(0);

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
  status = 0; // init_I2C(dev_path_i2c);
  if (status != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com I2C\n");
    exit(EXIT_FAILURE);
  }

  // init serial connection UART with ESCP32 in MODBUS protocol
  status = init_UART();
  if (status != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com ESP32\n");
    exit(EXIT_FAILURE);
  }

  // configure parameters pid
  pid_configura_constantes(30.0, 0.2, 400.0);

  // init system state
  uart_init_system_state(&system_stt, &warming_stt, &temp_mode_stt);

  // loop principal
  main_loop();

  return 0;
}