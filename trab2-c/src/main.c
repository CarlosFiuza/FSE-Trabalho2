#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include "pid.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <time.h>
#include "uart.h"
#include "logger.h"
#include "i2c.h"

#define pin_res 4
#define pin_cooler 5

#define usr_on_oven 0xA1
#define usr_off_oven 0xA2
#define usr_on_warming 0xA3
#define usr_off_warming 0xA4
#define usr_menu 0xA5

// global variables
int system_stt;
int warming_stt;
int temp_mode_stt;

clock_t time_curve_mode;
float last_tr = 30.0f;
float last_ti = 25.0f;
int sig_value_res = 0;
int sig_value_cooler = 0;

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
    sig_value_res = signal;
    sig_value_cooler = 0;
  }
  else if (signal < 0)
  {
    softPwmWrite(pin_cooler, signal);
    softPwmWrite(pin_res, 0);
    sig_value_res = 0;
    sig_value_cooler = signal;
  }
  else
  {
    softPwmWrite(pin_cooler, 0);
    softPwmWrite(pin_res, 0);
    sig_value_res = 0;
    sig_value_cooler = 0;
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
  else {
    sig_value_res = 0;
    sig_value_cooler = 0;
  }

  log_write(ti, ta, tr, sig_value_res, sig_value_cooler);

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
      //printf("Comando não reconhecido\n");
      break;
    }

    // to read usr comm in 500ms and control temp in 1s
    if (count_sleep == 2)
    {
      temperature_control();
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

  log_close();

  exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGINT, terminate_prog);

  // init connection with raspberry
  if (init_GPIO(pin_res, pin_cooler) != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com GPIO\n");
    exit(EXIT_FAILURE);
  }

  // init serial connection I2C with sensor BME280
  if (i2c_init() != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com I2C\n");
    exit(EXIT_FAILURE);
  }

  // init serial connection UART with ESCP32 in MODBUS protocol
  if (uart_init() != 0)
  {
    fprintf(stderr, "Falha em iniciar conexão com ESP32\n");
    exit(EXIT_FAILURE);
  }

  if (log_init_file() != 0)
  {
    fprintf(stderr, "Falha em abrir arquivo de log\n");
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