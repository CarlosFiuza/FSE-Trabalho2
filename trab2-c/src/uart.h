#ifndef UART_H_
#define UART_H_
#include <sys/time.h>

int uart_init();
int uart_read_usr_commands(int *command);
int uart_turn_on_oven(int *system_stt);
int uart_turn_off_oven(int *system_stt, int *warming_stt);
int uart_turn_on_warming(int *warming_stt, int temp_mode_stt, int system_stt, struct timeval *time_curve_mode);
int uart_turn_off_warming(int *warming_stt);
int uart_change_oven_mode(int *temp_mode_stt);
int uart_send_ta_temp(float ta);
int uart_read_ti(float *ti);
int uart_read_tr(float *tr);
int uart_send_ctrl_signal(int signal);
int uart_send_ref_signal(float signal);
void uart_init_system_state(int *system_stt, int *warming_stt, int *temp_mode_stt);
int uart_terminate(int *system_stt, int *warming_stt, int *temp_mode_stt);

#endif