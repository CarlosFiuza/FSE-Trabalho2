#ifndef LOGGER_H_
#define LOGGER_H_

int log_init_file();
int log_write(float ti, float ta, float tr, int sign_res, int sign_cool);
void log_close();

#endif