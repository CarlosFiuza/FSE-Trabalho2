#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "logger.h"

FILE *log_file;

int log_init_file() {
  log_file = fopen("log.csv", "w");
  if (log_file == NULL) {
    return -1;
  }
  fprintf(log_file, "timestamp, temp int, temp ext, temp usr, perc resistor, perc cooler\n");

  return 0;
}

int log_write(float ti, float ta, float tr, int sign_res, int sign_cool) {
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  int writed = fprintf(log_file, "%d-%02d-%02d %02d:%02d:%02d, %2.f, %2.f, %2.f, %d, %d\n",
                        tm.tm_mday, tm.tm_mon + 1,tm.tm_year + 1900, tm.tm_hour, tm.tm_min, tm.tm_sec,
                        ti,
                        ta,
                        tr,
                        sign_res,
                        sign_cool);
  fflush(log_file);
  if (writed < 1) {
    fprintf(stderr, "Falha no registro do logger\n");
    return -1;
  }

  return 0;
}

void log_close() {
  fclose(log_file);
}