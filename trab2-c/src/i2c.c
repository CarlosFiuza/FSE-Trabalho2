
#include "bme280.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "i2c.h"

#define dev_path_i2c "/dev/i2c-1"
#define i2c_add 0x76

struct bme280_dev dev;
int8_t rslt = BME280_OK;
struct bme280_data comp_data;
uint32_t req_delay;

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
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  /* Set the sensor settings */
  rslt = bme280_set_sensor_settings(settings_sel, &dev);
  if (rslt != BME280_OK)
  {
    fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);

    return rslt;
  }

  /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
   *  and the oversampling configuration. */
  req_delay = bme280_cal_meas_delay(&dev.settings);

  return rslt;
}

int i2c_init()
{
  struct identifier id;

  int8_t rslt = BME280_OK;

  if ((id.fd = open("/dev/i2c-1", O_RDWR)) < 0)
  {
    fprintf(stderr, "Falha em i2c bus\n");
    return -1;
  }

  id.dev_addr = BME280_I2C_ADDR_PRIM;

  if (ioctl(id.fd, I2C_SLAVE, id.dev_addr) < 0)
  {
    fprintf(stderr, "Falha endereco i2c\n");
    return -2;
  }

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

  return 0;
}

int i2c_read_ta(float *ta)
{
  set_stream_sensor_data_forced_mode();

  int i = 0;

  while (i < 4)
  {
    /* Set the sensor to forced mode */
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);

    if (rslt != BME280_OK)
    {
      fprintf(stderr, "Failed to set sensor mode (code %+d).\n", rslt);
      i++;
      continue;
    }

    /* Wait for the measurement to complete and print data */
    dev.delay_us(req_delay, dev.intf_ptr);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    if (rslt != BME280_OK)
    {
      fprintf(stderr, "Failed to get sensor data (code %+d).\n", rslt);
      i++;
      continue;
    }

    float temp, press, hum;

    temp = comp_data.temperature;
    press = 0.01 * comp_data.pressure;
    hum = comp_data.humidity;

    *ta = temp;
    return 0;
  }

  *ta = 24.0f;
  return -1;
}
