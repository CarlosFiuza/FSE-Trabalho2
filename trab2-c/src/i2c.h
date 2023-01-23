#ifndef I2C_H_
#define I2C_H_

int i2c_init();
void i2c_init_thread(int *running, float *ta);
void i2c_join_thread();

#endif