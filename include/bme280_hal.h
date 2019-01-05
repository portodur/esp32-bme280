/*
 * bme280_hal.h
 *
 *  Created on: 27 dic. 2018
 *      Author: portodur
 */

#ifndef BME280_HAL_H_
#define BME280_HAL_H_

#include <stdint.h>
#include <stddef.h>
#include "driver/i2c.h"
#include "bme280.h"
#include "bme280_defs.h"


xQueueHandle bme280_buffer;

struct sensor_t{
	double temp;
	double hum;
} sensor;

void bme280_hal_init();

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
int8_t stream_sensor_data_forced_mode();


#endif /* COMPONENTS_BME280_INCLUDE_BME280_HAL_H_ */
