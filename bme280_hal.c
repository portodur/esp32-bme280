/*
 * bme280_hal.c
 *
 *  Created on: 27 dic. 2018
 *      Author: portodur
 */

#include "bme280_hal.h"

static struct bme280_dev dev;


void bme280_hal_init()
{
	uint8_t settings_sel;
	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;
	bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_1X;
	dev.settings.osr_t = BME280_OVERSAMPLING_1X;
	dev.settings.filter = BME280_FILTER_COEFF_OFF;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

	bme280_set_sensor_settings(settings_sel, &dev);
}


int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t rslt;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, BME280_I2C_ADDR_PRIM <<1|0, 0);
	i2c_master_write_byte(cmd, reg_addr, 0);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, BME280_I2C_ADDR_PRIM<<1|1, 0);
	i2c_master_read(cmd, data, len, 2);
	i2c_master_stop(cmd);
	rslt  = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return rslt;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t rslt;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BME280_I2C_ADDR_PRIM << 1) | 0, 0);
	i2c_master_write_byte(cmd, reg_addr, 0);
	i2c_master_write(cmd, data, len, 0);
	i2c_master_stop(cmd);
	rslt = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return rslt;
}


void user_delay_ms(uint32_t period)
{
	vTaskDelay(period / portTICK_RATE_MS);
}

int8_t stream_sensor_data_forced_mode()
{
	int8_t rslt;
	struct bme280_data comp_data;
	bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	dev.delay_ms(100);
	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	sensor.temp = comp_data.temperature;
	sensor.hum = comp_data.humidity;
	xQueueOverwrite(bme280_buffer, &sensor);
	return rslt;
}
