#define DT_DRV_COMPAT adi_max31827

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(max31827, CONFIG_SENSOR_LOG_LEVEL);

/* MAX31827 Register Addresses */
#define MAX31827_REG_TEMP        0x00  /* Temperature register (16-bit) */
#define MAX31827_REG_CONFIG      0x01  /* Configuration register */

/* Default I2C address */
#define MAX31827_I2C_ADDR        0x5a

/* Configuration values */
#define MAX31827_CONFIG_ONE_SHOT 0x01  /* One-shot conversion mode */
#define MAX31827_CONFIG_CONT     0x00  /* Continuous conversion mode */

struct max31827_config {
	struct i2c_dt_spec i2c;
};

struct max31827_data {
	int16_t temperature;
};

/* Read a 16-bit register from MAX31827 */
static int max31827_reg_read(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct max31827_config *cfg = dev->config;
	uint8_t buf[2];
	int ret;

	ret = i2c_write_read_dt(&cfg->i2c, &reg, 1, buf, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read register 0x%02x: %d", reg, ret);
		return ret;
	}

	/* MAX31827 returns MSB first */
	*val = (buf[0] << 8) | buf[1];

	return 0;
}

static int max31827_reg_write(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct max31827_config *cfg = dev->config;
	uint8_t buf[3];

	buf[0] = reg;
	buf[1] = (val >> 8) & 0xFF;  /* MSB */
	buf[2] = val & 0xFF;          /* LSB */

	return i2c_write_dt(&cfg->i2c, buf, 3);
}

/* Sample fetch - triggers and reads temperature */
static int max31827_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct max31827_data *data = dev->data;
	uint16_t raw_temp;
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	//k_msleep(100);

	/* Read temperature register */
	ret = max31827_reg_read(dev, MAX31827_REG_TEMP, &raw_temp);
	if (ret < 0) {
		return ret;
	}

	data->temperature = (int16_t)raw_temp;
    
    return 0;
}

/* Channel get - returns temperature value */
static int max31827_channel_get(const struct device *dev,
				 enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct max31827_data *data = dev->data;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	int32_t temp_micro = (int32_t)data->temperature * 62500;
	val->val1 = temp_micro / 1000000;
	val->val2 = temp_micro % 1000000;

	return 0;
}

/* Initialize the sensor */
static int max31827_init(const struct device *dev)
{
	const struct max31827_config *cfg = dev->config;
	uint16_t config_val;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	//k_msleep(2000);

	ret = max31827_reg_read(dev, MAX31827_REG_CONFIG, &config_val);
    	if (ret < 0) {
        	LOG_ERR("Failed to communicate with device (check I2C address and wiring): %d", ret);
        	return ret;
	}

	/* Configure for continuous conversion mode */
	ret = max31827_reg_write(dev, MAX31827_REG_CONFIG, MAX31827_CONFIG_CONT);
	if (ret < 0) {
		LOG_ERR("Failed to configure device: %d", ret);
		return ret;
	}

	LOG_INF("MAX31827 initialized");
	return 0;
}

static const struct sensor_driver_api max31827_api = {
	.sample_fetch = max31827_sample_fetch,
	.channel_get = max31827_channel_get,
};

#define MAX31827_DEFINE(inst)                                                   \
	static struct max31827_data max31827_data_##inst;                          \
										\
	static const struct max31827_config max31827_config_##inst = {             \
	.i2c = I2C_DT_SPEC_INST_GET(inst),                                     \
	};                                                                        \
										\
	DEVICE_DT_INST_DEFINE(inst,                                                \
			  max31827_init,                                       \
			  NULL,                                                \
			  &max31827_data_##inst,                               \
			  &max31827_config_##inst,                             \
			  POST_KERNEL,                                         \
			  CONFIG_SENSOR_INIT_PRIORITY,                         \
			  &max31827_api);

DT_INST_FOREACH_STATUS_OKAY(MAX31827_DEFINE)