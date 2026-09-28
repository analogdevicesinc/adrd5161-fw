
#define DT_DRV_COMPAT adi_max17320

#include "max17320.h"

#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(max17320, CONFIG_FUEL_GAUGE_LOG_LEVEL);


struct max17320_data { 
	uint16_t soc;     /* State of charge in % * 100 */
	uint16_t voltage;   /* Voltage Battery in mV */
	uint16_t design_voltage; /* Design Voltage in mV */
	uint16_t voltage1;   /* Voltage in mV */
	uint16_t voltage2;   /* Voltage in mV */
	uint16_t voltage3;   /* Voltage in mV */
	uint16_t voltage4;   /* Voltage in mV */
	int current;    /* Current in uA */
	int16_t temp;       /* Temperature in °C */
	uint16_t remaining_capacity; /* mAh */
	uint16_t full_charge_capacity; /* mAh */
	uint16_t time_to_empty;
	uint16_t time_to_full;
	bool charging; /* True if battery is charging, false if discharging */
	uint8_t chg_off;

};
    
static int max17320_read_register(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct max17320_config *config = dev->config;
	uint8_t buf[2];
	int ret;

	ret = i2c_write_read_dt(&config->i2c, &reg, 1, buf, sizeof(buf));

	if (ret < 0) {
		LOG_ERR("Failed to read register 0x%02x: %d", reg, ret);
		return ret;
	}
	//*val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	*val = sys_get_le16(buf);
	return 0;
}

static int max17320_write_register(const struct device *dev, uint8_t reg, uint16_t val)
{
	uint8_t tx_buf[3];
	int ret;
	const struct max17320_config *config = dev->config;

	tx_buf[0] = reg;
	tx_buf[1] = (uint8_t)(val & 0XFF); // LSB first
	tx_buf[2] = (uint8_t)((val >> 8) & 0xFF); // MSB second 

	ret = i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf));

	if (ret < 0) {
		LOG_ERR("Failed to write register 0x%02x: %d", reg, ret);
		return ret;
	}

	return 0;
}

static int max17320_wait_nv_ready(const struct device *dev)
{
	int ret;
	uint16_t commstat;
	int timeout = 100; 

	while(timeout-- > 0) {
		ret = max17320_read_register(dev, MAX17320_COMMSTAT_REG, &commstat);
		if (ret < 0)
			return ret;
		if ((commstat & (1 << COMMSTAT_NVBUSY_BIT)) == 0)
			return 0;
		k_msleep(1);
	}

	LOG_ERR("Error waiting for NV memory to be ready");
	ret = -2; // timeout error 
	return ret; 

}

static int max17320_disable_write_protection(const struct device *dev)
{
	int ret;
	uint16_t commstat_val = 0x0000; // clear all write protection bits
	uint16_t readback;

	ret = max17320_wait_nv_ready(dev);
	if (ret < 0)
		return ret;
	LOG_DBG("Disabling write protection");

	ret =  max17320_write_register(dev, MAX17320_COMMSTAT_REG, commstat_val);
	if (ret < 0)
		return ret;
	
	ret = max17320_write_register(dev, MAX17320_COMMSTAT_REG, commstat_val);
	if (ret < 0)
		return ret;

	ret = max17320_read_register(dev, MAX17320_COMMSTAT_REG, &readback);
	if (ret < 0)
		return ret;
	LOG_DBG("Commstat Readback 0x%04X", readback);
	
	if ((readback & (COMMSTAT_WPGLOBAL_MASK | COMMSTAT_WP_ALL_MASK)) != 0) {
		LOG_ERR("Write protection not properly disabled, CommStat: 0x%04X", readback);
		return -3;
	}

	LOG_DBG("Write Protection disabled succesfully");
	return 0;
	

}

static int max17320_enable_write_protection(const struct device *dev)
{
	int ret;
	uint16_t commstat_value = COMMSTAT_WPGLOBAL_MASK | COMMSTAT_WP_ALL_MASK;
	
	LOG_DBG("Enabling Write Protection");

	ret = max17320_write_register(dev,  MAX17320_COMMSTAT_REG, commstat_value);
	if (ret < 0)
		return ret;
	ret = max17320_write_register(dev,  MAX17320_COMMSTAT_REG, commstat_value);
	if (ret < 0)
		return ret;
	LOG_DBG("Write protection re-enabled");

	return 0;
}

int max17320_voltage(const struct device *i2c_dev, uint16_t reg, uint16_t *response)
{
	uint16_t data;
	int ret = max17320_read_register(i2c_dev, reg, &data);

	if (ret < 0) {
		return ret;
	}

	*response = (data * MAX17320_VCELL_LSB_MV);
	return 0;
}

int max17320_battery(const struct device *i2c_dev, uint16_t *response)
{
	uint16_t data;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_BATT, &data);

	if (ret < 0) {
		return ret;
	}

	*response = (data * MAX17320_BATT_PACK_LSB);
	return 0;
}

int max17320_get_design_voltage(const struct device *i2c_dev, uint16_t *response)
{
	uint16_t data;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_PCKP, &data);
	if (ret < 0) {
		return ret;
	}

	*response = (data * MAX17320_BATT_PACK_LSB);
	return 0;
}

int max17320_percent(const struct device *i2c_dev, uint16_t *response)
{	
	uint16_t data;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_REPSOC, &data);
	if (ret < 0) {
	    return ret;
	}

	*response = data / 256;
	if (*response > 100) 
		*response = 100;
 
	return 0;

}

int max17320_current(const struct device *i2c_dev, int *response) {
	
	int16_t data;
	int16_t rsense_miliohms = 1;
	
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_CURRENT, &data);
	if (ret < 0) {
	    return ret;
	}
	//*response = ((data * MAX17320_CURRENT_LSB_UA) / 10) / rsense_miliohms; /* Convert to uA */
	*response = (data * 15625)/10;
	return 0;

}

int max17320_temp(const struct device *i2c_dev, int16_t *response)
{
	int16_t data;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_TEMP, &data);
	if (ret < 0) {
	    return ret;
	}
	*response = data / 256;

}

int max17320_rem_cap(const struct device *i2c_dev, uint16_t *response)
{
	uint16_t data;
	int rsense_miliohms = 1;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_REPCAP, &data);
	if (ret < 0) {
		return ret;
	} 
	*response = (data * MAX17320_CAPACITY_LSB_MAH);/* mAh */
	return 0;
}

int max17320_full_cap(const struct device *i2c_dev, uint16_t *response)
{
	uint16_t data;
	int rsense_miliohms = 1;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_FULLCAPREP, &data); 
	if (ret < 0) {
		return ret;
	}
	*response = data * MAX17320_CAPACITY_LSB_MAH;//(data * MAX17320_CAPACITY_LSB_UAH ) / 10; /* mAh */
	return 0;

}

int max17320_time_to_empty(const struct device *i2c_dev, uint16_t *response)
{
	uint16_t data;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_TTE, &data); 
	if (ret < 0) {
		return ret;
	}
	*response = (data * 5625) / 1000 / 60; //minutes
	return 0;
}

int max17320_time_to_full(const struct device *i2c_dev, uint16_t *response)
{
	uint16_t data;
	int ret = max17320_read_register(i2c_dev, MAX17320_REG_TTF, &data); 
	if (ret < 0) {
		return ret;
	}
	*response =  (data * 5625) / 1000 / 60; //minutes
	return 0;
}

int max17320_write_fullcap(const struct device *i2c_dev, uint16_t cap_mah)
{

	int ret;
	bool wp_enabled = false;
	uint16_t original_commstat;
	ret = max17320_read_register(i2c_dev, MAX17320_COMMSTAT_REG, &original_commstat);
	if (ret < 0)
		return ret;

	wp_enabled = (original_commstat &(COMMSTAT_WPGLOBAL_MASK | (1 << COMMSTAT_WP2_BIT))) != 0;
	if(wp_enabled) {
		LOG_DBG("Disabling write protection to configure FullCapRep register");
		ret = max17320_disable_write_protection(i2c_dev);
		if(ret < 0 ) {
			LOG_ERR("Failed to disable write prottection");
			return ret;
		}
	}

	uint16_t reg_val = cap_mah / MAX17320_CAPACITY_LSB_MAH;
	ret = max17320_write_register(i2c_dev, MAX17320_REG_FULLCAPREP, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write FullCapRep register, enabling write protection");
		max17320_enable_write_protection(i2c_dev);
		return ret;
	}

	uint16_t readback;
	ret = max17320_read_register(i2c_dev, MAX17320_REG_FULLCAPREP, &readback);
	if (ret < 0) {
		LOG_ERR("Failed to read back FullCapRep Register");
		return ret;
	}

	if(readback != reg_val) {
		LOG_ERR("FullCapRep write validation failed. Expected: 0x%04X. Got: 0x%04X", reg_val, readback);
		return -3;
	}

	if (wp_enabled) {

		ret = max17320_enable_write_protection(i2c_dev);
		if(ret == 0) {
			LOG_DBG("Write protection restored");
		}
		else {
			LOG_DBG("Failed to enable write protection after succesfull write to FullCapRep Register");
		}
	}

	return 0;

}

int max17320_set_fets(const struct device *dev, uint8_t chgoff, uint8_t disoff)
{
	int ret;
	uint16_t commstat_val;
	uint16_t readback;
	int actual_chgoff, actual_disoff;
	LOG_DBG("Start MAX17320 FETs configuration");

	ret = max17320_disable_write_protection(dev);
	if ( ret < 0 ) {
		LOG_ERR("Failed to disable write protection");
		return ret;
	}

	ret = max17320_read_register(dev, MAX17320_COMMSTAT_REG, &commstat_val);
	if (ret < 0)
		return ret;

	if (commstat_val & COMMSTAT_WPGLOBAL_MASK) {
		LOG_ERR("Write Protection still enabled");
		return -3;
	}

	commstat_val &= ~(COMMSTAT_CHGOFF_MASK | COMMSTAT_DISOFF_MASK); // clear fet bits

	if (chgoff)
		commstat_val |= COMMSTAT_CHGOFF_MASK;
	if (disoff)
		commstat_val |= COMMSTAT_DISOFF_MASK;

	LOG_DBG("Setting CommStat to: 0x%04X (CHG_OFF=%d, DIS_OFF=%d)", 
	commstat_val, chgoff ? 1 : 0, disoff ? 1 : 0);

	//CommStat Register needs to be written twice

	ret = max17320_write_register(dev, MAX17320_COMMSTAT_REG, commstat_val);
	if (ret < 0)
		return ret;
	
	ret = max17320_write_register(dev, MAX17320_COMMSTAT_REG, commstat_val);
	if (ret < 0)
		return ret;

	k_msleep(10);

	// check bits are set correctly
	ret = max17320_read_register(dev, MAX17320_COMMSTAT_REG, &readback);
	if (ret < 0)
		return ret;

	actual_chgoff = (readback & COMMSTAT_CHGOFF_MASK) ? 1 : 0;
	if (actual_chgoff != chgoff){
		LOG_ERR("CHGOFF bit not set correctly. Expected: %d. Got : %d",
		chgoff ? 1 : 0, actual_chgoff ? 1 : 0);
		return -4; // bits not set error
	}

	actual_disoff = (readback & COMMSTAT_DISOFF_MASK) ? 1 : 0;
	if (actual_disoff != disoff){
		LOG_ERR("DISOFF bit not set correctly. Expected: %d. Got : %d",
		disoff ? 1 : 0, actual_disoff ? 1 : 0);
		return -4; // bits not set error
	}

	LOG_DBG("FET control set succesfully: CHG FET - %s, Discharge FET - %s",
	chgoff ? "disabled" : "enabled", disoff ? "disabled" : "enabled");

	ret = max17320_enable_write_protection(dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int max17320_get_fets_status(const struct device *dev, uint8_t *chgoff, uint8_t *disoff)
{
	uint16_t commstat_val;
	int ret;

	ret = max17320_read_register(dev, MAX17320_COMMSTAT_REG, &commstat_val);
	if (ret < 0 )
		return ret;

	*chgoff = (commstat_val & COMMSTAT_CHGOFF_MASK) ? 1 : 0;
	*disoff = (commstat_val & COMMSTAT_DISOFF_MASK) ? 1 : 0;

	LOG_DBG("FET Status - CHGOFF: %s, DISOFF: %s",
		*chgoff ? "disabled" : "enabled",
		*disoff ? "disabled" : "enabled");
	return 0;
}

static int max17320_set_prop(const struct device *dev,
			      fuel_gauge_prop_t prop,
			      union fuel_gauge_prop_val val)
{
	int ret = 0;
	int tmp_val = 0;
	struct max17320_data *data = dev->data;
	switch (prop) {
		case FUEL_GAUGE_FLAGS:
			if( val.flags == 1 ) { 
				ret = max17320_set_fets(dev, 1, 0);
				if (ret < 0)
					return ret;
			}
			if( val.flags == 0 ){
				ret = max17320_set_fets(dev, 0, 0);
				if (ret < 0)
					return ret;
			}
			val.flags = tmp_val;
			break;
		case FUEL_GAUGE_FULL_CHARGE_CAPACITY:
			uint16_t value = val.full_charge_capacity;
			ret = max17320_write_fullcap(dev, value);
			if (ret < 0)
				return ret;
			val.full_charge_capacity = tmp_val;
			break;
	}
	return ret;
}
static int max17320_get_single_prop_impl(const struct device *dev,
					fuel_gauge_prop_t prop,
					union fuel_gauge_prop_val *val)
{
	struct max17320_data *data = dev->data;
	int ret = 0;

	switch (prop) {
		case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE:
			ret = max17320_percent(dev, &data->soc);
			if (ret < 0)
				return ret;
			val->relative_state_of_charge = data->soc;
			break;
		case FUEL_GAUGE_VOLTAGE:
			ret = max17320_battery(dev, &data->voltage);
			if (ret < 0)
				return ret;	
			val->voltage = data->voltage;
			break;
		case FUEL_GAUGE_CURRENT:
			ret = max17320_current(dev, &data->current);
			if (ret < 0)
				return ret;
			val->current = data->current;
			break;
		case FUEL_GAUGE_TEMPERATURE:
			ret = max17320_temp(dev, &data->temp);
			if (ret < 0)
				return ret;
			val->temperature = data->temp;
			break;
		case FUEL_GAUGE_REMAINING_CAPACITY:
			ret = max17320_rem_cap(dev, &data->remaining_capacity);
			if (ret < 0)
				return ret;
			val->remaining_capacity = data->remaining_capacity;
			break;
		case FUEL_GAUGE_FULL_CHARGE_CAPACITY: 
			ret = max17320_full_cap(dev, &data->full_charge_capacity);
			if (ret < 0)
				return ret;
			val->full_charge_capacity = data->full_charge_capacity;
			break;
		case FUEL_GAUGE_RUNTIME_TO_EMPTY:
			ret = max17320_time_to_empty(dev, &data->time_to_empty);
			if (ret < 0)
				return ret;
			val->runtime_to_empty = data->time_to_empty;
			break;
		case FUEL_GAUGE_RUNTIME_TO_FULL:
			ret = max17320_time_to_full(dev, &data->time_to_full);
			if (ret < 0)
				return ret;
			val->runtime_to_full = data->time_to_full;
			break;
		case FUEL_GAUGE_FLAGS:
			uint8_t chgoff, disoff;
			ret = max17320_get_fets_status(dev, &chgoff, &disoff);
			if(chgoff == 1)
				val->flags = 1;
			if(chgoff == 0)
				val->flags = 0;
			break;
		case FUEL_GAUGE_DESIGN_VOLTAGE:
			ret = max17320_get_design_voltage(dev, &data->design_voltage);
			if(ret < 0)
				return ret;
			val->design_volt = data->design_voltage;
			break;
		default:
			return -ENOTSUP;
	}

	return 0;
}

static int max17320_get_prop(const struct device *dev,
			      fuel_gauge_prop_t prop,
			      union fuel_gauge_prop_val *val)
{
	struct max17320_data *data = dev->data;
	int ret;

	if (prop == FUEL_GAUGE_VOLTAGE) {
		ret = max17320_battery(dev, &data->voltage);
		if (ret < 0) {
			return ret;
		}
		val->voltage = data->voltage;

		ret = max17320_current(dev, &data->current);
		if (ret < 0) {
			return ret;
		}
		data->charging = (data->current >= 0);

		ret = max17320_voltage(dev, MAX17320_REG_VCELL1, &data->voltage1);
		if (ret < 0) {
			return ret;
		}
		ret = max17320_voltage(dev, MAX17320_REG_VCELL2, &data->voltage2);
		if (ret < 0) {
			return ret;
		}
		ret = max17320_voltage(dev, MAX17320_REG_VCELL3, &data->voltage3);
		if (ret < 0) {
			return ret;
		}
		return 0;
	}

	return max17320_get_single_prop_impl(dev, prop, val);
}

static int max17320_init(const struct device *dev)
{
	const struct max17320_config *config = dev->config;
	uint16_t devname = 0;
	int ret = 0;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	/* Verify device by reading device name register */

	
	// TODO: Investigate timing issue - 
	// When cold booting, MAX17320 does not read out the correct
	// DEVNAME register. Maybe MAX17320 does not powerup completely before 
	// initialization occurs
	// This can be due to a chip boot-up issue
	// or a threading issue when starting the MAX17320 driver

	k_msleep(2000);
	
	ret = max17320_read_register(dev, MAX17320_REG_DEVNAME, &devname);
	if (ret < 0) {
		LOG_ERR("Failed to read device name: %d", ret);
		return ret;
	}

	if (devname != MAX17320_DEVNAME_VALUE) {
		LOG_ERR("Invalid device name 0x%04x, expected 0x%04x",
					devname, MAX17320_DEVNAME_VALUE);
		return -ENODEV;
	}

	LOG_INF("MAX17320 initialized successfully");
	return 0;
}

static const struct fuel_gauge_driver_api max17320_driver_api = {
    .get_property = &max17320_get_prop,
    .set_property = &max17320_set_prop,
};

#define MAX17320_DEFINE(inst)                                                                      \
	static struct max17320_data max17320_data_##inst;                                          \
												   \
	static const struct max17320_config max17320_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst)};                                                \
												   \
	DEVICE_DT_INST_DEFINE(inst, &max17320_init, NULL, &max17320_data_##inst,                   \
			&max17320_config_##inst, POST_KERNEL,                                \
			CONFIG_FUEL_GAUGE_INIT_PRIORITY, &max17320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17320_DEFINE)
