#define DT_DRV_COMPAT adi_max77961

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

LOG_MODULE_REGISTER(max77961, CONFIG_CHARGER_LOG_LEVEL);

/* Zephyr Charger API uses microA, microV
	MAX77961 uses miliA, miliV
	When using get_prop/ set_prop -> values in microA, microV */

/* MAX77961 Registers */
#define MAX77961_I2C_ADDR 0x69	    /* 7-bit I2C address */
#define MAX77961_REG_DEVICE_ID 0x00 /* Device ID register */
#define MAX77961_DEVICE_ID 0x51	    /* Expected device ID */

#define MAX77961_REG_INT_STS 0x03   /* Interrupt Status register */
#define MAX77961_VBUS_OK_STS BIT(3) /* VBUS_OK_STS bit for charger presence */

#define MAX77961_CHG_DTLS_01 0x14
#define CHG_CNFG_00 0x16 // COMM_MODE
#define CHG_CNFG_06 0x1C // CHGPROT
#define CHG_CNFG_04 0x1A


#define MAX77961_REG_CHGIN_ILIM 0x1E
#define MAX77961_REG_CHG_CC 0x18
#define MAX77961_REG_CHG_CV 0x1A

/* CHGIN_ILIM bitfield (bits 6:0) */
#define CHGIN_ILIM_MASK 0x7F
#define CHGIN_ILIM_MIN_MA 100
#define CHGIN_ILIM_STEP_MA 100
#define CHGIN_ILIM_MAX_MA 5000

/* CHG_CC bitfield (bits 5:0) */
#define CHGCC_MASK 0x3F
#define CHGCC_LIM_MIN_MA 100
#define CHGCC_LIM_STEP_MA 50
#define CHGCC_LIM_MAX_MA 6000
#define CHGCC_STEP_LOW_MA 50    /* Step size for 100-500mA range */
#define CHGCC_STEP_HIGH_MA 100  /* Step size for 500mA and above */
#define CHGCC_THRESHOLD_MA 500  /* Threshold where step size changes */

/* CHG_CV bitfield (bits 5:0) */
#define CHG_CV_MASK 0x3F
#define CHG_CV_MIN_MV 12000//4100
#define CHG_CV_STEP_MV 30
#define CHG_CV_MAX_MV 13050

/* Charger Status Bitfields (CHG_DTLS_01) */
#define CHG_DTLS_01_CHG_STAT_MASK 0x0F
#define CHG_DTLS_01_CHG_STAT_SHIFT 0

#define CHGPROT_UNLOCK      0x0C    // CHGPROT[1:0] = 11 (bits 3:2)
#define CHGPROT_LOCK        0x00    // CHGPROT[1:0] = 00 (bits 3:2)
#define CHGPROT_MASK        0x0C    // Mask for CHGPROT bits

/* Charger Status Values (simplified for CHARGER_PROP_STATUS) */
enum max77961_chg_status
{
	MAX77961_CHG_STAT_OFF = 0x00,
	MAX77961_CHG_STAT_PRECHARGE = 0x01,
	MAX77961_CHG_STAT_FAST_CHARGE = 0x02,
	MAX77961_CHG_STAT_TOP_OFF = 0x03,
	MAX77961_CHG_STAT_DONE = 0x04,
	MAX77961_CHG_STAT_FAULT = 0x05,
};

struct max77961_config
{
	struct i2c_dt_spec i2c;
};

struct max77961_charger_data
{
	uint32_t charge_current_ua;
	uint32_t input_current_limit_ua;
	uint32_t termination_voltage_uv;
};

static int max77961_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct max77961_config *config = dev->config;
	return i2c_reg_read_byte_dt(&config->i2c, reg, val);
}

static int max77961_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct max77961_config *cfg = dev->config;
	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

static int max77961_comm_i2c(const struct device *dev)
{
	// Ensure COMM_MODE = 1 (I²C mode)
	uint8_t reg_val;
	int ret = 0;
	ret =  max77961_reg_read(dev, CHG_CNFG_00, &reg_val);
	if ( ret != 0) {
		LOG_ERR("Error: Failed to read CHG_CNFG_00\n");
		return -1;
	}
	if ((reg_val & 0x80) == 0) { // Check COMM_MODE bit
		reg_val |= 0x80; // Set COMM_MODE = 1
		ret = max77961_reg_write(dev, CHG_CNFG_00, reg_val);
		if (ret != 0) {
			LOG_ERR("Error: Failed to set I²C mode\n");
            		return -1;
        	}
    	}
}

static int max77961_unlock_write(const struct device *dev)
{
    	uint8_t reg_val;
	int ret = 0;
	// Unlock charger settings (CHGPROT = 0b11)
	reg_val = (reg_val & ~CHGPROT_MASK) | CHGPROT_UNLOCK;
	ret = max77961_reg_write(dev, CHG_CNFG_06, reg_val);
    	if ( ret != 0) { // 0x0C = CHGPROT[1:0] = 0b11
		LOG_ERR("Error: Failed to unlock charger settings\n");
		return -1;
    	}
	k_sleep(K_MSEC(10));

	return ret;
}

static int max77961_lock_write(const struct device *dev)
{
	uint8_t reg_val;
	int ret = 0;
	ret =  max77961_reg_read(dev, CHG_CNFG_00, &reg_val);
	if ( ret != 0) {
		LOG_ERR("Error: Failed to read CHG_CNFG_00\n");
		return -1;
	}
	if ((reg_val & 0x80) == 0) { // Check COMM_MODE bit
		reg_val |= 0x80; // Set COMM_MODE = 1
		ret = max77961_reg_write(dev, CHG_CNFG_00, reg_val);
		if (ret != 0) {
			LOG_ERR("Error: Failed to set I²C mode\n");
            		return -1;
        	}
    	}
	reg_val = (reg_val & ~CHGPROT_MASK) | CHGPROT_LOCK;
	ret = max77961_reg_write(dev, CHG_CNFG_06, reg_val);
	if ( ret != 0) {
		LOG_ERR("Error: Failed to lock charger settings\n");
		return -1;
    	}
	k_sleep(K_MSEC(10));

	return ret;

}

static uint8_t current_ma_to_chgcc_reg(uint32_t current_ma)
{
	uint8_t reg_val;
	
	if (current_ma <= CHGCC_THRESHOLD_MA) {
		/* Range: 100-500mA with 50mA steps */
		reg_val = (current_ma - CHGCC_LIM_MIN_MA) / CHGCC_STEP_LOW_MA;
	} else {
		/* Range: 500mA+ with 100mA steps 
		   Register value 8 = 500mA (base of high range)
		   Formula: 8 + ((current - 500) / 100) */
		reg_val = 8 + ((current_ma - CHGCC_THRESHOLD_MA) / CHGCC_STEP_HIGH_MA);
	}
	
	return reg_val & CHGCC_MASK;
}
static uint32_t chgcc_reg_to_current_ma(uint8_t reg_val)
{
	uint32_t current_ma;
	
	reg_val &= CHGCC_MASK;
	
	if (reg_val <= 8) {
		/* Range: 100-500mA with 50mA steps */
		current_ma = CHGCC_LIM_MIN_MA + (reg_val * CHGCC_STEP_LOW_MA);
	} else {
		/* Range: 500mA+ with 100mA steps */
		current_ma = CHGCC_THRESHOLD_MA + ((reg_val - 8) * CHGCC_STEP_HIGH_MA);
	}
	
	return current_ma;
}
static int max77961_get_status(const struct device *dev, charger_prop_t prop, union charger_propval *val)
{
	if (prop != CHARGER_PROP_STATUS) {
		return -ENOTSUP;
	}
	uint8_t reg_val;
	int ret = max77961_reg_read(dev, MAX77961_CHG_DTLS_01, &reg_val);
	if (ret) {
		LOG_ERR("Failed to read CHG_DTLS_01: %d", ret);
		return ret;
	}

	uint8_t chg_stat = (reg_val & CHG_DTLS_01_CHG_STAT_MASK) >> CHG_DTLS_01_CHG_STAT_SHIFT;

	switch (chg_stat) {
	case MAX77961_CHG_STAT_OFF:
		val->status = CHARGER_STATUS_NOT_CHARGING;
		break;
	case MAX77961_CHG_STAT_PRECHARGE:
	case MAX77961_CHG_STAT_FAST_CHARGE:
	case MAX77961_CHG_STAT_TOP_OFF:
		val->status = CHARGER_STATUS_CHARGING;
		break;
	case MAX77961_CHG_STAT_DONE:
		val->status = CHARGER_STATUS_FULL;
		break;
	case MAX77961_CHG_STAT_FAULT:
		val->status = CHARGER_STATUS_DISCHARGING;
		break;
	default:
		val->status = CHARGER_STATUS_UNKNOWN;
		break;
	}
	return 0;
}

static int max77961_set_prop(const struct device *dev, charger_prop_t prop,
			     const union charger_propval *val)
{
	int ret = 0;

	ret = max77961_comm_i2c(dev);

	switch (prop)
	{
	case CHARGER_PROP_PRECHARGE_CURRENT_UA:
	{
		uint32_t current_ma = val->precharge_current_ua / 1000;
		ret = max77961_unlock_write(dev);
		if (current_ma < CHGIN_ILIM_MIN_MA || current_ma > CHGIN_ILIM_MAX_MA)
		{
			LOG_ERR("Invalid input current limit: %u mA", current_ma);
			return -EINVAL;
		}
		uint8_t reg_val = (current_ma - CHGIN_ILIM_MIN_MA) / CHGIN_ILIM_STEP_MA;
		reg_val &= CHGIN_ILIM_MASK;
		ret = max77961_reg_write(dev, MAX77961_REG_CHGIN_ILIM, reg_val);
		if (ret < 0)
		{
			LOG_ERR("Failed to set CHGIN_ILIM: %d", ret);
			return ret;
		}
		ret = max77961_lock_write(dev);
		LOG_INF("Set CHGIN_ILIM to %u mA (reg: 0x%02x)", current_ma, reg_val);
		break;
	}

	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
	{
		uint32_t current_ma = val->const_charge_current_ua / 1000;
		ret = max77961_unlock_write(dev);
		if (current_ma < CHGCC_LIM_MIN_MA || current_ma > CHGCC_LIM_MAX_MA)
		{
			LOG_ERR("Invalid charge current: %u mA", current_ma);
			return -EINVAL;
		}
		uint8_t reg_val = current_ma_to_chgcc_reg(current_ma);
		ret = max77961_reg_write(dev, MAX77961_REG_CHG_CC, reg_val);
		if (ret < 0)
		{
			LOG_ERR("Failed to set CHG_CC: %d", ret);
			return ret;
		}
		ret = max77961_lock_write(dev);
		LOG_INF("Set CHG_CC to %u mA (reg: 0x%02x)", current_ma, reg_val);
		break;
	}

	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
	{
		uint32_t voltage_mv = val->const_charge_voltage_uv / 1000;
		ret = max77961_unlock_write(dev);
		if (voltage_mv < CHG_CV_MIN_MV || voltage_mv > CHG_CV_MAX_MV)
		{
			LOG_ERR("Invalid charge voltage: %u mV", voltage_mv);
			return -EINVAL;
		}
		uint8_t reg_val = (voltage_mv - CHG_CV_MIN_MV) / CHG_CV_STEP_MV;
		reg_val &= CHG_CV_MASK;
		ret = max77961_reg_write(dev, MAX77961_REG_CHG_CV, reg_val);
		
		if (ret < 0)
		{
			LOG_ERR("Failed to set CHG_CV: %d", ret);
			return ret;
		}
		LOG_INF("Set CHG_CV to %u mV (reg: 0x%02x) ", voltage_mv, reg_val);
		ret = max77961_lock_write(dev);
		break;
	}

	default:
		LOG_ERR("Unsupported property: %d", prop);
		return -ENOTSUP;
	}

	return ret;
}

static int max77961_get_prop(const struct device *dev, charger_prop_t prop,
			     union charger_propval *val)
{
	int ret;

	switch (prop)
	{
	case CHARGER_PROP_PRECHARGE_CURRENT_UA:
	{
		uint8_t reg_val;
		ret = max77961_reg_read(dev, MAX77961_REG_CHGIN_ILIM, &reg_val);
		if (ret < 0)
		{
			LOG_ERR("Failed to read CHGIN_ILIM: %d", ret);
			return ret;
		}
		reg_val &= CHGIN_ILIM_MASK;
		val->precharge_current_ua = (CHGIN_ILIM_MIN_MA + (reg_val * CHGIN_ILIM_STEP_MA)) * 1000;
		LOG_INF("Read CHGIN_ILIM: %u uA (reg: 0x%02x)", val->precharge_current_ua, reg_val);
		break;
	}

	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
	{
		uint8_t reg_val;
		ret = max77961_reg_read(dev, MAX77961_REG_CHG_CC, &reg_val);
		if (ret < 0)
		{
			LOG_ERR("Failed to read CHG_CC: %d", ret);
			return ret;
		}
		uint32_t current_ma = chgcc_reg_to_current_ma(reg_val);
		val->const_charge_current_ua = current_ma * 1000;
		LOG_INF("Read CHGCC: %u uA (reg: 0x%02x)", val->const_charge_current_ua, reg_val);
		break;
	}

	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
	{
		uint8_t reg_val;
		ret = max77961_reg_read(dev, MAX77961_REG_CHG_CV, &reg_val);
		if (ret < 0)
		{
			LOG_ERR("Failed to read CHG_CV: %d", ret);
			return ret;
		}
		reg_val &= CHG_CV_MASK;
		val->const_charge_voltage_uv = ( CHG_CV_MIN_MV + (reg_val * CHG_CV_STEP_MV)) * 1000;
		LOG_INF("Read CHG_CV: %u uV (reg: 0x%02x)", val->const_charge_voltage_uv, reg_val);
		break;
	}

	case CHARGER_PROP_STATUS:
	{
		ret = max77961_get_status(dev, prop, val);
		if (ret < 0) {
			return ret;
		}
		break;
	}

	default:
		LOG_ERR("Unsupported property: %d", prop);
		return -ENOTSUP;
	}

	return 0;
}

static int max77961_init(const struct device *dev)
{
	const struct max77961_config *cfg = dev->config;
	struct max77961_charger_data *data = dev->data;
	uint8_t device_id;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c))
	{
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	uint8_t stat0;
	ret = max77961_reg_read(dev, MAX77961_CHG_DTLS_01, &stat0);
	if (ret < 0)
	{
		LOG_ERR("Failed to read STAT0 register: %d", ret);
		return ret;
	}

	LOG_INF("MAX77961 initialized successfully");
	return 0;
}


static const struct charger_driver_api max77961_driver_api = {
    .get_property = max77961_get_prop,
    .set_property = max77961_set_prop,
};


#define MAX77961_DEFINE(inst)                                                                                     \
	static struct max77961_charger_data max77961_charger_data_##inst;                                         \
	static const struct max77961_config max77961_config_##inst = {                                            \
	    .i2c = I2C_DT_SPEC_INST_GET(inst),                                                                    \
	};                                                                                                        \
														  \
	DEVICE_DT_INST_DEFINE(inst, &max77961_init, NULL, &max77961_charger_data_##inst, &max77961_config_##inst, \
			      POST_KERNEL, CONFIG_CHARGER_INIT_PRIORITY, &max77961_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX77961_DEFINE)