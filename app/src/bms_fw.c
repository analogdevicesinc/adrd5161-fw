#include "button_manager/button_manager.h"
#include "drivers/adp5589.h"
#include <canopennode.h>
#include <app_version.h>
#include "301/CO_ODinterface.h"
#include <OD.h>
#include <CANopen.h>
#include <CO_driver_target.h>
#include <wrap_max32_sys.h>
#include <zephyr/devicetree.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/smf.h>

LOG_MODULE_REGISTER(bms_fw, LOG_LEVEL_INF);

/* Forward declaration of state table */
static const struct smf_state bms_states[];

/* List of states */
enum bms_state {
	BMS_CONFIG,
	BMS_NORMAL,
	BMS_CHARGING,
	BMS_LOW_POWER,
	BMS_SHUTDOWN,
	BMS_ERROR,
};

/* Fault codes surfaced on the OLED via the BMS_ERROR state */
typedef enum {
	BMS_ERR_NONE = 0,
	BMS_ERR_FG_TIMEOUT,   /* persistent fuel gauge read failures */
	BMS_ERR_EXPANDER,     /* GPIO expander unresponsive */
	BMS_ERR_PERM_FG_FAIL, /* permanent fuel gauge failure */
	BMS_ERR_GENERIC,
} bms_error_t;

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

typedef struct {
	uint16_t batteryCellVoltages[4];
	int32_t current;
	uint8_t batteryStatus;
	uint8_t chargerStatus;
	int16_t temperature;
	uint16_t ahReturnedDuringLastCharge;
	uint32_t batteryVoltage;
	uint16_t chargeCurrentRequested;
	uint8_t chargerStateOfCharge;
	uint8_t batteryStateOfCharge;
} bms_status_t;

bms_status_t bms_status;

static int i2c_cooldown = 0;
static int i2c_cooldown_base = 5;
static int i2c_recovery_failures = 0;
static int temp_sensor_errors = 0;
static int consecutive_lock_failures = 0;
static int consecutive_expander_errors = 0;
static int consecutive_fg_errors = 0;
#define MAX_LOCK_FAILURES 20
#define MAX_EXPANDER_ERRORS 10
#define MAX_FG_ERRORS 15

static uint32_t lock_fail_window_start = 0;
static int lock_fail_window_count = 0;
#define LOCK_FAIL_WINDOW_MS 30000
#define LOCK_FAIL_WINDOW_MAX 15

struct s_object {
	/* State machine context */
	struct smf_ctx ctx;

	/* Other state specific data */
	int32_t some_data;
} s_obj;

typedef enum {
	CONFIG_STATE_INIT,
	CONFIG_STATE_WAIT_FOR_ENTRY,
	CONFIG_STATE_SET_CAPACITY,
	CONFIG_STATE_SET_VOLTAGE,
	CONFIG_STATE_SET_CURRENT,
	CONFIG_STATE_FINISH,
} config_sub_state_t;

static config_sub_state_t config_sub_state = CONFIG_STATE_INIT;

static union fuel_gauge_prop_val fg_vals[10] = {0};
static union charger_propval chg_val_c;
static union charger_propval chg_val_v;
static const fuel_gauge_prop_t props[] = {FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE,
					  FUEL_GAUGE_VOLTAGE,
					  FUEL_GAUGE_CURRENT,
					  FUEL_GAUGE_TEMPERATURE,
					  FUEL_GAUGE_REMAINING_CAPACITY,
					  FUEL_GAUGE_FULL_CHARGE_CAPACITY,
					  FUEL_GAUGE_RUNTIME_TO_EMPTY,
					  FUEL_GAUGE_RUNTIME_TO_FULL,
					  FUEL_GAUGE_FLAGS,
					  FUEL_GAUGE_DESIGN_VOLTAGE};

static const struct gpio_dt_spec intmcu_1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(intmcu1), gpios);
static const struct gpio_dt_spec intmcu_2 =
    GPIO_DT_SPEC_GET(DT_ALIAS(intmcu2), gpios);
static const struct gpio_dt_spec intmcu_3 =
    GPIO_DT_SPEC_GET(DT_ALIAS(intmcu3), gpios);
static const struct gpio_dt_spec intmcu_4 =
    GPIO_DT_SPEC_GET(DT_ALIAS(intmcu4), gpios);
static const struct gpio_dt_spec pfail_fg =
    GPIO_DT_SPEC_GET(DT_ALIAS(pfailfg), gpios);
static const struct gpio_dt_spec stat_chg =
    GPIO_DT_SPEC_GET(DT_ALIAS(statchg), gpios);
static const struct gpio_dt_spec inokn_chg =
    GPIO_DT_SPEC_GET(DT_ALIAS(inoknchg), gpios);
static const struct gpio_dt_spec fpp1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(faultpowerpath1), gpios);
static const struct gpio_dt_spec fpp2 =
    GPIO_DT_SPEC_GET(DT_ALIAS(faultpowerpath2), gpios);

const struct device *temp_sensor =
	DEVICE_DT_GET(DT_NODELABEL(max31827));
// static const struct gpio_dt_spec shdn_powerpath =
// GPIO_DT_SPEC_GET(DT_ALIAS(shdnpowerpath), gpios);
static const struct device *can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CAN_BITRATE                                                            \
	(DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate,                         \
			    DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed,            \
					       CONFIG_CAN_DEFAULT_BITRATE)) /                  \
	 1000)

struct canopen co;
extern struct k_thread canopen_thread;
struct adp5589_dev *exp_dev = 0;

struct k_mutex i2c_mutex;

static volatile uint32_t main_loop_heartbeat;
static uint32_t sw_wdt_last_seen;
static int sw_wdt_stall_count;

static void sw_wdt_expiry(struct k_timer *timer)
{
	uint32_t current = main_loop_heartbeat;
	if (current == sw_wdt_last_seen) {
		sw_wdt_stall_count++;
		if (sw_wdt_stall_count >= 3) {
			sys_reboot(SYS_REBOOT_COLD);
		}
	} else {
		sw_wdt_stall_count = 0;
		sw_wdt_last_seen = current;
	}
}

K_TIMER_DEFINE(sw_wdt_timer, sw_wdt_expiry, NULL);

bool powerpath_battery_enabled = false;
bool powerpath_vbus_enabled = false;

/* Fault to display in BMS_ERROR; set via enter_error() before the transition */
static bms_error_t g_error_code = BMS_ERR_NONE;
/* Set in NORMAL when the charge FET cannot be enabled; drives the OLED warning */
static bool charge_fault_active = false;
/* Grace-period tracking for confirming real charging progress in CHARGING */
static int charging_confirm_counter = 0;
static bool not_charging_warn = false;
#define CHARGING_CONFIRM_GRACE 5 /* refresh cycles before warning */

static void bms_config_entry(void *o);
static void bms_config_run(void *o);
static void bms_normal_entry(void *o);
static void bms_normal_run(void *o);
static void bms_charging_entry(void *o);
static void bms_charging_run(void *o);
static void bms_low_power_entry(void *o);
static void bms_low_power_run(void *o);
static void bms_shutdown_entry(void *o);
static void bms_shutdown_run(void *o);
static void bms_error_entry(void *o);
static void bms_error_run(void *o);

static const struct smf_state bms_states[] = {
    [BMS_CONFIG] =
	SMF_CREATE_STATE(bms_config_entry, bms_config_run, NULL, NULL, NULL),
    [BMS_NORMAL] =
	SMF_CREATE_STATE(bms_normal_entry, bms_normal_run, NULL, NULL, NULL),
    [BMS_CHARGING] = SMF_CREATE_STATE(bms_charging_entry, bms_charging_run,
				      NULL, NULL, NULL),
    [BMS_LOW_POWER] = SMF_CREATE_STATE(bms_low_power_entry, bms_low_power_run,
				       NULL, NULL, NULL),
    [BMS_SHUTDOWN] = SMF_CREATE_STATE(bms_shutdown_entry, bms_shutdown_run,
				      NULL, NULL, NULL),
    [BMS_ERROR] =
	SMF_CREATE_STATE(bms_error_entry, bms_error_run, NULL, NULL, NULL),
};

static int init_mcupins_config() {
	int ret;
	ret = gpio_pin_configure_dt(&inokn_chg, GPIO_INPUT);
	ret = gpio_pin_configure_dt(&pfail_fg,
				    GPIO_INPUT | GPIO_INT_EDGE_FALLING);
	ret = gpio_pin_configure_dt(&intmcu_4,
				    GPIO_INPUT | GPIO_INT_EDGE_FALLING);
	ret = gpio_pin_configure_dt(&intmcu_1,
				    GPIO_INPUT | GPIO_INT_EDGE_FALLING);
	ret = gpio_pin_configure_dt(&intmcu_2,
				    GPIO_INPUT | GPIO_INT_EDGE_FALLING);
	ret = gpio_pin_configure_dt(&intmcu_3,
				    GPIO_INPUT | GPIO_INT_EDGE_FALLING);
	ret = gpio_pin_configure_dt(&fpp1, GPIO_INPUT | GPIO_INT_EDGE_FALLING);
	ret = gpio_pin_configure_dt(&fpp2, GPIO_INPUT | GPIO_INT_EDGE_FALLING);

	if (ret < 0) {
		LOG_ERR("failed to config GPIOs");
		return ret;
	}
	return 0;
}

static int init_adp5589_config(struct adp5589_dev *dev)
{
	uint8_t mask_a = 0x30;
	uint8_t mask_b = 0x08;
	uint8_t mask_c = 0x07;

	//lock_mutex_helper();
	adp5589_gpio_direction(dev, ADP5589_ADR_GPIO_DIRECTION_A, mask_a);
	adp5589_gpio_direction(dev, ADP5589_ADR_GPIO_DIRECTION_C, mask_c);

	uint8_t low_state =
	    (uint8_t)adp5589_get_pin_state(dev, ADP5589_ADR_GPO_DATA_OUT_A) &
	    ~mask_a;
	adp5589_set_register_value(dev, ADP5589_ADR_GPO_DATA_OUT_A, mask_a);

	low_state =
	    (uint8_t)adp5589_get_pin_state(dev, ADP5589_ADR_GPO_DATA_OUT_C) &
	    ~mask_c;
	adp5589_set_register_value(dev, ADP5589_ADR_GPO_DATA_OUT_C, low_state);

	//unlock_mutex_helper();

	return 0;
}

static int init_display_config() {
	const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

	if (display_dev == NULL) {
		LOG_ERR("display pointer is null\n");
		return -1;
	}

	if (!device_is_ready(display_dev)) {
		LOG_ERR("display not ready\n");
		return -1;
	}

	if (cfb_framebuffer_init(display_dev)) {
		LOG_ERR("cfb init failed\n");
		return -1;
	}

	return 0;
}

void update_bms_status_from_max17320(struct max17320_data *fg_data,
				     bms_status_t *bms) {

	bms->batteryCellVoltages[0] = fg_data->voltage1;
	bms->batteryCellVoltages[1] = fg_data->voltage2;
	bms->batteryCellVoltages[2] = fg_data->voltage3;
	bms->batteryCellVoltages[3] = fg_data->voltage4;

	bms->batteryStateOfCharge = fg_data->soc;
	bms->batteryVoltage = fg_data->voltage;
	bms->current = fg_data->current;
	bms->temperature = fg_data->temp;

}

void update_bms_status_from_max77961(const struct device *chg_dev,
				     bms_status_t *bms) {

	union charger_propval chg_val;

	int ret = charger_get_prop(chg_dev, CHARGER_PROP_STATUS, &chg_val);
	if (ret == 0) {
		bms->chargerStatus = chg_val.status;
	}
}
struct sensor_value check_temp_sensor() {
	struct sensor_value temp = {0, 0};

	if (temp_sensor_errors >= 3) {
		return temp;
	}

	int ret = sensor_sample_fetch(temp_sensor);
	if (ret < 0) {
		temp_sensor_errors++;
		LOG_WRN("Temp sensor read failed (%d), error count: %d", ret, temp_sensor_errors);
		return temp;
	}

	temp_sensor_errors = 0;
	sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp);

	return temp;
}

void update_configure_display(struct max17320_data *fg_data,
			      union fuel_gauge_prop_val fg_vals[9],
			      union charger_propval chg_val_c,
			      union charger_propval chg_val_v,
			      const struct device *display_dev) {
	int ret;
	char buf[256];
	LOG_INF("\n---------- Configure State ---------- \n");
	LOG_INF("SoC: %d %%\n", fg_vals[0].relative_state_of_charge);
	LOG_INF("Voltage: %d mV\n", fg_vals[1].voltage);
	//LOG_INF("Design Voltage: %d mV\n", fg_vals[9].design_volt);
	LOG_INF("Voltage Cell 1 %d \n", fg_data->voltage1);
	LOG_INF("Voltage Cell 2 %d \n", fg_data->voltage2);
	LOG_INF("Voltage Cell 3 %d \n", fg_data->voltage3);
	LOG_INF("Current: %d uA\n", fg_vals[2].current);
	LOG_INF("Temperature: %d °C\n", fg_vals[3].temperature);
	LOG_INF("Remaining Capacity: %d mAh\n", fg_vals[4].remaining_capacity);
	LOG_INF("Full Capacity: %d mAh\n", fg_vals[5].full_charge_capacity);
	LOG_INF("Runtime to empty : %d mins \n", fg_vals[6].runtime_to_empty);
	LOG_INF("Time to full : %d mins \n", fg_vals[7].runtime_to_full);
	LOG_INF("Charging Current: %d uA\n", chg_val_c.const_charge_current_ua);

	cfb_framebuffer_clear(display_dev, true);
	snprintf(buf, sizeof(buf), "Configure");
	cfb_print(display_dev, buf, 0, 0);
	snprintf(buf, sizeof(buf), "FCap:%dmAh", fg_vals[5].full_charge_capacity);
	cfb_print(display_dev, buf, 0, 15);

	snprintf(buf, sizeof(buf), "Vt:%dmV",
		 chg_val_v.const_charge_voltage_uv / 1000);
	cfb_print(display_dev, buf, 0, 30);

	snprintf(buf, sizeof(buf), "ChgC:%dmA",
		 chg_val_c.const_charge_current_ua / 1000);
	cfb_print(display_dev, buf, 0, 45);
	cfb_framebuffer_finalize(display_dev);
}

/* Full-screen 3-line message on the OLED. Any line may be NULL to skip it. */
static void show_message_display(const struct device *display_dev,
				 const char *l0, const char *l1,
				 const char *l2)
{
	cfb_framebuffer_clear(display_dev, true);
	if (l0)
		cfb_print(display_dev, (char *)l0, 0, 0);
	if (l1)
		cfb_print(display_dev, (char *)l1, 0, 15);
	if (l2)
		cfb_print(display_dev, (char *)l2, 0, 30);
	cfb_framebuffer_finalize(display_dev);
}

/* Human-readable reason for a fault code shown in the BMS_ERROR state. */
static const char *error_text(bms_error_t code)
{
	switch (code) {
	case BMS_ERR_FG_TIMEOUT:
		return "Fuel gauge I2C";
	case BMS_ERR_EXPANDER:
		return "IO expander";
	case BMS_ERR_PERM_FG_FAIL:
		return "Perm FG fail";
	case BMS_ERR_GENERIC:
	default:
		return "System fault";
	}
}

/* Latch a fault code and hand control to the BMS_ERROR state. */
static void enter_error(bms_error_t code)
{
	g_error_code = code;
	LOG_ERR("Entering error state: %s (%d)", error_text(code), code);
	smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_ERROR]);
}

/*
 * Make sure the charge FET is enabled so the pack can actually charge.
 * FUEL_GAUGE_FLAGS reports/controls charge-FET-off (1 = disabled, 0 = enabled)
 * via the MAX17320 CommStat CHGOFF bit. Returns 0 when the charge FET is
 * enabled, negative if it could not be enabled.
 */
static int ensure_charge_fet_enabled(const struct device *fg_dev)
{
	union fuel_gauge_prop_val v;
	int ret;

	ret = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_FLAGS, &v);
	if (ret < 0) {
		LOG_ERR("Failed to read FET status: %d\n", ret);
		return ret;
	}
	if (v.flags == 0) {
		return 0; /* charge FET already enabled */
	}

	/* Attempt to enable both FETs (chgoff=0, disoff=0), then re-check. */
	union fuel_gauge_prop_val fc = {.flags = 0};
	ret = fuel_gauge_set_prop(fg_dev, FUEL_GAUGE_FLAGS, fc);
	if (ret < 0) {
		LOG_ERR("Failed to enable charge FET: %d\n", ret);
		return ret;
	}

	ret = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_FLAGS, &v);
	if (ret < 0) {
		LOG_ERR("Failed to re-read FET status: %d\n", ret);
		return ret;
	}
	return (v.flags == 0) ? 0 : -EIO;
}

void update_com_console(struct max17320_data *fg_data,
			union fuel_gauge_prop_val fg_vals[10],
			union charger_propval chg_val, int state, struct sensor_value temp) {
	if (state == BMS_NORMAL)
		LOG_INF("\n---------- Normal State ----------\n");
	if (state == BMS_CHARGING)
		LOG_INF("\n---------- Charging State ----------\n");

	LOG_INF("SoC: %d %%\n", fg_vals[0].relative_state_of_charge);
	LOG_INF("Voltage: %d mV\n", fg_vals[1].voltage);
	//LOG_INF("Design Voltage: %d mV\n", fg_vals[9].design_volt);
	LOG_INF("Voltage Cell 1 %d mV \n", fg_data->voltage1);
	LOG_INF("Voltage Cell 2 %d mV \n", fg_data->voltage2);
	LOG_INF("Voltage Cell 3 %d mV \n", fg_data->voltage3);
	LOG_INF("Current: %d uA\n", fg_vals[2].current);
	LOG_INF("Temp FG: %d °C\n", fg_vals[3].temperature);
	LOG_INF("Remaining Capacity: %d mAh\n", fg_vals[4].remaining_capacity);
	LOG_INF("Full Capacity: %d mAh\n", fg_vals[5].full_charge_capacity);
	LOG_INF("Runtime to empty : %d mins \n", fg_vals[6].runtime_to_empty);
	LOG_INF("Time to full : %d mins \n", fg_vals[7].runtime_to_full);
	LOG_INF("Charging Current: %d uA\n", chg_val.const_charge_current_ua);
	LOG_INF("Tsensor: %d.%03d °C\n", temp.val1, temp.val2);
}
const char* charger_status_text(int status)
{
	switch(status){
		case CHARGER_STATUS_NOT_CHARGING:
			return "off";
		case CHARGER_STATUS_CHARGING:
			return "chrg";
		case CHARGER_STATUS_FULL:
			return "full";
		case CHARGER_STATUS_DISCHARGING:
			return "fault";
		case CHARGER_STATUS_UNKNOWN:
			return "?";
	}
	return "invalid";
}

void update_com_display(struct max17320_data *fg_data, union fuel_gauge_prop_val fg_vals[10],
	int chg_current_ua, int chg_status, const struct device *display_dev, int state, int count, struct sensor_value temp)
{
	char buf[256];
	const char *status;
	status = charger_status_text(chg_status);

	cfb_framebuffer_clear(display_dev, true);
	if (count == 0) {
		if (state == BMS_NORMAL){
			snprintf(buf, sizeof(buf), "Normal");
			cfb_print(display_dev, buf, 0,0);
		}
		if (state == BMS_CHARGING) {
			snprintf(buf, sizeof(buf), "Charging");
			cfb_print(display_dev, buf, 0,0);
		}
		snprintf(buf, sizeof(buf), "SoC: %d %%", fg_vals[0].relative_state_of_charge);
 		cfb_print(display_dev, buf, 0, 15);

 		snprintf(buf, sizeof(buf), "Vbat:%d.%01dV", fg_vals[1].voltage/1000, fg_vals[1].voltage%1000/100);
		cfb_print(display_dev, buf, 0, 30);

 		snprintf(buf, sizeof(buf), "C:%dmA", fg_vals[2].current/1000);
 		cfb_print(display_dev, buf, 0, 45);
 		cfb_framebuffer_finalize(display_dev);
	}

	if (count == 1) {
		snprintf(buf, sizeof(buf), "VC1:%dmV", fg_data->voltage1);
		cfb_print(display_dev, buf, 0,0);
		snprintf(buf, sizeof(buf), "VC2:%dmV", fg_data->voltage2);
 		cfb_print(display_dev, buf, 0, 15);
 		snprintf(buf, sizeof(buf), "VC3:%dmV", fg_data->voltage3);
		cfb_print(display_dev, buf, 0, 30);
 		cfb_framebuffer_finalize(display_dev);
	}
	if (count == 2) {
		snprintf(buf, sizeof(buf), "RCap:%dmAh", fg_vals[4].remaining_capacity);
 		cfb_print(display_dev, buf, 0,0);
 		snprintf(buf, sizeof(buf), "FCap:%dmAh", fg_vals[5].full_charge_capacity);
		cfb_print(display_dev, buf, 0, 15);
 		snprintf(buf, sizeof(buf), "TFG:%d°C", fg_vals[3].temperature);
 		cfb_print(display_dev, buf, 0, 30);
		snprintf(buf, sizeof(buf), "ChgStat:%s", status);
		cfb_print(display_dev, buf, 0, 45);
 		cfb_framebuffer_finalize(display_dev);
	}
	if (count == 3) {
		if (state == 1){
			snprintf(buf, sizeof(buf), "Normal 4/4");
			cfb_print(display_dev, buf, 0,0);
			snprintf(buf, sizeof(buf), "TTE %d mins", fg_vals[6].runtime_to_empty);
			cfb_print(display_dev, buf, 0, 15);
		}
		if (state == 2) {
			snprintf(buf, sizeof(buf), "Charging 4/4");
			cfb_print(display_dev, buf, 0,0);
			snprintf(buf, sizeof(buf), "TTF %d mins", fg_vals[7].runtime_to_full);
			cfb_print(display_dev, buf, 0, 15);
		}	
		snprintf(buf, sizeof(buf), "ChgC:%dmA", chg_current_ua/1000);
 		cfb_print(display_dev, buf, 0, 30);
		snprintf(buf, sizeof(buf),"Ts: %d.%03d °C\n", temp.val1, temp.val2);
		cfb_print(display_dev, buf, 0, 45);
 		cfb_framebuffer_finalize(display_dev);
	}
}

static int check_pfail() {
	int pfg_fault = gpio_pin_get_dt(&pfail_fg);
	if (pfg_fault == 1)
		LOG_ERR("Permanent Fuel Guage Failure detected\n");
	return pfg_fault;
}

static int check_pd_fault() {
	int pd_fault = gpio_pin_get_dt(&intmcu_1);
	if (pd_fault == 1)
		LOG_ERR("PowerDelivery chip fault detected\n");
	return pd_fault;
}

static int check_fg_fault() {
	int fg_fault = gpio_pin_get_dt(&intmcu_2);
	if (fg_fault == 1)
		LOG_ERR("Fuel Gauge chip fault detected\n");
	return fg_fault;
}

static int check_chg_fault() {
	int chg_fault = gpio_pin_get_dt(&intmcu_3);
	if (chg_fault == 1)
		LOG_ERR("Charger chip fault detected\n");
	return chg_fault;
}

static int check_powerpath_vbus_fault() {
	int f1_power_path = gpio_pin_get_dt(&fpp1);

	if (f1_power_path) {
		printf(
		    "Battery Power Path Fault detected, starting reset seq \n");
		adp5589_set_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A,
				      0X00); // drive DISABLEN1/DISABLEN2 low
		k_sleep(K_SECONDS(1));
		int check = gpio_pin_get_dt(&fpp1);
		if (!check) {
			printf("Reset Battery PowerPath successfull\n");
		}
	}

	return f1_power_path;
}

static int check_powerpath_battery_fault() {
	int f2_power_path = gpio_pin_get_dt(&fpp2);
	if (f2_power_path) {
		LOG_ERR("USB Fault detected, starting reset seq \n");
		adp5589_set_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A,
				      0X00); // drive DISABLEN1/DISABLEN2 low
		k_sleep(K_SECONDS(1));
		int check = gpio_pin_get_dt(&fpp2);
		if (!check) {
			LOG_INF("Reset USB PowerPath successfull\n");
		}
	}

	return f2_power_path;
}

static int check_power_faults() {
	int fault_power = 0;

	if (check_pfail() || check_chg_fault() || check_pd_fault() ||
	    check_fg_fault())
		fault_power = 1;

	return fault_power;
}

static int check_usbc()
{
	int16_t input_b;

	input_b = adp5589_get_pin_state(exp_dev, ADP5589_ADR_GPI_STATUS_B);
	if (input_b < 0) {
		return -1;
	}

	return ((input_b >> 0) & 0x01) == 1 ? 1 : 0;
}

void update_can_od()
{
	bms_status_t snap = bms_status;

	OD_RAM.x2060_batteryCellVoltages[0] = snap.batteryCellVoltages[0];
	OD_RAM.x2060_batteryCellVoltages[1] = snap.batteryCellVoltages[1];
	OD_RAM.x2060_batteryCellVoltages[2] = snap.batteryCellVoltages[2];
	OD_RAM.x2060_batteryCellVoltages[3] = snap.batteryCellVoltages[3];
	OD_RAM.x2071_current = snap.current;
	OD_RAM.x6000_batteryStatus = snap.batteryStatus;
	OD_RAM.x6001_chargerStatus = snap.chargerStatus;
	OD_RAM.x6010_temperature = snap.temperature;
	OD_RAM.x6052_ahReturnedDuringLastCharge = snap.ahReturnedDuringLastCharge;
	OD_RAM.x6060_batteryVoltage = snap.batteryVoltage;
	OD_RAM.x6080_chargerStateOfCharge = snap.chargerStateOfCharge;
	OD_RAM.x6081_batteryStateOfCharge = snap.batteryStateOfCharge;

	// Read from OD
	uint16_t req = OD_RAM.x6070_chargeCurrentRequested;

	bms_status.chargeCurrentRequested = req;

}
static void bms_config_entry(void *o) {
	int ret;
	const struct device *fg_dev = DEVICE_DT_GET(DT_NODELABEL(max17320));
	const struct device *chg_dev = DEVICE_DT_GET(DT_NODELABEL(max77961));
	const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

	// Configure peripherals
	ret = init_adp5589_config(exp_dev);
	if (ret < 0) {
		LOG_ERR("failed adp5589 config\n");
		return;
	}

	ret = init_mcupins_config();
	if (ret < 0) {
		LOG_ERR("failed MCU GPIOs config\n");
		return;
	}

	ret = fuel_gauge_get_props(fg_dev, props, fg_vals, ARRAY_SIZE(props));
	if (ret < 0) {
		LOG_ERR("Failed to get fuel gauge properties: %d\n", ret);
		return;
	}

	ret = charger_get_prop(chg_dev, CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA,
			       &chg_val_c);
	if (ret < 0) {
		LOG_ERR("Failed to get charge current: %d\n", ret);
		return;
	}

	ret = charger_get_prop(chg_dev, CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV,
			       &chg_val_v);
	if (ret < 0) {
		LOG_ERR("Failed to get charge voltage: %d\n", ret);
		return;
	}

	struct max17320_data *fg_data = (struct max17320_data *)fg_dev->data;
	update_configure_display(fg_data, fg_vals, chg_val_c, chg_val_v,
				display_dev);

	// Clear any pending button events
	button_clear_events();

	config_sub_state = CONFIG_STATE_WAIT_FOR_ENTRY;
}

static void bms_config_run(void *o) {
	static int timeout_counter = 0;
	static int refresh_counter = 0; // New counter for display refresh
	const struct device *fg_dev = DEVICE_DT_GET(DT_NODELABEL(max17320));
	const struct device *chg_dev = DEVICE_DT_GET(DT_NODELABEL(max77961));
	const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
	button_event_t btn_event;
	int ret;
	char buf[256];
	union fuel_gauge_prop_val fg_val_set;
	union charger_propval chg_val_set;
	union fuel_gauge_prop_val fet_config;

	switch (config_sub_state) {
	case CONFIG_STATE_WAIT_FOR_ENTRY:
		if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
			if (btn_event.button_id == BTN_UP &&
			    btn_event.event == BTN_EVENT_PRESSED) {
				LOG_INF("Starting hardware configuration\n");
				fet_config.flags = 1;
				ret = fuel_gauge_set_prop(
				    fg_dev, FUEL_GAUGE_FLAGS, fet_config);
				if (ret < 0) {
					LOG_ERR("Failed to disable CHG FET\n");
				}
				config_sub_state = CONFIG_STATE_SET_CAPACITY;
				timeout_counter = 0;
				refresh_counter = 0;
			}
		} else {
			timeout_counter++;
			if (timeout_counter >= 50) {
				LOG_INF("No configuration requested, using "
					"defaults\n");
				config_sub_state = CONFIG_STATE_FINISH;
				timeout_counter = 0;
				refresh_counter = 0; // Reset for next state
			}
		}
		break;

	case CONFIG_STATE_SET_CAPACITY:
		if (refresh_counter == 0) {
			cfb_framebuffer_clear(display_dev, true);
			snprintf(buf, sizeof(buf), "Configuring");
			cfb_print(display_dev, buf, 0, 0);
			ret = fuel_gauge_get_props(fg_dev, props, fg_vals,
							   ARRAY_SIZE(props));
			if (ret < 0) {
				LOG_ERR(
				    "Failed to get fuel gauge properties\n");
				break;
			}
			snprintf(buf, sizeof(buf), "FCap: %d",
				 fg_vals[5].full_charge_capacity);
			cfb_print(display_dev, buf, 0, 15);
			cfb_framebuffer_finalize(display_dev);
		}

		if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
			timeout_counter = 0;
			refresh_counter = 0;
			if (btn_event.button_id == BTN_UP &&
			    btn_event.event == BTN_EVENT_PRESSED) {
				fg_val_set.full_charge_capacity =
				    fg_vals[5].full_charge_capacity + 100;
				if (fg_val_set.full_charge_capacity > 16000) {
					fg_val_set.full_charge_capacity = 16000;
					snprintf(buf, sizeof(buf),
						 "Max: 16000 mAh");
					cfb_print(display_dev, buf, 0, 30);
					cfb_framebuffer_finalize(display_dev);

				}
				ret = fuel_gauge_set_prop(
				    fg_dev, FUEL_GAUGE_FULL_CHARGE_CAPACITY,
				    fg_val_set);
			} else if (btn_event.button_id == BTN_DOWN &&
				   btn_event.event == BTN_EVENT_PRESSED) {
				fg_val_set.full_charge_capacity =
				    fg_vals[5].full_charge_capacity - 100;
				if (fg_val_set.full_charge_capacity < 100) {
					fg_val_set.full_charge_capacity = 100;
					snprintf(buf, sizeof(buf),
						 "Min: 100 mAh");
					cfb_print(display_dev, buf, 0, 45);
					cfb_framebuffer_finalize(display_dev);
				}
				ret = fuel_gauge_set_prop(
				    fg_dev, FUEL_GAUGE_FULL_CHARGE_CAPACITY,
				    fg_val_set);
			} else if (btn_event.button_id == BTN_OK &&
				   btn_event.event == BTN_EVENT_PRESSED) {
				config_sub_state = CONFIG_STATE_SET_VOLTAGE;
				refresh_counter = 0; // Reset for next state
			}
		} else {
			timeout_counter++;
			if (timeout_counter >= 300) { 
				LOG_INF("Configuration timeout\n");
				config_sub_state = CONFIG_STATE_FINISH;
				timeout_counter = 0;
				refresh_counter = 0; // Reset for next state
			}
		}
		refresh_counter++;
		if (refresh_counter >= 10) { // 10 * 100ms = 1s
			refresh_counter = 0;
		}
		break;

	case CONFIG_STATE_SET_VOLTAGE:
		if (refresh_counter == 0) {
			cfb_framebuffer_clear(display_dev, true);
			snprintf(buf, sizeof(buf), "Configuring");
			cfb_print(display_dev, buf, 0, 0);
			ret = charger_get_prop(
			    chg_dev, CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV,
			    &chg_val_v);
			if (ret < 0) {
				LOG_ERR("Failed to get termination voltage\n");
				break;
			}
			snprintf(buf, sizeof(buf), "Vt: %d mV",
				 chg_val_v.const_charge_voltage_uv / 1000);
			cfb_print(display_dev, buf, 0, 15);
			cfb_framebuffer_finalize(display_dev);
		}

		if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
			timeout_counter = 0;
			refresh_counter = 0;
			if (btn_event.button_id == BTN_UP &&
			    btn_event.event == BTN_EVENT_PRESSED) {
				chg_val_set.const_charge_voltage_uv =
				    chg_val_v.const_charge_voltage_uv + 300000;
				if (chg_val_set.const_charge_voltage_uv >
				    12600000) {
					chg_val_set.const_charge_voltage_uv =
					    12600000;
					snprintf(buf, sizeof(buf),
						 "Max: 12.6V");
					cfb_print(display_dev, buf, 0, 30);
					cfb_framebuffer_finalize(display_dev);
				}
				ret = charger_set_prop(
				    chg_dev,
				    CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV,
				    &chg_val_set.const_charge_voltage_uv);
			} else if (btn_event.button_id == BTN_DOWN &&
				   btn_event.event == BTN_EVENT_PRESSED) {
				chg_val_set.const_charge_voltage_uv =
				    chg_val_v.const_charge_voltage_uv - 300000;
				if (chg_val_set.const_charge_voltage_uv <
				    10200000) {
					chg_val_set.const_charge_voltage_uv =
					    10200000;
					snprintf(buf, sizeof(buf),
						 "Min: 10.2V");
					cfb_print(display_dev, buf, 0, 30);
					cfb_framebuffer_finalize(display_dev);

				}
				ret = charger_set_prop(
				    chg_dev,
				    CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV,
				    &chg_val_set.const_charge_voltage_uv);
			} else if (btn_event.button_id == BTN_OK &&
				   btn_event.event == BTN_EVENT_PRESSED) {
				config_sub_state = CONFIG_STATE_SET_CURRENT;
				refresh_counter = 0; // Reset for next state
			}
		} else {
			timeout_counter++;
			if (timeout_counter >= 100) { // 10 seconds timeout
				LOG_INF("Configuration timeout\n");
				config_sub_state = CONFIG_STATE_FINISH;
				timeout_counter = 0;
				refresh_counter = 0; // Reset for next state
			}
		}
		refresh_counter++;
		if (refresh_counter >= 10) { // 10 * 100ms = 1s
			refresh_counter = 0;
		}
		break;

	case CONFIG_STATE_SET_CURRENT:
		if (refresh_counter == 0) {
			cfb_framebuffer_clear(display_dev, true);
			snprintf(buf, sizeof(buf), "Configuring");
			cfb_print(display_dev, buf, 0, 0);
			ret = charger_get_prop(
			    chg_dev, CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA,
			    &chg_val_c);
			if (ret < 0) {
				LOG_ERR("Failed to get charge current\n");
				break;
			}
			snprintf(buf, sizeof(buf), "ChgC:%d mA",
				 chg_val_c.const_charge_current_ua / 1000);
			cfb_print(display_dev, buf, 0, 15);
			cfb_framebuffer_finalize(display_dev);
		}

		if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
			timeout_counter = 0;
			refresh_counter = 0;
			if (btn_event.button_id == BTN_UP &&
			    btn_event.event == BTN_EVENT_PRESSED) {
				chg_val_set.const_charge_current_ua =
				    chg_val_c.const_charge_current_ua + 100000;
				if (chg_val_set.const_charge_current_ua >
				    3000000) {
					chg_val_set.const_charge_current_ua =
					    3000000;
					snprintf(buf, sizeof(buf), "Max: 3A");
					cfb_print(display_dev, buf, 0, 30);
					cfb_framebuffer_finalize(display_dev);
				}
				ret = charger_set_prop(
				    chg_dev,
				    CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA,
				    &chg_val_set.const_charge_current_ua);
			} else if (btn_event.button_id == BTN_DOWN &&
				   btn_event.event == BTN_EVENT_PRESSED) {
				chg_val_set.const_charge_current_ua =
				    chg_val_c.const_charge_current_ua - 100000;
				if (chg_val_set.const_charge_current_ua <
				    100000) {
					chg_val_set.const_charge_current_ua =
					    100000;
					snprintf(buf, sizeof(buf),
						 "Min: 100mA");
					cfb_print(display_dev, buf, 0, 30);
					cfb_framebuffer_finalize(display_dev);
				}
				ret = charger_set_prop(
				    chg_dev,
				    CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA,
				    &chg_val_set.const_charge_current_ua);
			} else if (btn_event.button_id == BTN_OK &&
				   btn_event.event == BTN_EVENT_PRESSED) {
				config_sub_state = CONFIG_STATE_FINISH;
				refresh_counter = 0; // Reset for next state
			}
		} else {
			timeout_counter++;
			if (timeout_counter >= 300) { // 30 seconds timeout
				LOG_INF("Configuration timeout\n");
				config_sub_state = CONFIG_STATE_FINISH;
				timeout_counter = 0;
				refresh_counter = 0; // Reset for next state
			}
		}
		refresh_counter++;
		if (refresh_counter >= 10) { // 10 * 100ms = 1s
			refresh_counter = 0;
		}
		break;

	case CONFIG_STATE_FINISH:
		fet_config.flags = 0;
		ret = fuel_gauge_set_prop(fg_dev, FUEL_GAUGE_FLAGS, fet_config);
		if (ret < 0) {
			LOG_ERR("Failed to enable CHG FET\n");
		}
		struct max17320_data *fg_data =
		    (struct max17320_data *)fg_dev->data;
		update_configure_display(fg_data, fg_vals, chg_val_c, chg_val_v,
					 display_dev);
		k_sleep(K_SECONDS(2));

		int usbc_unplugged = check_usbc();
		if (usbc_unplugged == 1) {
			smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_NORMAL]);
		} else {
			smf_set_state(SMF_CTX(&s_obj),
				      &bms_states[BMS_CHARGING]);
		}
		config_sub_state = CONFIG_STATE_INIT; // Reset for next time
		refresh_counter = 0; // Reset for next time
		break;

	default:
		config_sub_state = CONFIG_STATE_INIT;
		refresh_counter = 0; // Reset for next time
		break;
	}
}

static void bms_normal_entry(void *o)
{
	static bool first_entry = true;

	if (first_entry) {
		int16_t en_5v_status = adp5589_get_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_B);
		if (en_5v_status >= 0 && ((en_5v_status >> 3) & 0x01) != 1) {
			adp5589_set_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_B,
					      0x08);
			LOG_INF("CAN Transceiver powered up\n");
		}
		first_entry = false;
	}
}

static void bms_normal_run(void *o) {
	static int display_page = 0;
	static int refresh_counter = 0;
	const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
	const struct device *fg_dev = DEVICE_DT_GET(DT_NODELABEL(max17320));
	const struct device *chg_dev = DEVICE_DT_GET(DT_NODELABEL(max77961));
	union charger_propval chg_val;
	int ret = 0;

	/* non-blocking button check - don't hold locks while waiting */
	button_event_t btn_event;
	if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
		if (btn_event.button_id == BTN_SHUTDOWN && btn_event.event == BTN_EVENT_PRESSED) {
			smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_SHUTDOWN]);
		} else if (btn_event.button_id == BTN_OK && btn_event.event == BTN_EVENT_PRESSED) {
			smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_CONFIG]);
		} else if (btn_event.button_id == BTN_DOWN && btn_event.event == BTN_EVENT_PRESSED) {
			display_page++;
			if (display_page >= 4)
				display_page = 0;
			refresh_counter = 0;
		}
	}

	struct sensor_value temp = {0, 0};

	if (refresh_counter == 0) {
		ret = fuel_gauge_get_props(fg_dev, props, fg_vals,
						   ARRAY_SIZE(props));
		if (ret < 0) {
			LOG_ERR("Failed to get fuel gauge properties: %d\n",
				ret);
			consecutive_fg_errors++;
			if (ret == -ETIMEDOUT) {
				i2c_cooldown = i2c_cooldown_base;
				if (i2c_cooldown_base < 20) {
					i2c_cooldown_base *= 2;
				}
			}
			if (consecutive_fg_errors >= MAX_FG_ERRORS) {
				enter_error(BMS_ERR_FG_TIMEOUT);
				return;
			}
		} else {
			consecutive_fg_errors = 0;
			i2c_cooldown_base = 5;
			i2c_recovery_failures = 0;
			temp = check_temp_sensor();
			struct max17320_data *fg_data =
			    (struct max17320_data *)fg_dev->data;
			ret = charger_get_prop(chg_dev, CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA, &chg_val);
			if (ret < 0) {
				LOG_ERR("Failed to get charger current: %d\n", ret);
			} else {
				union charger_propval chg_val_status;
				ret = charger_get_prop(chg_dev, CHARGER_PROP_STATUS, &chg_val_status);
				if (ret < 0) {
					LOG_ERR("Failed to get charger status: %d\n", ret);
				} else {
					bms_status.chargerStatus = chg_val_status.status;
					update_com_console(fg_data, fg_vals, chg_val, BMS_NORMAL, temp);
					if (charge_fault_active) {
						show_message_display(display_dev, "CHG FET FAULT", "Cannot charge", "Retrying...");
					} else {
						update_com_display(fg_data, fg_vals, chg_val.const_charge_current_ua, chg_val_status.status, display_dev, BMS_NORMAL, display_page, temp);
					}
				}
			}
		}

		int usbc_unplugged = check_usbc();

		if (usbc_unplugged < 0) {
			consecutive_expander_errors++;
			LOG_WRN("GPIO expander read failed (%d/%d)",
				consecutive_expander_errors, MAX_EXPANDER_ERRORS);
			if (consecutive_expander_errors >= MAX_EXPANDER_ERRORS) {
				enter_error(BMS_ERR_EXPANDER);
				return;
			}
		} else {
			consecutive_expander_errors = 0;

			if (!powerpath_battery_enabled) {
				int fault1 = gpio_pin_get_dt(&fpp1);
				int fault2 = gpio_pin_get_dt(&fpp2);
				if (usbc_unplugged == 1 && !fault1 && !fault2) {
					powerpath_battery_enabled = true;
					adp5589_set_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A,
							      0x20);
					LOG_INF("Power Path enabled\n");
				}
			}

			if (usbc_unplugged == 0) {
				/* Only enter CHARGING if the pack can actually
				 * charge (charge FET enabled). Otherwise warn on
				 * the OLED and stay here, retrying next cycle. */
				if (ensure_charge_fet_enabled(fg_dev) == 0) {
					charge_fault_active = false;
					display_page = 0;
					smf_set_state(SMF_CTX(&s_obj),
						      &bms_states[BMS_CHARGING]);
				} else {
					charge_fault_active = true;
					LOG_ERR("Charge FET not enabled, cannot charge\n");
				}
			}
		}
	}

	refresh_counter++;
	if (refresh_counter >= 10) {
		refresh_counter = 0;
	}
}

static void bms_charging_entry(void *o)
{
	charging_confirm_counter = 0;
	not_charging_warn = false;
}

static void bms_charging_run(void *o) {
	static int display_page = 0;
	static int refresh_counter = 0;
	static bool can_transceiver_checked = false;
	union charger_propval chg_val;
	const struct device *fg_dev = DEVICE_DT_GET(DT_NODELABEL(max17320));
	const struct device *chg_dev = DEVICE_DT_GET(DT_NODELABEL(max77961));
	const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
	int ret = 0;

	/* non-blocking button check - don't hold locks while waiting */
	button_event_t btn_event;
	if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
		if (btn_event.button_id == BTN_SHUTDOWN && btn_event.event == BTN_EVENT_PRESSED) {
			smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_SHUTDOWN]);
		} else if (btn_event.button_id == BTN_OK && btn_event.event == BTN_EVENT_PRESSED) {
			smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_CONFIG]);
		} else if (btn_event.button_id == BTN_DOWN && btn_event.event == BTN_EVENT_PRESSED) {
			display_page++;
			if (display_page >= 4)
				display_page = 0;
			refresh_counter = 0;
		}
	}

	struct sensor_value temp = {0, 0};

	if (refresh_counter == 0) {
		/* Detect USB-C unplug BEFORE rendering, so we never draw a
		 * charging frame (e.g. "USB-C CONNECTED / Waiting for current")
		 * on the way out to NORMAL. */
		int usbc_unplugged = check_usbc();
		if (usbc_unplugged < 0) {
			consecutive_expander_errors++;
			LOG_WRN("GPIO expander read failed in charging (%d/%d)",
				consecutive_expander_errors, MAX_EXPANDER_ERRORS);
			if (consecutive_expander_errors >= MAX_EXPANDER_ERRORS) {
				enter_error(BMS_ERR_EXPANDER);
				return;
			}
		} else {
			consecutive_expander_errors = 0;
			if (usbc_unplugged == 1) {
				display_page = 0;
				smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_NORMAL]);
				powerpath_vbus_enabled = false;
				powerpath_battery_enabled = false;
				can_transceiver_checked = false;
				return;
			}
		}

		ret = fuel_gauge_get_props(fg_dev, props, fg_vals,
						   ARRAY_SIZE(props));
		if (ret < 0) {
			LOG_ERR("Failed to get properties: %d\n", ret);
			consecutive_fg_errors++;
			if (ret == -ETIMEDOUT) {
				i2c_cooldown = i2c_cooldown_base;
				if (i2c_cooldown_base < 20) {
					i2c_cooldown_base *= 2;
				}
			}
			if (consecutive_fg_errors >= MAX_FG_ERRORS) {
				enter_error(BMS_ERR_FG_TIMEOUT);
				return;
			}
		} else {
			consecutive_fg_errors = 0;
			i2c_cooldown_base = 5;
			i2c_recovery_failures = 0;
			temp = check_temp_sensor();
			struct max17320_data *fg_data = (struct max17320_data *)fg_dev->data;
			ret = charger_get_prop(chg_dev, CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA, &chg_val);
			if (ret < 0) {
				LOG_ERR("Failed to get charger current: %d\n", ret);
			} else {
				union charger_propval chg_val_status;
				ret = charger_get_prop(chg_dev, CHARGER_PROP_STATUS, &chg_val_status);
				if (ret < 0) {
					LOG_ERR("Failed to get charger status: %d\n", ret);
				} else {
					bms_status.chargerStatus = chg_val_status.status;
					/* Charging is confirmed ONLY by actual
					 * into-battery current. The charger status is
					 * never trusted to declare charging (it can
					 * report CHARGING while a protector/FET blocks
					 * current) - it is used only to word the
					 * not-charging message (full vs fault). The
					 * charging data screen is shown ONLY when real
					 * current is flowing, so we never display a
					 * false "charging" state. */
					int st = chg_val_status.status;
					bool actually_charging = (fg_vals[2].current > 0);

					update_com_console(fg_data, fg_vals, chg_val, BMS_CHARGING, temp);

					if (actually_charging) {
						charging_confirm_counter = 0;
						not_charging_warn = false;
						update_com_display(fg_data, fg_vals, chg_val.const_charge_current_ua, chg_val_status.status, display_dev, BMS_CHARGING, display_page, temp);
					} else if (st == CHARGER_STATUS_FULL) {
						/* Battery full, USB-C still plugged: zero
						 * current is expected, not a fault. */
						charging_confirm_counter = 0;
						not_charging_warn = false;
						show_message_display(display_dev, "CHARGED", "Battery full", "");
					} else if (charging_confirm_counter < CHARGING_CONFIRM_GRACE) {
						/* Grace period: current may still be
						 * ramping up right after USB-C connect. */
						charging_confirm_counter++;
						show_message_display(display_dev, "USB-C CONNECTED", "Waiting for current", "");
					} else {
						/* USB-C plugged but no current is flowing
						 * into the battery - real fault. */
						not_charging_warn = true;
						show_message_display(display_dev, "NOT CHARGING", "No current flow", "Check charger/FET");
					}
				}
			}
		}
	}

	/* check CAN transceiver on first iteration */
	if (!can_transceiver_checked) {
		int16_t en_5v_status =
		    adp5589_get_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_B);
		if (en_5v_status >= 0 && ((en_5v_status >> 3) & 0x01) != 1) {
			adp5589_set_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_B,
					      0x08);
			LOG_INF("CAN Transceiver powered up \n");
		}
		can_transceiver_checked = true;
	}

	/* Enable the VBUS powerpath on refresh iterations. USB-C unplug and
	 * expander errors are already handled at the top of this function. */
	if (refresh_counter == 0) {
		if (!powerpath_vbus_enabled) {
			int16_t input_a =
			    adp5589_get_pin_state(exp_dev, ADP5589_ADR_GPI_STATUS_A);
			if (input_a >= 0) {
				int vbus_good = (input_a >> 0) & 0x01;

				if (vbus_good == 1) {
					LOG_INF("VBus PowerPath enabled\n");
					adp5589_set_register_value(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A,
								   0x30);
					powerpath_vbus_enabled = true;
				}
			}
		}
	}

	refresh_counter++;
	if (refresh_counter >= 10) {
		refresh_counter = 0;
	}
}

static void bms_low_power_entry(void *o)
{
	uint8_t mask_a = 0x30;
	uint8_t mask_b = 0x08;

	int16_t reg_a = adp5589_get_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A);
	if (reg_a >= 0) {
		uint8_t low_state = ((uint8_t)reg_a) & ~mask_a;
		adp5589_set_register_value(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A,
					   low_state);
	}
	int16_t reg_b = adp5589_get_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_B);
	if (reg_b >= 0) {
		uint8_t low_state = ((uint8_t)reg_b) & ~mask_b;
		adp5589_set_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_B, low_state);
	}
}

static void bms_low_power_run(void *o)
{
	/* The logic for this state is handled on entry. */
}

static void bms_shutdown_entry(void *o)
{
	uint8_t mask_a = 0x30;
	char buf[256];
	const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

	powerpath_battery_enabled = false;
	powerpath_vbus_enabled = false;

	int16_t reg_a = adp5589_get_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A);
	if (reg_a >= 0) {
		uint8_t low_state = ((uint8_t)reg_a) & ~mask_a;
		adp5589_set_register_value(exp_dev, ADP5589_ADR_GPO_DATA_OUT_A,
					   low_state);
	}

	cfb_framebuffer_clear(display_dev, true);
	snprintf(buf, sizeof(buf), "Shutdown");
	cfb_print(display_dev, buf, 0, 0);
	snprintf(buf, sizeof(buf), "Press Btn 1");
	cfb_print(display_dev, buf, 0, 15);
	snprintf(buf, sizeof(buf), "to wake up");
	cfb_print(display_dev, buf, 0, 30);
	cfb_framebuffer_finalize(display_dev);

	button_clear_events();
}

static void bms_shutdown_run(void *o) {
	button_event_t btn_event;
	if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
		if (btn_event.button_id == BTN_SHUTDOWN && btn_event.event == BTN_EVENT_PRESSED) {
			smf_set_state(SMF_CTX(&s_obj), &bms_states[BMS_NORMAL]);
		}
	}
}

/* Display the latched fault on the OLED and drain any stale button presses so a
 * queued event doesn't reboot us immediately. */
static void bms_error_entry(void *o)
{
	const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

	button_clear_events();
	show_message_display(display_dev, "SYSTEM ERROR",
			     error_text(g_error_code), "Press UP to reset");
}

/* Hold the error on screen until the operator presses BTN_UP, then reboot.
 * Non-blocking: returns each iteration so the main loop keeps feeding the
 * hardware and software watchdogs while the message is displayed. */
static void bms_error_run(void *o)
{
	button_event_t btn_event;

	if (button_wait_event(&btn_event, K_NO_WAIT) == 0) {
		if (btn_event.button_id == BTN_UP &&
		    btn_event.event == BTN_EVENT_PRESSED) {
			LOG_INF("Error acknowledged, rebooting\n");
			k_sleep(K_MSEC(100));
			sys_reboot(SYS_REBOOT_COLD);
		}
	}
}

int lock_mutex_helper(void) {
	if (k_mutex_lock(&i2c_mutex, K_MSEC(200)) < 0) {
		return -1;
	}

	if (k_mutex_lock(&co.CO->CANmodule->od_mutex, K_MSEC(200)) < 0) {
		k_mutex_unlock(&i2c_mutex);
		return -1;
	}

	if (k_mutex_lock(&co.CO->CANmodule->can_send_mutex, K_MSEC(200)) < 0) {
		k_mutex_unlock(&co.CO->CANmodule->od_mutex);
		k_mutex_unlock(&i2c_mutex);
		return -1;
	}

	if (k_mutex_lock(&co.CO->CANmodule->emcy_mutex, K_MSEC(200)) < 0) {
		k_mutex_unlock(&co.CO->CANmodule->can_send_mutex);
		k_mutex_unlock(&co.CO->CANmodule->od_mutex);
		k_mutex_unlock(&i2c_mutex);
		return -1;
	}

	return 0;
}

void unlock_mutex_helper(void) {
	k_mutex_unlock(&co.CO->CANmodule->emcy_mutex);
	k_mutex_unlock(&co.CO->CANmodule->can_send_mutex);
	k_mutex_unlock(&co.CO->CANmodule->od_mutex);
	k_mutex_unlock(&i2c_mutex);
}

int main(void) {

	int32_t ret;
	/* Initialize mutex for I2C access */
	k_mutex_init(&i2c_mutex);

	int count = 0;
	Wrap_MXC_SYS_ClockCalibrate(MXC_SYS_CLOCK_IPO);
	k_sleep(K_MSEC(100));
	struct adp5589_init_param init_param = {
	    .i2c_dt_spec = I2C_DT_SPEC_GET(DT_NODELABEL(adp5589))};

	if (adp5589_init(&exp_dev, init_param) != 0) {
		LOG_ERR("failed to init GPIO expander dev\n");
	}

	ret = button_manager_init(exp_dev, &i2c_mutex);
	if (ret < 0) {
		LOG_ERR("Failed to initialize button manager: %d\n", ret);
		return -1;
	}

	k_mutex_lock(&i2c_mutex,K_FOREVER);
	adp5589_gpio_direction(exp_dev, ADP5589_ADR_GPIO_DIRECTION_B, 0x08);
	adp5589_set_pin_state(exp_dev, ADP5589_ADR_GPO_DATA_OUT_B, 0x08);
	k_mutex_unlock(&i2c_mutex);

	device_init(can);

	memset(&co, 0, sizeof(co));

	// Initialize CAN, CANopen
	co.can_dev = can;
	co.node_id = 2; // TODO: reconfigure
	co.bitrate = CAN_BITRATE;
	co.nmt_control = CO_NMT_STARTUP_TO_OPERATIONAL;
	ret = canopen_init(&co);

	if (ret < 0) {
		LOG_ERR("CANopenNode init failed");
		return -1;
	}

	const struct device *fuel_gauge_dev =
	    DEVICE_DT_GET(DT_NODELABEL(max17320));

	if (!device_is_ready(fuel_gauge_dev)) {
		LOG_ERR("Device %s is not ready\n", fuel_gauge_dev->name);
		return -ENODEV;
	}
	const struct device *charger_dev =
	    DEVICE_DT_GET(DT_NODELABEL(max77961));

	if (!device_is_ready(charger_dev)) {
		LOG_ERR("Device %s is not ready\n", charger_dev->name);
		return -ENODEV;
	}

	init_display_config();
	adp5589_gpio_direction(exp_dev, ADP5589_ADR_GPIO_DIRECTION_A, 0x30);  // DISABLE1/2 outputs
	adp5589_gpio_direction(exp_dev, ADP5589_ADR_GPIO_DIRECTION_C, 0x07);

	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	const struct device *wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	int wdt_channel_id = -1;
	if (device_is_ready(wdt_dev)) {
		struct wdt_timeout_cfg wdt_cfg = {
			.window.min = 0,
			.window.max = 10000,
			.callback = NULL,
			.flags = WDT_FLAG_RESET_SOC,
		};
		wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
		if (wdt_channel_id >= 0) {
			wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
			LOG_INF("Watchdog started (10s timeout)");
		} else {
			LOG_ERR("Watchdog install failed: %d", wdt_channel_id);
		}
	} else {
		LOG_ERR("Watchdog device not ready");
	}


	smf_set_initial(SMF_CTX(&s_obj), &bms_states[BMS_NORMAL]);

	k_timer_start(&sw_wdt_timer, K_SECONDS(30), K_SECONDS(5));

	static uint32_t iteration_count = 0;
	static uint32_t last_progress_log = 0;

	while (true) {
		iteration_count++;
		main_loop_heartbeat = iteration_count;
		uint32_t loop_start = k_uptime_get_32();

		k_yield();

		if (lock_mutex_helper() < 0) {
			uint32_t now = k_uptime_get_32();
			consecutive_lock_failures++;

			if (now - lock_fail_window_start > LOCK_FAIL_WINDOW_MS) {
				lock_fail_window_start = now;
				lock_fail_window_count = 0;
			}
			lock_fail_window_count++;

			LOG_WRN("Lock fail %d/%d (window %d/%d), iter %u",
				consecutive_lock_failures, MAX_LOCK_FAILURES,
				lock_fail_window_count, LOCK_FAIL_WINDOW_MAX,
				iteration_count);

			if (consecutive_lock_failures >= MAX_LOCK_FAILURES ||
			    lock_fail_window_count >= LOCK_FAIL_WINDOW_MAX) {
				LOG_ERR("Persistent lock failure, rebooting");
				k_sleep(K_MSEC(100));
				sys_reboot(SYS_REBOOT_COLD);
			}
			if (wdt_channel_id >= 0) {
				wdt_feed(wdt_dev, wdt_channel_id);
			}
			k_yield();
			continue;
		}
		consecutive_lock_failures = 0;

		if (i2c_cooldown > 0) {
			i2c_cooldown--;
			if (i2c_cooldown == 0) {
				temp_sensor_errors = 0;
				i2c_recover_bus(i2c_dev);
				i2c_recovery_failures++;
				LOG_INF("I2C bus recovery attempted (%d/%d, backoff=%d)",
					i2c_recovery_failures, 5, i2c_cooldown_base);
				if (i2c_recovery_failures >= 5) {
					LOG_ERR("I2C bus unrecoverable, rebooting");
					unlock_mutex_helper();
					k_sleep(K_MSEC(100));
					sys_reboot(SYS_REBOOT_COLD);
				}
			}
			unlock_mutex_helper();
			if (wdt_channel_id >= 0) {
				wdt_feed(wdt_dev, wdt_channel_id);
			}
			k_sleep(K_MSEC(200));
			continue;
		}

		button_poll();

		ret = smf_run_state(SMF_CTX(&s_obj));

		if (ret) {
			LOG_ERR("State machine returned error %d", ret);
			unlock_mutex_helper();
			break;
		}

		struct max17320_data *fg_data =
		    (struct max17320_data *)fuel_gauge_dev->data;
		update_bms_status_from_max17320(fg_data, &bms_status);

		update_can_od();
		unlock_mutex_helper();

		uint32_t loop_end = k_uptime_get_32();

		if (loop_end - last_progress_log > 60000) {
			LOG_INF("Loop %u: total=%ums, CAN_err=0x%04x",
				iteration_count,
				loop_end - loop_start,
				co.CO->CANmodule->CANerrorStatus);
			last_progress_log = loop_end;
		}

		if (loop_end - loop_start > 2000) {
			LOG_WRN("Slow iteration %u: %ums",
				iteration_count,
				loop_end - loop_start);
		}

		if (wdt_channel_id >= 0) {
			wdt_feed(wdt_dev, wdt_channel_id);
		}

		k_yield();
		k_sleep(K_MSEC(90));
	}
	return 0;
}
