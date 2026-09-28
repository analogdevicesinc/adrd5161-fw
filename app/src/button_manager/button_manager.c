/* Button Manager for ADP5589 GPIO Expander */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "drivers/adp5589.h"

LOG_MODULE_REGISTER(button_mgr, LOG_LEVEL_ERR);

/* Button definitions */
#define BTN_SHUTDOWN       4  // Port B, bit 4
#define BTN_UP             5  // Port B, bit 5
#define BTN_DOWN           6  // Port B, bit 6
#define BTN_OK             7  // Port B, bit 7

/* Button event types */
typedef enum {
	BTN_EVENT_NONE = 0,
	BTN_EVENT_PRESSED,
	BTN_EVENT_RELEASED,
	BTN_EVENT_HELD
} button_event_type_t;

/* Button event structure */
typedef struct {
	uint8_t button_id;
	button_event_type_t event;
	uint32_t timestamp;
} button_event_t;

/* Button state structure */
typedef struct {
	uint8_t current_state;
	uint8_t last_state;
	uint32_t press_time;
	uint32_t last_change_time;
	bool debounced;
} button_state_t;

/* Configuration */
#define DEBOUNCE_TIME_MS      50    // Debounce period
#define LONG_PRESS_TIME_MS    1000  // Time for long press
#define BUTTON_QUEUE_SIZE     16    // Event queue size
#define BUTTON_POLL_PERIOD_MS 50    // How often to poll buttons

/* Global button manager state */
static struct {
	struct adp5589_dev *dev;
	button_state_t buttons[8];  // there are 4 USR buttons connected to MCU, but 8 bits in port B
	struct k_msgq event_queue;
	struct k_mutex *i2c_mutex;
	bool initialized;
} btn_mgr;

/* Message queue buffer */
static char __aligned(4) event_queue_buffer[BUTTON_QUEUE_SIZE * sizeof(button_event_t)];
extern int lock_mutex_helper(void);
extern void unlock_mutex_helper(void);
/* Initialize button manager */
int button_manager_init(struct adp5589_dev *dev, struct k_mutex *mut)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	btn_mgr.dev = dev;
	btn_mgr.initialized = true;

	/* Initialize message queue for button events */
	k_msgq_init(&btn_mgr.event_queue, event_queue_buffer, sizeof(button_event_t), BUTTON_QUEUE_SIZE);

	btn_mgr.i2c_mutex = mut;

	/* Initialize button states */
	for (int i = 0; i < 8; i++) {
		btn_mgr.buttons[i].current_state = 0;
		btn_mgr.buttons[i].last_state = 0;
		btn_mgr.buttons[i].press_time = 0;
		btn_mgr.buttons[i].last_change_time = 0;
		btn_mgr.buttons[i].debounced = true;
	}

	LOG_INF("Button manager initialized");
	return 0;
}

/* Process a single button state */
static void process_button(uint8_t btn_id, uint8_t new_state, uint32_t current_time)
{
	button_state_t *btn = &btn_mgr.buttons[btn_id];
	button_event_t event;
    
	/* Check if state changed */
	if (new_state != btn->current_state) {
		btn->current_state = new_state;
		btn->last_change_time = current_time;
		btn->debounced = false;
		return;  // Wait for debounce
	}
    
	/* Check if debounce period elapsed */
	if (!btn->debounced && (current_time - btn->last_change_time) >= DEBOUNCE_TIME_MS) {
		btn->debounced = true;
	
		/* Generate events */
		if (new_state && !btn->last_state) {
	    		/* Button pressed */
	    		btn->press_time = current_time;
	    		event.button_id = btn_id;
			event.event = BTN_EVENT_PRESSED;
			event.timestamp = current_time;
			k_msgq_put(&btn_mgr.event_queue, &event, K_NO_WAIT);
			LOG_DBG("Button %d pressed", btn_id);
		}
		else if (!new_state && btn->last_state) {
	    		/* Button released */
	    		event.button_id = btn_id;
	    		event.event = BTN_EVENT_RELEASED;
	    		event.timestamp = current_time;
	    		k_msgq_put(&btn_mgr.event_queue, &event, K_NO_WAIT);
			LOG_DBG("Button %d released", btn_id);
	}
	
	btn->last_state = new_state;
	}
    
	/* Check for long press */
	if (btn->debounced && new_state && btn->last_state && (current_time - btn->press_time) >= LONG_PRESS_TIME_MS) {
		/* Generate long press event once */
		if ((current_time - btn->press_time) < (LONG_PRESS_TIME_MS + BUTTON_POLL_PERIOD_MS)) {
	    		event.button_id = btn_id;
	    		event.event = BTN_EVENT_HELD;
	    		event.timestamp = current_time;
			k_msgq_put(&btn_mgr.event_queue, &event, K_NO_WAIT);
			LOG_DBG("Button %d held", btn_id);
		}
	}
}

int button_wait_event(button_event_t *event, k_timeout_t timeout)
{
	if (!btn_mgr.initialized) {
		return -ENODEV;
	}
    
	return k_msgq_get(&btn_mgr.event_queue, event, timeout);
}

int button_wait_press(uint8_t button_id, k_timeout_t timeout)
{
	button_event_t event;
	uint32_t start_time = k_uptime_get_32();
	int64_t timeout_ms = k_ticks_to_ms_floor64(timeout.ticks);
    
	while (1) {
		/* Calculate remaining timeout */
		int64_t elapsed = k_uptime_get_32() - start_time;
		if (timeout_ms != SYS_FOREVER_MS && elapsed >= timeout_ms) {
			return -EAGAIN;  // Timeout
		}
	
		k_timeout_t remaining = (timeout_ms == SYS_FOREVER_MS) ? K_FOREVER : K_MSEC(timeout_ms - elapsed);
	
		int ret = k_msgq_get(&btn_mgr.event_queue, &event, remaining);
		if (ret != 0) {
			return ret;  // Timeout or error
		}
	
		/* Check if this is the button we want */
		if (event.button_id == button_id && event.event == BTN_EVENT_PRESSED) {
			return 0;  // Success
		}
		/* If not, discard and continue waiting */
	}
}

bool button_is_pressed(uint8_t button_id)
{
	if (!btn_mgr.initialized || button_id >= 8) {
		return false;
	}
    
	return btn_mgr.buttons[button_id].current_state != 0;
}

void button_clear_events(void)
{
	button_event_t dummy;
	while (k_msgq_get(&btn_mgr.event_queue, &dummy, K_NO_WAIT) == 0) {
		/* Drain queue */
	}
}

uint8_t button_get_state(uint8_t button_id)
{
	if (!btn_mgr.initialized || button_id >= 8) {
		return 0;
	}

	return btn_mgr.buttons[button_id].current_state;
}

void button_manager_i2c_lock(void)
{
	lock_mutex_helper();
}

void button_manager_i2c_unlock(void)
{
	unlock_mutex_helper();
}

void button_poll(void)
{
	int16_t button_register;
	uint32_t current_time;

	if (!btn_mgr.initialized) {
		return;
	}

	button_register = adp5589_get_pin_state(btn_mgr.dev, ADP5589_ADR_GPI_STATUS_B);
	if (button_register < 0) {
		return;
	}

	current_time = k_uptime_get_32();

	for (int i = 0; i < 8; i++) {
		uint8_t state = (button_register >> i) & 0x01;
		process_button(i, state, current_time);
	}
}