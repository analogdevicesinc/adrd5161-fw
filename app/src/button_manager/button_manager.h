/* button_manager.h - Button Manager API for ADP5589 */

#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#include <zephyr/kernel.h>
#include "drivers/adp5589.h"

/* Button IDs matching ADP5589 Port B pins */

/* Button definitions */
#define BTN_SHUTDOWN       4  // Port B, bit 4
#define BTN_UP             5  // Port B, bit 5 (also used to trigger HW GUI CONFIG in config state)
#define BTN_DOWN           6  // Port B, bit 6 (also used as NEXT to navigate display)
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

/**
 * @brief Initialize the button manager
 * 
 * @param dev Pointer to initialized ADP5589 device
 * @return 0 on success, negative error code on failure
 */
int button_manager_init(struct adp5589_dev *dev, struct k_mutex *mut);

/**
 * @brief Wait for any button event with timeout
 * 
 * @param event Pointer to store the button event
 * @param timeout Timeout duration (use K_FOREVER to wait indefinitely)
 * @return 0 on success (event received), -EAGAIN on timeout, other negative on error
 */
int button_wait_event(button_event_t *event, k_timeout_t timeout);

/**
 * @brief Wait for a specific button press event
 * 
 * @param button_id Button ID to wait for (BTN_UP, BTN_DOWN, etc.)
 * @param timeout Timeout duration
 * @return 0 on success, -EAGAIN on timeout, other negative on error
 */
int button_wait_press(uint8_t button_id, k_timeout_t timeout);

/**
 * @brief Check if a button is currently pressed (non-blocking)
 * 
 * @param button_id Button ID to check
 * @return true if button is pressed, false otherwise
 */
bool button_is_pressed(uint8_t button_id);

/**
 * @brief Clear all pending button events from the queue
 */
void button_clear_events(void);

/**
 * @brief Get the current state of a button directly
 * 
 * @param button_id Button ID to query
 * @return 1 if pressed, 0 if not pressed
 */
uint8_t button_get_state(uint8_t button_id);

/**
 * @brief Lock the I2C mutex for shared ADP5589 access
 */
void button_manager_i2c_lock(void);

/**
 * @brief Unlock the I2C mutex for shared ADP5589 access
 */
void button_manager_i2c_unlock(void);

/**
 * @brief Poll buttons from main loop (caller must hold i2c_mutex)
 */
void button_poll(void);

#endif /* BUTTON_MANAGER_H */