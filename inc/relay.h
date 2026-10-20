#ifndef RELAY_H
#define RELAY_H

#include "board_config.h"
#include <stdint.h>

typedef enum {
    RELAY_OK = 0,
    RELAY_ERROR,
    RELAY_INVALID_CHANNEL
} relay_status_t;

void relays_init(void);
void relay_set(uint8_t channel, uint8_t state);
uint8_t relay_get(uint8_t channel);
void relays_set_all(uint8_t mask);
uint8_t relays_get_all(void);
void relay_toggle(uint8_t channel);
void relay_led_update(uint8_t channel, uint8_t state);

#endif /* RELAY_H */
