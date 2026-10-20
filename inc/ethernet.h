#ifndef ETHERNET_H
#define ETHERNET_H

#include "board_config.h"
#include <stdint.h>

typedef enum {
    ETH_OK = 0,
    ETH_ERROR,
    ETH_NOT_READY,
    ETH_LINK_DOWN
} eth_status_t;

typedef void (*eth_rx_callback_t)(uint8_t *data, uint16_t len);

void ethernet_init(void);
eth_status_t ethernet_send(uint8_t *data, uint16_t len);
eth_status_t ethernet_get_status(void);
void ethernet_set_rx_callback(eth_rx_callback_t callback);
void ethernet_process(void);
uint8_t ethernet_is_link_up(void);

#endif /* ETHERNET_H */
