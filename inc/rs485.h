#ifndef RS485_H
#define RS485_H

#include "board_config.h"
#include <stdint.h>

typedef enum {
    RS485_OK = 0,
    RS485_ERROR,
    RS485_BUSY,
    RS485_TIMEOUT
} rs485_status_t;

typedef void (*rs485_rx_callback_t)(uint8_t *data, uint16_t len);

void rs485_init(uint32_t baudrate);
rs485_status_t rs485_send(uint8_t *data, uint16_t len);
rs485_status_t rs485_send_with_timeout(uint8_t *data, uint16_t len, uint32_t timeout_ms);
uint16_t rs485_bytes_available(void);
uint16_t rs485_read(uint8_t *buffer, uint16_t max_len);
void rs485_flush_rx(void);
void rs485_set_rx_callback(rs485_rx_callback_t callback);
void rs485_set_rx_mode(void);
void rs485_set_tx_mode(void);
void rs485_process(void);

#endif /* RS485_H */
