#ifndef ANALOG_IO_H
#define ANALOG_IO_H

#include "board_config.h"
#include <stdint.h>

typedef enum {
    AIO_OK = 0,
    AIO_ERROR,
    AIO_INVALID_CHANNEL
} aio_status_t;

void analog_inputs_init(void);
uint16_t analog_input_read_raw(uint8_t channel);
float analog_input_read_voltage(uint8_t channel);
float analog_input_read_eng_unit(uint8_t channel);
void analog_inputs_scan_all(uint16_t *buffer);

void analog_outputs_init(void);
void analog_output_write_raw(uint8_t channel, uint16_t value);
void analog_output_write_voltage(uint8_t channel, float voltage);
aio_status_t analog_output_write_eng_unit(uint8_t channel, float value);

#endif /* ANALOG_IO_H */
