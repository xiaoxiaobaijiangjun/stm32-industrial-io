#ifndef DIGITAL_IO_H
#define DIGITAL_IO_H

#include "board_config.h"
#include <stdint.h>

typedef enum {
    DIO_OK = 0,
    DIO_ERROR,
    DIO_INVALID_CHANNEL
} dio_status_t;

void digital_inputs_init(void);
uint8_t digital_inputs_read_all(void);
uint8_t digital_input_read(uint8_t channel);
void digital_inputs_scan(void);

void digital_outputs_init(void);
void digital_outputs_write_all(uint8_t value);
void digital_output_write(uint8_t channel, uint8_t state);
uint8_t digital_output_read(uint8_t channel);
uint8_t digital_outputs_read_all(void);

#endif /* DIGITAL_IO_H */
