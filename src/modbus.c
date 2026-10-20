#include "modbus.h"
#include "digital_io.h"
#include "analog_io.h"
#include "relay.h"
#include "industrial.h"

static uint8_t modbus_slave_id = 1;
static uint16_t holding_regs[MODBUS_MAX_REGISTERS] = {0};
static uint16_t input_regs[MODBUS_MAX_REGISTERS] = {0};
static uint8_t  coil_bits[(MODBUS_MAX_COILS + 7) / 8] = {0};
static uint8_t  discrete_bits[(MODBUS_MAX_COILS + 7) / 8] = {0};

static uint16_t low_crc_table[256];
static uint16_t high_crc_table[256];
static uint8_t  crc_table_initialized = 0;

static void modbus_crc_init_tables(void)
{
    if (crc_table_initialized) return;

    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
        low_crc_table[i]  = crc & 0xFF;
        high_crc_table[i] = crc >> 8;
    }
    crc_table_initialized = 1;
}

uint16_t modbus_crc16(uint8_t *buf, uint16_t len)
{
    modbus_crc_init_tables();

    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;
    uint8_t idx;

    for (uint16_t i = 0; i < len; i++) {
        idx = crc_lo ^ buf[i];
        crc_lo = crc_hi ^ high_crc_table[idx];
        crc_hi = low_crc_table[idx];
    }
    return ((uint16_t)crc_hi << 8) | crc_lo;
}

void modbus_rtu_init(uint8_t slave_id)
{
    modbus_slave_id = slave_id;
    modbus_crc_init_tables();
    for (uint16_t i = 0; i < MODBUS_MAX_REGISTERS; i++) {
        holding_regs[i] = 0;
        input_regs[i]  = 0;
    }
    for (uint16_t i = 0; i < sizeof(coil_bits); i++) {
        coil_bits[i] = 0;
        discrete_bits[i] = 0;
    }
}

static uint8_t modbus_bit_read(uint8_t *table, uint16_t addr)
{
    if (addr >= MODBUS_MAX_COILS) return 0;
    return (table[addr / 8] >> (addr % 8)) & 0x01;
}

static void modbus_bit_write(uint8_t *table, uint16_t addr, uint8_t value)
{
    if (addr >= MODBUS_MAX_COILS) return;
    if (value) {
        table[addr / 8] |= (1 << (addr % 8));
    } else {
        table[addr / 8] &= ~(1 << (addr % 8));
    }
}

static void modbus_regs_write(uint16_t *regs, uint16_t addr, uint16_t value)
{
    if (addr >= MODBUS_MAX_REGISTERS) return;
    regs[addr] = value;
}

static void modbus_sync_registers(void)
{
    /* Synchronize hardware I/O with Modbus register map.
     * While the comm watchdog forces the safe state, output writes are
     * suppressed (recovery re-syncs from the coil table). */
    if (!ind_output_override_active()) {
        /* Digital outputs -> Coils 0-7 */
        for (uint8_t i = 0; i < DO_COUNT; i++) {
            uint8_t state = modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + i);
            digital_output_write(i, state);
        }

        /* Relays -> Coils 8-11 */
        for (uint8_t i = 0; i < RELAY_COUNT; i++) {
            uint8_t state = modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + 8 + i);
            relay_set(i, state);
        }
    }

    /* Digital inputs -> Discrete inputs 0-7 */
    uint8_t di = digital_inputs_read_all();
    for (uint8_t i = 0; i < DI_COUNT; i++) {
        modbus_bit_write(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + i,
                         (di >> i) & 0x01);
    }

    /* Analog inputs -> Input registers 0-3 */
    uint16_t ai_buf[AI_COUNT];
    analog_inputs_scan_all(ai_buf);
    for (uint8_t i = 0; i < AI_COUNT; i++) {
        input_regs[MODBUS_INPUT_REG_OFFSET + i] = ai_buf[i];
    }

    /* Analog outputs -> Holding registers 0-1 */
    for (uint8_t i = 0; i < AO_COUNT; i++) {
        uint16_t val = holding_regs[MODBUS_HOLDING_REG_OFFSET + i];
        analog_output_write_raw(i, val);
    }
}

/* Public: re-push the coil table to the outputs (comm-watchdog recovery). */
void modbus_sync_outputs(void)
{
    modbus_sync_registers();
}

/* ============================================================
 * Modbus RTU Frame Processing
 * ============================================================ */
modbus_status_t modbus_rtu_process(uint8_t *rx_buf, uint16_t rx_len,
                                    uint8_t *tx_buf, uint16_t *tx_len)
{
    if (rx_len < 4) return MODBUS_ERROR;

    uint16_t calc_crc = modbus_crc16(rx_buf, rx_len - 2);
    uint16_t sent_crc = ((uint16_t)rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
    if (sent_crc != calc_crc) {
        ind_on_crc_error();
        return MODBUS_CRC_ERROR;
    }

    uint8_t slave_addr = rx_buf[0];
    uint8_t func_code  = rx_buf[1];

    if (slave_addr != modbus_slave_id && slave_addr != 0) {
        return MODBUS_OK; /* Not for us */
    }

    /* valid frame addressed to this device (any link): comm watchdog kick */
    ind_comm_kick();

    *tx_len  = 0;
    tx_buf[0] = modbus_slave_id;
    tx_buf[1] = func_code;

    uint16_t start_addr = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
    uint16_t quantity    = 0;
    uint16_t resp_len    = 2;

    if (rx_len >= 6) {
        quantity = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];
    }

    switch (func_code) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        if (start_addr + quantity > MODBUS_MAX_COILS) {
            tx_buf[1] |= 0x80;
            tx_buf[2]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            resp_len   = 3;
            break;
        }
        uint8_t byte_count = (quantity + 7) / 8;
        tx_buf[2] = byte_count;
        for (uint8_t i = 0; i < byte_count; i++) {
            uint8_t val = 0;
            for (uint8_t b = 0; b < 8; b++) {
                uint16_t addr = start_addr + i * 8 + b;
                if (b + i * 8 >= quantity) break;
                if (func_code == MODBUS_FC_READ_COILS) {
                    val |= modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + addr) << b;
                } else {
                    val |= modbus_bit_read(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + addr) << b;
                }
            }
            tx_buf[3 + i] = val;
        }
        resp_len = 3 + byte_count;
        break;
    }
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS: {
        if (func_code == MODBUS_FC_READ_HOLDING_REGISTERS) {
            uint16_t cfg_vals[MODBUS_RTU_FRAME_MAX / 2];
            uint8_t r = ind_regs_read(start_addr, quantity, cfg_vals);
            if (r == 2U) {
                tx_buf[1] |= 0x80;
                tx_buf[2]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
                resp_len   = 3;
                break;
            }
            if (r == 1U) {
                uint8_t bc = quantity * 2U;
                tx_buf[2] = bc;
                for (uint16_t i = 0; i < quantity; i++) {
                    tx_buf[3 + i * 2]     = cfg_vals[i] >> 8;
                    tx_buf[3 + i * 2 + 1] = cfg_vals[i] & 0xFF;
                }
                resp_len = 3 + bc;
                break;
            }
        }
        if (start_addr + quantity > MODBUS_MAX_REGISTERS) {
            tx_buf[1] |= 0x80;
            tx_buf[2]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            resp_len   = 3;
            break;
        }
        uint8_t byte_count = quantity * 2;
        tx_buf[2] = byte_count;
        for (uint16_t i = 0; i < quantity; i++) {
            uint16_t val;
            if (func_code == MODBUS_FC_READ_HOLDING_REGISTERS) {
                val = holding_regs[MODBUS_HOLDING_REG_OFFSET + start_addr + i];
            } else {
                val = input_regs[MODBUS_INPUT_REG_OFFSET + start_addr + i];
            }
            tx_buf[3 + i * 2]     = val >> 8;
            tx_buf[3 + i * 2 + 1] = val & 0xFF;
        }
        resp_len = 3 + byte_count;
        break;
    }
    case MODBUS_FC_WRITE_SINGLE_COIL: {
        uint16_t coil_val = (quantity == 0xFF00) ? 1 : 0;
        modbus_bit_write(coil_bits, MODBUS_COIL_OFFSET + start_addr, coil_val);
        modbus_sync_registers();
        tx_buf[2] = rx_buf[2];
        tx_buf[3] = rx_buf[3];
        tx_buf[4] = rx_buf[4];
        tx_buf[5] = rx_buf[5];
        resp_len  = 6;
        break;
    }
    case MODBUS_FC_WRITE_SINGLE_REGISTER: {
        uint16_t reg_val = quantity;
        uint8_t wv = ind_reg_validate(start_addr, reg_val);
        if (wv == 2U) {
            tx_buf[1] |= 0x80;
            tx_buf[2]  = MODBUS_EXC_ILLEGAL_DATA_VALUE;
            resp_len   = 3;
            break;
        }
        if (wv == 1U) {
            /* flash save failure is a device fault, not an address error */
            if (ind_reg_write(start_addr, reg_val) == 2U) {
                tx_buf[1] |= 0x80;
                tx_buf[2]  = MODBUS_EXC_SLAVE_DEVICE_FAILURE;
                resp_len   = 3;
                break;
            }
        } else {
            modbus_regs_write(holding_regs, MODBUS_HOLDING_REG_OFFSET + start_addr, reg_val);
        }
        modbus_sync_registers();
        tx_buf[2] = rx_buf[2];
        tx_buf[3] = rx_buf[3];
        tx_buf[4] = rx_buf[4];
        tx_buf[5] = rx_buf[5];
        resp_len  = 6;
        break;
    }
    case MODBUS_FC_WRITE_MULTIPLE_COILS: {
        if (start_addr + quantity > MODBUS_MAX_COILS) {
            tx_buf[1] |= 0x80;
            tx_buf[2]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            resp_len   = 3;
            break;
        }
        (void)rx_buf[6]; /* byte_count validated by quantity check above */
        for (uint16_t i = 0; i < quantity; i++) {
            uint8_t val = (rx_buf[7 + i / 8] >> (i % 8)) & 0x01;
            modbus_bit_write(coil_bits, MODBUS_COIL_OFFSET + start_addr + i, val);
        }
        modbus_sync_registers();
        tx_buf[2] = rx_buf[2];
        tx_buf[3] = rx_buf[3];
        tx_buf[4] = rx_buf[4];
        tx_buf[5] = rx_buf[5];
        resp_len  = 6;
        break;
    }
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        uint8_t reject = 0U;
        /* validate the whole request first: plain registers OR the industrial
         * config area; holes / read-only / bad values reject ATOMICALLY
         * (nothing is written for a rejected frame) */
        if ((7U + (uint32_t)quantity * 2U + 2U) > rx_len) {
            reject = 1U;
        }
        for (uint16_t i = 0U; (i < quantity) && !reject; i++) {
            uint16_t a = start_addr + i;
            uint16_t val = ((uint16_t)rx_buf[7 + i * 2] << 8) | rx_buf[7 + i * 2 + 1];
            uint8_t wv = ind_reg_validate(a, val);
            if (wv == 2U || (wv == 0U && a >= MODBUS_MAX_REGISTERS)) {
                reject = 1U;
            }
        }
        if (reject) {
            tx_buf[1] |= 0x80;
            tx_buf[2]  = MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
            resp_len   = 3;
            break;
        }
        for (uint16_t i = 0U; i < quantity; i++) {
            uint16_t val = ((uint16_t)rx_buf[7 + i * 2] << 8) | rx_buf[7 + i * 2 + 1];
            if (ind_reg_write(start_addr + i, val) == 0U) {
                modbus_regs_write(holding_regs, MODBUS_HOLDING_REG_OFFSET + start_addr + i, val);
            }
        }
        modbus_sync_registers();
        tx_buf[2] = rx_buf[2];
        tx_buf[3] = rx_buf[3];
        tx_buf[4] = rx_buf[4];
        tx_buf[5] = rx_buf[5];
        resp_len  = 6;
        break;
    }
    default:
        tx_buf[1] |= 0x80;
        tx_buf[2]  = MODBUS_EXC_ILLEGAL_FUNCTION;
        resp_len   = 3;
        break;
    }

    if (resp_len > 2) {
        uint16_t tx_crc = modbus_crc16(tx_buf, resp_len);
        tx_buf[resp_len]     = tx_crc & 0xFF;
        tx_buf[resp_len + 1] = tx_crc >> 8;
        *tx_len = resp_len + 2;
    }

    return MODBUS_OK;
}

/* ============================================================
 * Modbus Register Access API
 * ============================================================ */
uint16_t modbus_read_coil(uint16_t addr)
{
    return modbus_bit_read(coil_bits, MODBUS_COIL_OFFSET + addr);
}

void modbus_write_coil(uint16_t addr, uint16_t value)
{
    modbus_bit_write(coil_bits, MODBUS_COIL_OFFSET + addr, value ? 1 : 0);
}

uint16_t modbus_read_discrete_input(uint16_t addr)
{
    return modbus_bit_read(discrete_bits, MODBUS_DISCRETE_INPUT_OFFSET + addr);
}

uint16_t modbus_read_holding_register(uint16_t addr)
{
    if (addr >= MODBUS_MAX_REGISTERS) return 0;
    return holding_regs[MODBUS_HOLDING_REG_OFFSET + addr];
}

void modbus_write_holding_register(uint16_t addr, uint16_t value)
{
    modbus_regs_write(holding_regs, MODBUS_HOLDING_REG_OFFSET + addr, value);
}

uint16_t modbus_read_input_register(uint16_t addr)
{
    if (addr >= MODBUS_MAX_REGISTERS) return 0;
    return input_regs[MODBUS_INPUT_REG_OFFSET + addr];
}
