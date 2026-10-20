/*
 * industrial.c - 工业三件套实现（见 industrial.h 的设计说明）
 */

#include "industrial.h"
#include "board_config.h"
#include "stm32f4xx_hal.h"
#include "digital_io.h"
#include "relay.h"
#include "modbus.h"

/* ---------------- flash storage (last 128 KB sector, F407VG 1 MB) --------- */
#define IND_FLASH_SECTOR     11U
#define IND_FLASH_ADDRESS    0x080E0000U
#define IND_MAGIC            0x31444E49U   /* "IND1" */
#define IND_CFG_VERSION      1U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t wdt_timeout_ms;
    uint16_t safe_bitmap;
    uint8_t  wdt_enable;
    uint8_t  safe_mode;
    uint8_t  slave_id;
    uint8_t  baud_index;
    uint32_t crc;                 /* CRC32 over the 16 bytes above */
} ind_store_t;

typedef char ind_size_check[(sizeof(ind_store_t) == 20U) ? 1 : -1];

static const uint32_t ind_baud_table[5] = {9600U, 19200U, 38400U, 57600U, 115200U};

static industrial_cfg_t cfg = {
    1U,      /* wdt_enable            */
    3000U,   /* wdt_timeout_ms        */
    IND_SAFE_ALL_OFF,   /* safe_mode    */
    0x0000U, /* safe_bitmap           */
    1U,      /* slave_id              */
    4U,      /* baud_index = 115200   */
};

/* runtime state */
static volatile uint32_t last_frame_tick = 0U;
static volatile uint8_t  wdt_expired = 0U;
static volatile uint32_t stat_rx_frames = 0U;
static volatile uint32_t stat_crc_errors = 0U;
static uint32_t reset_csr_snapshot = 0U;
static volatile uint32_t idle_stack_min = 0xFFFFFFFFU;

/* ---------------- software CRC32 (same style as the other projects) ------ */
static uint32_t ind_crc32(const uint8_t *d, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i;
    uint8_t  bit;

    for(i = 0U; i < n; i++)
    {
        crc ^= d[i];
        for(bit = 0U; bit < 8U; bit++)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ---------------- storage ------------------------------------------------ */
static uint8_t ind_store_save(void)
{
    ind_store_t s;
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0U;
    uint32_t i;
    uint8_t ok = 1U;
    const uint8_t *p8;

    s.magic = IND_MAGIC;
    s.version = IND_CFG_VERSION;
    s.wdt_timeout_ms = cfg.wdt_timeout_ms;
    s.safe_bitmap = cfg.safe_bitmap;
    s.wdt_enable = cfg.wdt_enable;
    s.safe_mode = cfg.safe_mode;
    s.slave_id = cfg.slave_id;
    s.baud_index = cfg.baud_index;
    s.crc = ind_crc32((const uint8_t *)&s, 16U);

    /* NOTE: erasing a 128 KB sector stalls the CPU for ~1-2 s; acceptable
     * because SAVE is a rare, explicit configuration action (not in the
     * 24 h polling path). */
    HAL_FLASH_Unlock();
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = IND_FLASH_SECTOR;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if(HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        ok = 0U;
    }
    p8 = (const uint8_t *)&s;
    for(i = 0U; (i < sizeof(ind_store_t)) && ok; i += 4U)
    {
        uint32_t word = (uint32_t)p8[i] | ((uint32_t)p8[i + 1U] << 8) |
                        ((uint32_t)p8[i + 2U] << 16) | ((uint32_t)p8[i + 3U] << 24);
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IND_FLASH_ADDRESS + i, word) != HAL_OK)
        {
            ok = 0U;
        }
    }
    HAL_FLASH_Lock();

    if(ok)
    {
        const ind_store_t *p = (const ind_store_t *)IND_FLASH_ADDRESS;
        ok = (p->magic == IND_MAGIC) && (p->crc == s.crc);
    }
    return ok;
}

static void ind_store_load(void)
{
    const ind_store_t *p = (const ind_store_t *)IND_FLASH_ADDRESS;

    if((p->magic != IND_MAGIC) || (p->version != IND_CFG_VERSION) ||
       (ind_crc32((const uint8_t *)p, 16U) != p->crc))
    {
        return;   /* blank/corrupt: keep factory defaults */
    }
    if(!((p->wdt_timeout_ms >= 500U) && (p->wdt_timeout_ms <= 10000U) &&
         (p->safe_mode <= IND_SAFE_PRESET) && (p->slave_id >= 1U) && (p->slave_id <= 247U) &&
         (p->baud_index <= 4U)))
    {
        return;   /* out-of-range values: fall back to defaults */
    }
    cfg.wdt_enable = p->wdt_enable ? 1U : 0U;
    cfg.wdt_timeout_ms = p->wdt_timeout_ms;
    cfg.safe_mode = p->safe_mode;
    cfg.safe_bitmap = p->safe_bitmap;
    cfg.slave_id = p->slave_id;
    cfg.baud_index = p->baud_index;
}

/* ---------------- comm watchdog ------------------------------------------ */
void ind_comm_kick(void)
{
    stat_rx_frames++;
    last_frame_tick = HAL_GetTick();
    if(wdt_expired)
    {
        /* recovery: outputs follow the coil table again. The recovering
         * frame's own writes (if any) sync right after this in the engine,
         * pure reads restore the pre-timeout coil states here. */
        wdt_expired = 0U;
        modbus_sync_outputs();
    }
}

void ind_on_crc_error(void)
{
    stat_crc_errors++;
}

uint8_t ind_output_override_active(void)
{
    return wdt_expired;
}

/* apply the configured safe state to DO + relays */
static void apply_safe_state(void)
{
    uint8_t do_val = 0U;
    uint8_t rel_val = 0U;

    switch(cfg.safe_mode)
    {
        case IND_SAFE_HOLD:
            /* keep the current outputs: nothing to write */
            break;
        case IND_SAFE_PRESET:
            do_val = (uint8_t)(cfg.safe_bitmap & 0xFFU);
            rel_val = (uint8_t)((cfg.safe_bitmap >> 8) & 0x0FU);
            digital_outputs_write_all(do_val);
            relays_set_all(rel_val);
            break;
        case IND_SAFE_ALL_OFF:
        default:
            digital_outputs_write_all(0U);
            relays_set_all(0U);
            break;
    }
}

void ind_watchdog_service(void)
{
    if(!cfg.wdt_enable)
    {
        return;
    }
    if(!wdt_expired)
    {
        if((HAL_GetTick() - last_frame_tick) > cfg.wdt_timeout_ms)
        {
            wdt_expired = 1U;
            apply_safe_state();
        }
    }
}

/* ---------------- stats helpers ------------------------------------------ */
void ind_idle_hook(uint32_t stack_high_water)
{
    if(stack_high_water < idle_stack_min)
    {
        idle_stack_min = stack_high_water;
    }
}

/* ---------------- boot ---------------------------------------------------- */
uint32_t industrial_baud(void)
{
    return ind_baud_table[cfg.baud_index];
}

uint8_t industrial_slave_id(void)
{
    return cfg.slave_id;
}

void industrial_init(void)
{
    /* snapshot the reset cause, then clear the flags so a later poll of
     * 0x2014 always reflects THIS boot */
    reset_csr_snapshot = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    ind_store_load();
    last_frame_tick = HAL_GetTick();
}

/* ---------------- modbus config / stats registers ------------------------ */
#define IND_REG_WDT_ENABLE   0x2000U
#define IND_REG_WDT_TIMEOUT  0x2001U
#define IND_REG_SAFE_MODE    0x2002U
#define IND_REG_SAFE_BITMAPL 0x2003U
#define IND_REG_SAFE_BITMAPH 0x2004U
#define IND_REG_SAVE         0x2005U
#define IND_REG_SLAVE_ID     0x2006U
#define IND_REG_BAUD         0x2007U
#define IND_REG_WDT_STATE    0x2008U
#define IND_CFG_AREA_END     0x2009U

#define IND_REG_STAT_RXL     0x2010U
#define IND_REG_STAT_CRCL    0x2012U
#define IND_REG_STAT_RSTL    0x2014U
#define IND_REG_STAT_STKL    0x2016U
#define IND_STAT_AREA_END    0x2018U

static uint16_t stat_u32_lo(volatile uint32_t v) { return (uint16_t)(v & 0xFFFFU); }
static uint16_t stat_u32_hi(volatile uint32_t v) { return (uint16_t)(v >> 16); }

uint8_t ind_regs_read(uint16_t start, uint16_t qty, uint16_t *out)
{
    uint16_t in_cfg = 0U, in_stat = 0U;
    uint16_t i;

    if((start + qty) <= IND_REG_WDT_ENABLE || start >= IND_STAT_AREA_END)
    {
        return 0U;   /* not touching the industrial area */
    }
    for(i = 0U; i < qty; i++)
    {
        uint16_t a = start + i;
        if((a >= IND_REG_WDT_ENABLE) && (a < IND_CFG_AREA_END)) in_cfg++;
        else if((a >= IND_REG_STAT_RXL) && (a < IND_STAT_AREA_END)) in_stat++;
        else return 2U;   /* hole between the two blocks (0x2009..0x200F) */
    }
    if(in_cfg && in_stat)
    {
        return 2U;   /* spanning both blocks: keep clients simple, refuse */
    }

    for(i = 0U; i < qty; i++)
    {
        uint16_t a = start + i;
        if(in_cfg)
        {
            switch(a)
            {
                case IND_REG_WDT_ENABLE:   out[i] = cfg.wdt_enable; break;
                case IND_REG_WDT_TIMEOUT:  out[i] = cfg.wdt_timeout_ms; break;
                case IND_REG_SAFE_MODE:    out[i] = cfg.safe_mode; break;
                case IND_REG_SAFE_BITMAPL: out[i] = cfg.safe_bitmap & 0xFFFFU; break;
                case IND_REG_SAFE_BITMAPH: out[i] = 0U; break;
                case IND_REG_SAVE:         out[i] = 0U; break;
                case IND_REG_SLAVE_ID:     out[i] = cfg.slave_id; break;
                case IND_REG_BAUD:         out[i] = cfg.baud_index; break;
                case IND_REG_WDT_STATE:    out[i] = wdt_expired ? 1U : 0U; break;
                default:                   out[i] = 0U; break;
            }
        }
        else
        {
            switch(a)
            {
                case IND_REG_STAT_RXL:  out[i] = stat_u32_lo(stat_rx_frames); break;
                case IND_REG_STAT_RXL + 1U:  out[i] = stat_u32_hi(stat_rx_frames); break;
                case IND_REG_STAT_CRCL: out[i] = stat_u32_lo(stat_crc_errors); break;
                case IND_REG_STAT_CRCL + 1U: out[i] = stat_u32_hi(stat_crc_errors); break;
                case IND_REG_STAT_RSTL: out[i] = stat_u32_lo(reset_csr_snapshot); break;
                case IND_REG_STAT_RSTL + 1U: out[i] = stat_u32_hi(reset_csr_snapshot); break;
                case IND_REG_STAT_STKL: out[i] = (idle_stack_min == 0xFFFFFFFFU) ? 0xFFFFU : stat_u32_lo(idle_stack_min); break;
                case IND_REG_STAT_STKL + 1U: out[i] = (idle_stack_min == 0xFFFFFFFFU) ? 0xFFFFU : stat_u32_hi(idle_stack_min); break;
                default: out[i] = 0U; break;
            }
        }
    }
    return 1U;
}

uint8_t ind_reg_validate(uint16_t addr, uint16_t value)
{
    switch(addr)
    {
        case IND_REG_WDT_ENABLE:
        case IND_REG_SAFE_BITMAPL:
        case IND_REG_SAFE_BITMAPH:
            return 1U;
        case IND_REG_WDT_TIMEOUT:
            return ((value >= 500U) && (value <= 10000U)) ? 1U : 2U;
        case IND_REG_SAFE_MODE:
            return (value <= IND_SAFE_PRESET) ? 1U : 2U;
        case IND_REG_SAVE:
            return (value == 0x5AA5U) ? 1U : 2U;
        case IND_REG_SLAVE_ID:
            return ((value >= 1U) && (value <= 247U)) ? 1U : 2U;
        case IND_REG_BAUD:
            return (value <= 4U) ? 1U : 2U;
        default:
            if((addr == IND_REG_WDT_STATE) ||
               ((addr >= IND_REG_STAT_RXL) && (addr < IND_STAT_AREA_END)))
            {
                return 2U;   /* read-only */
            }
            return 0U;       /* not mine */
    }
}

uint8_t ind_reg_write(uint16_t addr, uint16_t value)
{
    switch(addr)
    {
        case IND_REG_WDT_ENABLE:
            cfg.wdt_enable = value ? 1U : 0U;
            if(!cfg.wdt_enable)
            {
                /* disabling the watchdog must also lift an active override */
                wdt_expired = 0U;
                modbus_sync_outputs();
            }
            return 1U;
        case IND_REG_WDT_TIMEOUT:
            if((value < 500U) || (value > 10000U))
            {
                return 2U;
            }
            cfg.wdt_timeout_ms = value;
            return 1U;
        case IND_REG_SAFE_MODE:
            if(value > IND_SAFE_PRESET)
            {
                return 2U;
            }
            cfg.safe_mode = (uint8_t)value;
            return 1U;
        case IND_REG_SAFE_BITMAPL:
            cfg.safe_bitmap = (cfg.safe_bitmap & 0xFF00U) | (value & 0x00FFU);
            return 1U;
        case IND_REG_SAFE_BITMAPH:
            /* only bits 8-11 map to relays: mask the rest */
            cfg.safe_bitmap = (cfg.safe_bitmap & 0x00FFU) | ((value << 8) & 0x0F00U);
            return 1U;
        case IND_REG_SAVE:
            return (value == 0x5AA5U) ? (ind_store_save() ? 1U : 2U) : 2U;
        case IND_REG_SLAVE_ID:
            if((value < 1U) || (value > 247U))
            {
                return 2U;
            }
            cfg.slave_id = (uint8_t)value;   /* applied at next boot */
            return 1U;
        case IND_REG_BAUD:
            if(value > 4U)
            {
                return 2U;
            }
            cfg.baud_index = (uint8_t)value; /* applied at next boot */
            return 1U;
        default:
            /* read-only registers (wdt state + stats area) reject writes */
            if((addr == IND_REG_WDT_STATE) ||
               ((addr >= IND_REG_STAT_RXL) && (addr < IND_STAT_AREA_END)))
            {
                return 2U;
            }
            return 0U;   /* not mine */
    }
}
