/*
 * industrial.h - 工业三件套：通信看门狗 + 掉电参数保持 + DI 消抖
 *
 * 通信看门狗：任何一条链路（RS485 / 以太网）收到 CRC 正确且寻址本机的
 * 有效主站帧即刷新时间戳；超时未刷新则 DO/继电器进入安全态，恢复后
 * 自动回到线圈表状态。安全态三种：
 *   IND_SAFE_HOLD    保持当前输出
 *   IND_SAFE_ALL_OFF 全部断开
 *   IND_SAFE_PRESET  输出预置位图（bit0-7 = DO0-7, bit8-11 = 继电器0-3）
 *
 * 掉电参数保持：Modbus 配置区 0x2000 起，写 0x2005=0x5AA5 触发落 Flash
 *（末扇区，magic+ver+CRC32，损坏回退出厂默认；保存期间通信会停约 1-2 秒
 *  属 128KB 扇区擦除的预期行为）。
 *
 * 配置寄存器（保持寄存器区，F03/F06/F10）：
 *   0x2000 看门狗使能 (0/1)          0x2001 超时 ms (500..10000)
 *   0x2002 安全态模式 (0/1/2)        0x2003-04 预置位图 (LE32)
 *   0x2005 保存触发 (写 0x5AA5)      0x2006 本机地址 (1..247, 重启生效)
 *   0x2007 波特率档 (0=9600..4=115200, 重启生效)
 *   0x2008 看门狗状态 (RO: 0=正常 1=已进安全态)
 * 统计寄存器（RO，24h 稳定性测试用）：
 *   0x2010-11 有效主站帧计数 u32      0x2012-13 CRC 错帧计数 u32
 *   0x2014-15 本次上电复位原因 RCC_CSR u32（快照，标志已清）
 *   0x2016-17 空闲任务最小剩余栈 u32（字）
 */

#ifndef INDUSTRIAL_H
#define INDUSTRIAL_H

#include <stdint.h>

#define IND_SAFE_HOLD       0U
#define IND_SAFE_ALL_OFF    1U
#define IND_SAFE_PRESET     2U

typedef struct
{
    uint8_t  wdt_enable;        /* 0/1                                */
    uint16_t wdt_timeout_ms;    /* 500..10000, default 3000           */
    uint8_t  safe_mode;         /* IND_SAFE_*                         */
    uint16_t safe_bitmap;       /* DO0-7 | relay0-3 in bits 0-11      */
    uint8_t  slave_id;          /* 1..247                             */
    uint8_t  baud_index;        /* 0=9600 1=19200 2=38400 3=57600 4=115200 */
} industrial_cfg_t;

/* boot: load params (flash or defaults), snapshot + clear reset flags */
void         industrial_init(void);
uint32_t     industrial_baud(void);       /* resolved baudrate for rs485_init */
uint8_t      industrial_slave_id(void);   /* slave id for modbus_rtu_init     */

/* comm watchdog: call from the frame-processing path (any link) */
void         ind_comm_kick(void);         /* valid master frame arrived       */
void         ind_on_crc_error(void);      /* frame failed CRC (stats only)    */
void         ind_watchdog_service(void);  /* call periodically (io task)      */
uint8_t      ind_output_override_active(void); /* safe state forced now       */

/* idle stack statistics (call from vApplicationIdleHook) */
void         ind_idle_hook(uint32_t stack_high_water);

/* config-area registers, called from the modbus engine.
 * return: 0 = address not in the industrial area (caller falls through),
 *         1 = fully handled (data filled / write accepted or stats-ignored),
 *         2 = partial overlap or illegal value -> ILLEGAL_DATA_ADDRESS     */
uint8_t      ind_regs_read(uint16_t start, uint16_t qty, uint16_t *out);
uint8_t      ind_reg_write(uint16_t addr, uint16_t value);
/* value/address validation WITHOUT side effects (atomic F10 reject) */
uint8_t      ind_reg_validate(uint16_t addr, uint16_t value);

#endif /* INDUSTRIAL_H */
