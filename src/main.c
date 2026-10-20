#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t sys_tick = 0;

extern void xPortSysTickHandler(void);

static void rs485_modbus_rx_callback(uint8_t *data, uint16_t len);
static void eth_modbus_callback(uint8_t *data, uint16_t len);

static void io_scan_task(void *arg);
static void rs485_task(void *arg);
static void ethernet_task(void *arg);

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

uint32_t HAL_GetTick(void)
{
    return sys_tick;
}

void HAL_Delay(uint32_t delay)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(delay));
    } else {
        uint32_t tickstart = sys_tick;
        while ((sys_tick - tickstart) < delay) {
            __WFI();
        }
    }
}

void vApplicationTickHook(void)
{
    sys_tick++;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    while (1);
}

void vApplicationMallocFailedHook(void)
{
    while (1);
}

void system_clock_config(void)
{
    RCC_OscInitTypeDef rcc_osc = {0};
    RCC_ClkInitTypeDef rcc_clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    rcc_osc.HSEState       = RCC_HSE_ON;
    rcc_osc.PLL.PLLState   = RCC_PLL_ON;
    rcc_osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    rcc_osc.PLL.PLLM       = 8;
    rcc_osc.PLL.PLLN       = 336;
    rcc_osc.PLL.PLLP       = RCC_PLLP_DIV2;
    rcc_osc.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&rcc_osc);

    rcc_clk.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                              RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    rcc_clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk.APB1CLKDivider = RCC_HCLK_DIV4;
    rcc_clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_5);
}

int main(void)
{
    HAL_Init();
    system_clock_config();

    SysTick_Config(SystemCoreClock / 1000);

    digital_inputs_init();
    digital_outputs_init();
    relays_init();
    analog_inputs_init();
    analog_outputs_init();
    rs485_init(RS485_BAUDRATE);
    ethernet_init();

    modbus_rtu_init(MODBUS_RTU_ADDRESS);
    modbus_tcp_init(MODBUS_SLAVE_ID);

    rs485_set_rx_callback(rs485_modbus_rx_callback);
    ethernet_set_rx_callback(eth_modbus_callback);

    analog_output_write_voltage(0, 0.0f);
    analog_output_write_voltage(1, 0.0f);

    xTaskCreate(io_scan_task,    "IO_Scan",  512,  NULL, 3, NULL);
    xTaskCreate(rs485_task,      "RS485",    512,  NULL, 2, NULL);
    xTaskCreate(ethernet_task,   "ETH",     1024,  NULL, 2, NULL);

    vTaskStartScheduler();

    while (1);
}

static void io_scan_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        digital_inputs_scan();

        uint16_t ai_buf[AI_COUNT];
        analog_inputs_scan_all(ai_buf);
        for (uint8_t i = 0; i < AI_COUNT; i++) {
            modbus_write_holding_register(MODBUS_HOLDING_REG_OFFSET + 100 + i, ai_buf[i]);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

static void rs485_task(void *arg)
{
    (void)arg;

    for (;;) {
        rs485_process();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void ethernet_task(void *arg)
{
    (void)arg;

    for (;;) {
        ethernet_process();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void rs485_modbus_rx_callback(uint8_t *data, uint16_t len)
{
    uint8_t response[MODBUS_RTU_FRAME_MAX];
    uint16_t resp_len = 0;

    modbus_rtu_process(data, len, response, &resp_len);
    if (resp_len > 0) {
        rs485_set_tx_mode();
        for (volatile uint32_t i = 0; i < 1000; i++) { __NOP(); }
        rs485_send(response, resp_len);
        rs485_set_rx_mode();
    }
}

static void eth_modbus_callback(uint8_t *data, uint16_t len)
{
    uint8_t response[MODBUS_TCP_MAX_ADU];
    uint16_t resp_len = 0;

    modbus_tcp_build_response(data, len, response, &resp_len);
    if (resp_len > 0) {
        ethernet_send(response, resp_len);
    }
}

void NMI_Handler(void) { while (1); }
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void DebugMon_Handler(void) {}
