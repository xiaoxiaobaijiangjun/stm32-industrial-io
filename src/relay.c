#include "relay.h"

static const uint16_t relay_pins[RELAY_COUNT] = {
    RELAY1_PIN, RELAY2_PIN, RELAY3_PIN, RELAY4_PIN
};

static const uint16_t relay_led_pins[RELAY_COUNT] = {
    RELAY1_LED_PIN, RELAY2_LED_PIN, RELAY3_LED_PIN, RELAY4_LED_PIN
};

static uint8_t relay_states = 0;

void relays_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RELAY_CLK_ENABLE();
    gpio.Pin   = RELAY1_PIN | RELAY2_PIN | RELAY3_PIN | RELAY4_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_PORT, &gpio);

    RELAY_LED_CLK_ENABLE();
    gpio.Pin   = RELAY1_LED_PIN | RELAY2_LED_PIN | RELAY3_LED_PIN | RELAY4_LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_LED_PORT, &gpio);

    relays_set_all(0x00);
}

void relay_set(uint8_t channel, uint8_t state)
{
    if (channel >= RELAY_COUNT) return;

    if (state) {
        relay_states |= (1 << channel);
        RELAY_PORT->BSRR = relay_pins[channel];
    } else {
        relay_states &= ~(1 << channel);
        RELAY_PORT->BSRR = (uint32_t)relay_pins[channel] << 16U;
    }
    relay_led_update(channel, state);
}

uint8_t relay_get(uint8_t channel)
{
    if (channel >= RELAY_COUNT) return 0;
    return (relay_states >> channel) & 0x01;
}

void relays_set_all(uint8_t mask)
{
    relay_states = mask & 0x0F;
    uint32_t bsrr_set = 0;
    uint32_t bsrr_reset = 0;

    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        if (mask & (1 << i)) {
            bsrr_set |= relay_pins[i];
        } else {
            bsrr_reset |= relay_pins[i];
        }
    }
    RELAY_PORT->BSRR = bsrr_reset << 16U;
    RELAY_PORT->BSRR = bsrr_set;

    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        relay_led_update(i, (mask >> i) & 0x01);
    }
}

uint8_t relays_get_all(void)
{
    return relay_states;
}

void relay_toggle(uint8_t channel)
{
    if (channel >= RELAY_COUNT) return;
    relay_set(channel, relay_get(channel) ^ 0x01);
}

void relay_led_update(uint8_t channel, uint8_t state)
{
    if (channel >= RELAY_COUNT) return;

    if (state) {
        RELAY_LED_PORT->BSRR = relay_led_pins[channel];
    } else {
        RELAY_LED_PORT->BSRR = (uint32_t)relay_led_pins[channel] << 16U;
    }
}
