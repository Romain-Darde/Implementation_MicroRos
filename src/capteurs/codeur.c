#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include "codeur.h"

// Définitions des pins (D2 correspond à PA10, D3 correspond à PB3)
#define PIN_A 10
#define PIN_B 3
static const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *gpiob = DEVICE_DT_GET(DT_NODELABEL(gpiob));
static struct gpio_callback codeur_cb;

static atomic_t compte = ATOMIC_INIT(0);
static int64_t dernier_temps_interruption = 0;

void codeur_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t temps_actuel = k_uptime_get_32();
    
    if (temps_actuel - (uint32_t)dernier_temps_interruption < 2) {
        return; // Si trop rapide on ignore évite de saturer 
    }
    dernier_temps_interruption = temps_actuel;

    int level_b = gpio_pin_get(gpiob, PIN_B);
    if (level_b < 0) {
        return;
    }

    if (level_b > 0) {
        atomic_inc(&compte);
    } else {
        atomic_dec(&compte);
    }
}

void codeur_init(void) {
    if (!device_is_ready(gpioa) || !device_is_ready(gpiob)) return;

    gpio_pin_configure(gpioa, PIN_A, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpiob, PIN_B, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpioa, PIN_A, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&codeur_cb, codeur_handler, BIT(PIN_A));
    gpio_add_callback(gpioa, &codeur_cb);
}

int codeur_get_ticks(void) {
    return (int)atomic_get(&compte);
}

void codeur_reset(void) {
    atomic_set(&compte, 0);
}