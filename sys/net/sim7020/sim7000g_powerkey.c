#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "periph/gpio.h"
#include "xtimer.h"
#include "net/sim7000g_powerkey.h"

static gpio_t sim7000g_pin = SIM7000G_LDO_GPIO;

int sim7000g_powerkey_init(void) {
    int res;
    if ((res = gpio_init(sim7000g_pin, GPIO_OUT) < 0)) {
        printf("Error initialize sim7000g gpio powerkey: %d\n", res);
    }
    return res;
}

void sim7000g_power_on(void) {
    gpio_set(sim7000g_pin);
}

void sim7000g_power_off(void) {
    gpio_clear(sim7000g_pin);
}

int sim7000g_power_toggle(int argc, char **argv) {
    if (argc != 2)
        goto usage;
    sim7000g_powerkey_init();
    if (strcmp(argv[1], "on") == 0) {
        sim7000g_power_on();
    }
    else if (strcmp(argv[1], "off") == 0) {
        sim7000g_power_off();
    }
    else
usage:
        printf("Usage: power on|off\n");
    return 0;
}
