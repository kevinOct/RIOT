#ifndef SIM7000G_POWERKEY_H
#define SIM7000G_POWERKEY_H

#ifndef SIM7000G_LDO_GPIO
#define SIM7000G_LDO_GPIO GPIO_PIN(3, 6)
#endif 

#ifndef SIM7000G_WAKE_GPIO
#define SIM7000G_WAKE_GPIO GPIO_PIN(3, 7)
#endif 

int sim7000g_powerkey_init(void);
void sim7000g_power_on(void);
void sim7000g_power_off(void);

#define sim7xxx_powerkey_init() sim7000g_powerkey_init()
#define sim7xxx_power_on() sim7000g_power_on()
#define sim7xxx_power_off() sim7000g_power_off()
#endif /* SIM7000G_POWERKEY_H */
