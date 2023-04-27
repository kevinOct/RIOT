#ifndef SIM7020_CONF_H
#define SIM7020_CONF_H

#define APN_SIZE 64
#define OPERATOR_SIZE 8

typedef enum {
    SIM7020_CONF_MANUAL_OPERATOR = 1 << 0,
    SIM7020_CONF_MANUAL_APN      = 1 << 1,
} conf_flags_t;

struct sim7020_conf {
    conf_flags_t flags;
    char apn[APN_SIZE]; /* APN (Access Point Name) */
    char operator[OPERATOR_SIZE];  /* Operator MCCMNC (mobile country code and mobile network code) */
};

typedef struct sim7020_conf sim7020_conf_t;

int cmd_sim7020_conf(int argc, char **argv);
void sim7020_conf_init(void);
#endif /* SIM7020_CONF_H */
