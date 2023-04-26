#ifndef SIM7020_CONF_H
#define SIM7020_CONF_H

#define APN_SIZE 64
#define OPERATOR_SIZE 8
struct sim7020_conf {
    char apn[APN_SIZE];
    char operator[OPERATOR_SIZE];
};
typedef struct sim7020_conf sim7020_conf_t;

sim7020_conf_t *sim7020_conf(void);
int cmd_sim7020_conf(int argc, char **argv);

void sim7020_conf_init(void);
char *sim7020_conf_apn(void);
char *sim7020_conf_operator(void);
#endif /* SIM7020_CONF_H */
