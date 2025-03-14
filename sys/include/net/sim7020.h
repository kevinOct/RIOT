#ifndef SIM7020_H
#define SIM7020_H

#include "net/netstats.h"

#ifndef SIM7020_UART_DEV
#define SIM7020_UART_DEV UART_DEV(1)
#endif

#ifndef SIM7020_BAUDRATE
#define SIM7020_BAUDRATE 57200
#endif

#ifndef AT_RADIO_MAX_RECV_LEN
#define AT_RADIO_MAX_RECV_LEN 1024
#endif

/* Timeout in seconds to wait for DNS request to complete. Seems to take around 70 seconds to get error code 8 */
#ifndef AT_RADIO_RESOLVE_TIMEOUT
#define AT_RADIO_RESOLVE_TIMEOUT 90
#endif 

#define SIM7020_MAX_SEND_LEN 768

#define SIM7020_MAX_SOCKETS 6
#define SIM7020_NO_SOCKET 7

typedef struct {
    netstats_t ns;
#define tx_unicast_count ns.tx_unicast_count
#define tx_mcast_count ns.tx_mcast_count
#define tx_success ns.tx_success
#define tx_failed ns.tx_failed
#define tx_bytes ns.tx_bytes
#define rx_count ns.rx_count
#define rx_bytes ns.rx_bytes
    uint32_t commfail_count;
    uint32_t reset_count;
    uint32_t activation_count;
    uint32_t activation_fail_count;    
} sim7020_netstats_t;

enum sim_model_id {
    M_SIM7000G,
    M_SIM7020E,
    M_UNKNOWN
} ;
typedef enum sim_model_id sim_model_id_t;

#if defined(SIM7020)
#include "net/sim7020_powerkey.h"
#elif defined(SIM7000G)
#include "net/sim7000g_powerkey.h"
#else
#error SIMCom module undefined
#endif /* SIM7020 */

extern sim_model_id_t sim_model;

#define APN_SIZE 64
#define OPERATOR_NUM_SIZE 8
#define OPERATOR_LONG_SIZE 32

typedef enum {
    SIM7020_CONF_MANUAL_OPERATOR = 1 << 0,
    SIM7020_CONF_MANUAL_APN      = 1 << 1,
} conf_flags_t;
#define SIM7020_CONF_FLAGS_DEFAULT (0)

/*
 * SIM7020 configuration
 */
struct sim7020_conf {
    conf_flags_t flags;
    char apn[APN_SIZE]; /* APN (Access Point Name) */
    char operator[OPERATOR_NUM_SIZE];  /* Operator MCCMNC (mobile country code and mobile network code) */
};
typedef struct sim7020_conf sim7020_conf_t;

/*
 * Structure with operator info
 */ 
struct sim7020_operator {
    char longname[OPERATOR_LONG_SIZE];   /* Long format alphanumeric */
    char numname[OPERATOR_NUM_SIZE];     /* Numeric */
    uint8_t stat;                        /* Status? unknown/available/current/forbidden */
    uint8_t netact;                      /* GSM/LTE/NBIoT */
};
typedef struct sim7020_operator sim7020_operator_t;

/* 
 * Telia: operator 24001 APN "lpwa.telia.iot"
 * Tre: operator 24002 APN "internet"
 * Tele2: operator 24007 APN "4g.tele2.se"
 */

typedef void (* sim7020_recv_callback_t)(void *, const uint8_t *data, uint16_t datalen);

int sim7020_init(void);
int sim7020_stop(void);
int sim7020_reset(void);
int sim7020_register(void);
int sim7020_activate(void);
int sim7020_status(void);
int sim7020_imsi(char *buf, int len);
int sim7020_imei(char *buf, int len);
int sim7020_apn(char *buf, int len);
int sim7020_operator(char *buf, int len);
int sim7020_scan(sim7020_operator_t *op, int first);
int sim7020_udp_socket(const sim7020_recv_callback_t recv_callback, void *recv_callback_arg);
int sim7020_close(uint8_t sockid);
int sim7020_connect(const uint8_t sockid, const sock_udp_ep_t *remote);
int sim7020_bind(const uint8_t sockid, const sock_udp_ep_t *remote);
int sim7020_send(uint8_t sockid, uint8_t *data, size_t datalen);
void *sim7020_recv_thread(void *arg);
int sim7020_resolve(const char *domain, char *result);
void sim7020_setconf(sim7020_conf_t *);
sim7020_netstats_t *sim7020_get_netstats(void);
int sim7020_active(void);
int sim7020_at(const char *cmd);

#endif /* SIM7020_H */
