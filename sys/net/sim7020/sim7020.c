/*
 * Copyright (C) 2020 Peter Sjödin, KTH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "net/af.h"
#include "net/ipv4/addr.h"
#include "net/ipv6/addr.h"
#include "net/sock/udp.h"

#include "async_at.h"
#include "xtimer.h"
#include "cond.h"
#include "periph/uart.h"

#include "net/sim7020.h"
//#include "net/sim7020_powerkey.h"

#define SIM7020_RECVHEX
#define TCPIPSERIALS
#define SIM7020_BIND_ENABLE

#define ASYNC 1

#define FORCE_OPERATOR

at_dev_t at_dev;
static char buf[256];
static char resp[1024];

static struct at_radio_status {
    enum {
        AT_RADIO_STATE_STOPPED,
        AT_RADIO_STATE_RESET,
        AT_RADIO_STATE_NONE,
        AT_RADIO_STATE_INIT,
        AT_RADIO_STATE_IDLE,
        AT_RADIO_STATE_REGISTERED,
        AT_RADIO_STATE_ACTIVE
    } state;
} status = { .state = AT_RADIO_STATE_NONE}; 

sim_model_id_t sim_model;

struct sock_sim7020 {
    uint8_t sockid;
    uint8_t flags;
#define MODULE_OOS 1     /* Module state out of sync with socket state */
    sock_udp_ep_t local;
    sock_udp_ep_t remote;
    sim7020_recv_callback_t recv_callback;
    void *recv_callback_arg;
};
typedef struct sock_sim7020 sim7020_socket_t;

static sim7020_netstats_t netstats;

static sim7020_socket_t sim7020_sockets[SIM7020_MAX_SOCKETS];


mutex_t sim7020_lock = MUTEX_INIT;       /* Exclusive access to device */
static mutex_t trans_mutex = MUTEX_INIT; /* To make request-response transactions atomic */

static unsigned int mlockline, munlockline;
#define P(...) printf(__VA_ARGS__)
//#define P(...)
//#define SIM_LOCK() {P("LOCK<%d> %d: 0x%x\n", thread_getpid(), __LINE__, sim7020_lock.queue.next); mlockline=__LINE__;mutex_lock(&sim7020_lock);}
//#define SIM_UNLOCK() {P("UNLOCK<%d> %d\n", thread_getpid(), __LINE__); munlockline=__LINE__;mutex_unlock(&sim7020_lock);}
#define SIM_LOCK() {mutex_lock(&sim7020_lock);}
#define SIM_UNLOCK() {mutex_unlock(&sim7020_lock);}

//#define TRANS_LOCK() {P("TLOCK<%d> %d: 0x%x\n", thread_getpid(), __LINE__, trans_mutex.queue.next); mlockline=__LINE__;mutex_lock(&trans_mutex);}
//#define TRANS_UNLOCK() {P("TUNLOCK<%d> %d\n", thread_getpid(), __LINE__); munlockline=__LINE__;mutex_unlock(&trans_mutex);}
#define TRANS_LOCK() {mutex_lock(&trans_mutex);}
#define TRANS_UNLOCK() {mutex_unlock(&trans_mutex);}

/* at_process_urc() allocates line buffer on stack, 
 * so make sure there is room for it
 */
static char sim7020_stack[THREAD_STACKSIZE_DEFAULT + AT_BUF_SIZE];

#define SIM7020_PRIO         (THREAD_PRIORITY_MAIN + 2)
static kernel_pid_t sim7020_pid = KERNEL_PID_UNDEF;

static int stopping; /* Module being administratively stopped */

static int _sock_close(uint8_t sockid);
static void *sim7020_thread(void *);

sim7020_netstats_t *sim7020_get_netstats(void) {
    return &netstats;
}

#ifdef TCPIPSERIALS
static void _conn_invalidate(void) {
  uint8_t i;
    for (i = 0; i < SIM7020_MAX_SOCKETS; i++) {
        sim7020_sockets[i].flags |= MODULE_OOS;
    }
}

static uint8_t _conn_alloc(void) {
    uint8_t i;
    for (i = 0; i < SIM7020_MAX_SOCKETS; i++) {
        if (sim7020_sockets[i].recv_callback == NULL) {
            return i;
        }
    }
    printf("No connections\n");
    return SIM7020_NO_SOCKET;
}
#endif

uint32_t sim7020_activation_usecs; /* How many usecs it took to activate uplink */
uint64_t sim7020_active_stamp; /* Timestamp when activatation completed */
uint64_t sim7020_prev_active_duration_usecs; /* How long previous activation lasted */

int sim7020_init(void) {

    sim7xxx_powerkey_init();
    int res = at_dev_init(&at_dev, SIM7020_UART_DEV, SIM7020_BAUDRATE, buf, sizeof(buf));
    if (res != UART_OK) {
        return res;
    }

    if (!pid_is_valid(sim7020_pid)) {
        sim7020_pid = thread_create(sim7020_stack, sizeof(sim7020_stack), SIM7020_PRIO, THREAD_CREATE_STACKTEST,
                                    sim7020_thread, NULL, "sim7020");
        if (!pid_is_valid(sim7020_pid)) {
            printf("launch sim7020: %d\n", sim7020_pid);
            return sim7020_pid;
        }
    }
    status.state = AT_RADIO_STATE_RESET;
    sim7020_activation_usecs = xtimer_now_usec();
    return 0;
}

/* Activation timer to abort activation attempts taking too long
 */

static uint8_t _acttimer_expired_flag = 0;

static void _acttimer_cb(void *arg) {
    (void) arg;
    printf("ACTTIMER EXPIRED\n");
    _acttimer_expired_flag = 1;
}

static xtimer_t _acttimer = {.callback = _acttimer_cb, .arg = NULL};

static void _acttimer_start(void) {
    _acttimer_expired_flag = 0;
    xtimer_set(&_acttimer, 300*US_PER_SEC);
}

static void _acttimer_stop(void) {
    _acttimer_expired_flag = 0;
    xtimer_remove(&_acttimer);
}

static int _acttimer_expired(void) {
    return _acttimer_expired_flag != 0;
}

/*
 * Reset module by powering it off 
 */
static void _module_reset(void) {
    if (sim_model == M_SIM7000G) {
        sim7xxx_power_off();
        //(void) at_send_cmd_wait_ok(&at_dev, "AT+CPOWD=1", 6*US_PER_SEC);
    }
    else
        sim7xxx_power_off();    

    _conn_invalidate();
    netstats.reset_count++;
    status.state = AT_RADIO_STATE_NONE;
    sim7020_activation_usecs = xtimer_now_usec();
    if (sim7020_active_stamp != 0) {
        sim7020_prev_active_duration_usecs = xtimer_now_usec64() - sim7020_active_stamp;
        sim7020_active_stamp = 0;
    }
}

/*
 * Module initialisation -- must be called with module mutex locked 
 */
static int _module_init(void) {
    sim7xxx_power_off();
    xtimer_sleep(2);
    sim7xxx_power_on();
    xtimer_sleep(2);
    int res = 0;
    SIM_LOCK();
    
    {
        char *ate;
        if (IS_ACTIVE(CONFIG_AT_SEND_SKIP_ECHO)) {
            ate = "ATE0";
        }
        else {
            ate = "ATE1";
        }
        int tries = 30;
        while (tries-- > 0) {
            if ((res = at_send_cmd_wait_ok(&at_dev, ate, 1*US_PER_SEC)) >= 0)
                break;
        }
    }
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: AT COMMFAIL\n", __LINE__);
        goto ret;
    }
    res = at_send_cmd_wait_ok(&at_dev, "AT+CPSMS=0", 5000000);
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: CPSMS COMMFAIL\n", __LINE__);
        (void) at_send_cmd_wait_ok(&at_dev, "AT+CPSMS?", 5000000);
        //goto ret;
    }

    /* Request Model Identification */
    char resp[32];
    res = at_send_cmd_get_resp(&at_dev, "AT+CGMM", resp, sizeof(resp), 10*US_PER_SEC);
    
    printf("Model %s\n", resp);
    if (strcmp(resp, "SIM7020E") == 0) {
        sim_model = M_SIM7020E;
    }
    else if (strcmp(resp, "SIMCOM_SIM7000G") == 0) {
        sim_model = M_SIM7000G;
    }
    else {
        printf("Model unknown: \"%s\"\n", resp);
        sim_model = M_UNKNOWN;
    }
    
    /* Limit bands to speed up roaming */
    /* WIP needs a generic solution */
    //res = at_send_cmd_wait_ok(&at_dev, "AT+CBAND=20", 5000000);

#ifdef SIM7020_RECVHEX

    /* Receive data as hex string */
#ifdef TCPIPSERIALS
    /* Show Data in Hex Mode of a Package Received */
    //res = at_send_cmd_wait_ok(&at_dev, "AT+CIPHEXS=2", 5000000);
#else
    res = at_send_cmd_wait_ok(&at_dev, "AT+CSORCVFLAG=0", 5000000);
#endif /* TCPIPSERIALS */
#else  
#ifdef TCPIPSERIALS
    /* Show Data in Hex Mode of a Package Received */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPHEXS=0", 5000000);
#else
    /* Receive binary data */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CSORCVFLAG=1", 5000000);
#endif /* TCPIPSERIALS */
#endif /* SIM7020_RECVHEX */
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
    }
    /* Signal Quality Report */
    res = at_send_cmd_get_resp(&at_dev, "AT+CSQ", resp, sizeof(resp), 10*US_PER_SEC);

    /* Wipe any open sockets */
    uint8_t i;
    for (i = 0; i < SIM7020_MAX_SOCKETS; i++) {
        _sock_close(i);
    }
    res = at_send_cmd_wait_ok(&at_dev, "AT+IPR?", 5000000);
    if (sim_model == M_SIM7000G) {
        res = at_send_cmd_wait_ok(&at_dev, "AT+CNMP=38", 5000000);
        res = at_send_cmd_wait_ok(&at_dev, "AT+CMNB=?", 5000000);
        res = at_send_cmd_wait_ok(&at_dev, "AT+CMNB?", 5000000);
        /* NB-IOT */
        //res = at_send_cmd_wait_ok(&at_dev, "AT+CMNB=2", 5000000);
        /* Cat-M */
        res = at_send_cmd_wait_ok(&at_dev, "AT+CMNB=1", 5000000);
        res = at_send_cmd_wait_ok(&at_dev, "AT+CBANDCFG=\"NB-IOT\",1,3,5,8,20,28", 5000000);

        /* Echo mode */
        if (CONFIG_AT_SEND_SKIP_ECHO)
            res = at_send_cmd_wait_ok(&at_dev, "ATE0", 500000);
        else
            res = at_send_cmd_wait_ok(&at_dev, "ATE1", 500000);
    }

    status.state = AT_RADIO_STATE_IDLE;
ret:
    SIM_UNLOCK();
    return res;
}

int sim7020_stop(void) {
    stopping = 1;
    return 0;
}

int sim7020_reset(void) {
    status.state = AT_RADIO_STATE_RESET;
    stopping = 0;
    return 0;
}
/*
 * Register with operator -- must be called with module mutex locked 
 */
/* Operator MCCMNC (mobile country code and mobile network code) */
/* Telia */
#define OPERATOR "24001"
#define APN "lpwa.telia.iot"
#define FORCE_OPERATOR
/* Tre  */
//#define OPERATOR "24002"
//#define APN "internet"
/* Tele2 */
//#define OPERATOR "24007"
//#define APN "4g.tele2.se"


int sim7020_register(void) {
    int res = 0;

    SIM_LOCK();
#ifdef FORCE_OPERATOR
    /* Force operator selection */
    res = at_send_cmd_wait_ok(&at_dev, "AT+COPS=1,2,\"" OPERATOR "\"", 240*US_PER_SEC);
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        SIM_UNLOCK();
        return res;
    }
#endif /* FORCE_OPERATOR */
    while (status.state == AT_RADIO_STATE_IDLE && !_acttimer_expired()) {
        res = at_send_cmd_get_resp(&at_dev, "AT+CREG?", resp, sizeof(resp), 120*US_PER_SEC);
        if (res < 0) {
        printf("%d: COMMFAIL\n", __LINE__);
            netstats.commfail_count++;
            _module_reset();
        }
        else {
            uint8_t creg;
                
#ifndef SCNu8
#define SCNu8 "hhu"
#endif
            if (1 == (sscanf(resp, "%*[^:]: %*" SCNu8 ",%" SCNu8, &creg))) {
                /* Wait for 1 (Registered, home network) or 5 (Registered, roaming) */
                if (creg == 1 || creg == 5) {
                    status.state = AT_RADIO_STATE_REGISTERED;
                    break;
                }
                else if ((sim_model == M_SIM7000G) && (creg == 0)) {
                    /* Is this for NB-IoT only, that we can't trust CREG and need to use CGREG? */
                    res = at_send_cmd_get_resp(&at_dev, "AT+CGREG?", resp, sizeof(resp), 120*US_PER_SEC);
                    if (res < 0) {
                        printf("%d: COMMFAIL\n", __LINE__);
                        netstats.commfail_count++;
                    }
                    else {
                        uint8_t creg;
                        if (1 == (sscanf(resp, "%*[^:]: %*" SCNu8 ",%" SCNu8, &creg))) {
                            /* Wait for 1 (Registered, home network) or 5 (Registered, roaming) */
                            if (creg == 1 || creg == 5) {
                                status.state = AT_RADIO_STATE_REGISTERED;
                                break;
                            }
                        }
                    }
                }
            }
        }
        xtimer_sleep(1);
    }
    /* Wait for GPRS/Packet Domain attach */
    while (status.state == AT_RADIO_STATE_REGISTERED && !_acttimer_expired()) {
        res = at_send_cmd_get_resp(&at_dev, "AT+CGATT=1", resp, sizeof(resp), 120*US_PER_SEC);
        res = at_send_cmd_get_resp(&at_dev, "AT+CGATT?", resp, sizeof(resp), 120*US_PER_SEC);
        if (res > 0) {
            if (0 == (strcmp(resp, "+CGATT: 1")))
                break;
        }
        else {
            netstats.commfail_count++;
            printf("%d: COMMFAIL\n", __LINE__);
        }
        xtimer_sleep(5);
        res = 0;
    }

    SIM_UNLOCK();
    /* Reset if registration did not work */
    if (status.state != AT_RADIO_STATE_REGISTERED)
        status.state = AT_RADIO_STATE_RESET;
    return res;
}

/*
 * Activate data connection 
 */

int sim7020_activate(void) {
    int res;
    uint8_t attempts = 10;
  
    SIM_LOCK();
    /* Set CIPMUX=1 - multiplexed connections */
    /* Check if it is 1 already */
    res = at_send_cmd_get_resp(&at_dev,"AT+CIPMUX?", resp, sizeof(resp), 120*US_PER_SEC);
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        goto ret;
    }
    at_drain(&at_dev);
    /* Set to 1 */
    if (strcmp(resp, "+CIPMUX: 1") != 0)
        res = at_send_cmd_wait_ok(&at_dev,"AT+CIPMUX=1", 120*US_PER_SEC);
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        goto ret;
    }
    at_drain(&at_dev);

    if (sim_model == M_SIM7000G) {
        /* Skip APN, if not using GPRS */
        /* Automatic mode -- get APN from base station */
        (void) at_send_cmd_wait_ok(&at_dev, "AT+CAPNMODE=0", 5*US_PER_SEC);
        /* Get Network APN in CAT-M Or NB-IOT */
        (void) at_send_cmd_get_resp(&at_dev, "AT+CGNAPN", resp, sizeof(resp), 5000000);
        /* Result scanned as: sscanf(resp, "+CGNAPN: 1,\"%31[^\"]\"", apn) */
    }

    /* Start Task and set APN */
    res = at_send_cmd_get_resp(&at_dev,"AT+CSTT=\"" APN "\",\"\",\"\"", resp, sizeof(resp), 120*US_PER_SEC);
    /* Check APN */
    res = at_send_cmd_get_resp(&at_dev,"AT+CSTT?", resp, sizeof(resp), 60*US_PER_SEC);
    /* Look for '+CSTT: "APN","USER","PWD"'
     * Result scanned as: sscanf(resp, "+CSTT: \"%31[^\"]\",\"%*[^\"]\",\"%*[^\"]\"", apn)
     */

    at_drain(&at_dev);
    while (!_acttimer_expired() && attempts--) {
        /* Bring Up Wireless Connection with GPRS or CSD. This may take a while. */
        printf("Bringing up wireless, be patient\n");
        res = at_send_cmd_wait_ok(&at_dev, "AT+CIICR", 600*US_PER_SEC);
        (void) at_send_cmd_wait_ok(&at_dev, "AT+CIPSTATUS", 5*US_PER_SEC);
        if (res == 0) {
            break;
        }
    }
    res = at_send_cmd_get_resp(&at_dev, "AT+CIFSR", resp, sizeof(resp), 5*US_PER_SEC);
    if (res >= 0) {
        ipv4_addr_t v4addr;

        res = sscanf(resp, "%" SCNu8 ".%" SCNu8 ".%" SCNu8 ".%" SCNu8,
                     &v4addr.u8[0], &v4addr.u8[1], &v4addr.u8[2], &v4addr.u8[3]);
        if (res == 4) {
            status.state = AT_RADIO_STATE_ACTIVE;
            goto active;
        }
    }
    /* If we end up here, we could not get local IP. Take this as indication
     * that activation failed.
     */
    printf("No IP\n");
    status.state = AT_RADIO_STATE_RESET;
    goto ret;

    //res = at_send_cmd_wait_ok(&at_dev, "AT+CIPPING=\"192.16.125.232\"", 5000000);

active:
    /* Show Data in Hex Mode of a Package Received */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPHEXS=2", 5000000);
    /* Show remote address and port on receive */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPSRIP=1", 5000000);
ret:
    SIM_UNLOCK();
    return res;
}

int sim7020_status(void) {
    int res;

    printf("SIM7020 status: %u (active == %u)\n", status.state, sim7020_active());
    SIM_LOCK();
    if (0) {
        printf("Searching for operators, be patient\n");
        res = at_send_cmd_get_resp(&at_dev, "AT+COPS=?", resp, sizeof(resp), 120*US_PER_SEC);
    }
    res = at_send_cmd_get_resp(&at_dev, "AT+CREG?", resp, sizeof(resp), 120*US_PER_SEC);
    /* Request International Mobile Subscriber Identity */
    res = at_send_cmd_get_resp(&at_dev, "AT+CIMI", resp, sizeof(resp), 10*US_PER_SEC);

    /* Request TA Serial Number Identification (IMEI) */
    res = at_send_cmd_get_resp(&at_dev, "AT+GSN", resp, sizeof(resp), 10*US_PER_SEC);

    /* Mode 0: Radio information for serving and neighbor cells */
    res = at_send_cmd_wait_ok(&at_dev,"AT+CENG=0", 60*US_PER_SEC);
    /* Report Network State */
    res = at_send_cmd_get_resp(&at_dev,"AT+CENG?", resp, sizeof(resp), 60*US_PER_SEC);

    res = at_send_cmd_wait_ok(&at_dev,"AT+CENG=1", 60*US_PER_SEC);
    /* Report Network State */
    res = at_send_cmd_get_resp(&at_dev,"AT+CENG?", resp, sizeof(resp), 60*US_PER_SEC);

    /* Signal Quality Report */
    res = at_send_cmd_wait_ok(&at_dev,"AT+CSQ", 60*US_PER_SEC);
    /* Task status, APN */
    res = at_send_cmd_get_resp(&at_dev,"AT+CSTT?", resp, sizeof(resp), 60*US_PER_SEC);

    res = at_send_cmd_get_resp(&at_dev,"AT+CIPSTATUS", resp, sizeof(resp), 60*US_PER_SEC);
    /* Get Local IP Address */
    res = at_send_cmd_get_resp(&at_dev,"AT+CIFSR", resp, sizeof(resp), 60*US_PER_SEC);
    /* PDP Context Read Dynamic Parameters */
    res = at_send_cmd_get_resp(&at_dev,"AT+CGCONTRDP", resp, sizeof(resp), 60*US_PER_SEC);
    /* Show PDP Address */
    res = at_send_cmd_get_resp(&at_dev,"AT+CGPADDR", resp, sizeof(resp), 60*US_PER_SEC);
    /* Define PDP Context */
    res = at_send_cmd_get_resp(&at_dev,"AT+CGDCONT", resp, sizeof(resp), 60*US_PER_SEC);

    /* Request TA Model Identification */
    res = at_send_cmd_get_resp(&at_dev,"AT+GMM", resp, sizeof(resp), 60*US_PER_SEC);
    /* Request Manufacturer Identification */
    res = at_send_cmd_get_resp(&at_dev,"AT+CGMI", resp, sizeof(resp), 60*US_PER_SEC);
    /* Request Model Identification */
    res = at_send_cmd_get_resp(&at_dev,"AT+CGMM", resp, sizeof(resp), 60*US_PER_SEC);
    SIM_UNLOCK();
    return res;
}


#define MAX_IMSI_LEN 16
int sim7020_imsi(char *buf, int len) {
    (void) buf; (void) len;
    /* Request International Mobile Subscriber Identity */
    int res = at_send_cmd_get_resp(&at_dev, "AT+CIMI", resp, sizeof(resp), 10*US_PER_SEC);
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        return 0;
    }
    else {
        strncpy(buf, resp, len);
        return strnlen(resp, len);
    }
}

#define MAX_IMEI_LEN 17
int sim7020_imei(char *buf, int len) {
    (void) buf; (void) len;
    /* Request International Mobile Equipment Identity */
    int res = at_send_cmd_get_resp(&at_dev, "AT+GSN", resp, sizeof(resp), 10*US_PER_SEC);
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        return 0;
    }
    else {
        strncpy(buf, resp, len);
        return strnlen(resp, len);
    }
}

int sim7020_udp_socket(const sim7020_recv_callback_t recv_callback, void *recv_callback_arg) {
    /* Create a socket: IPv4, UDP, 1 */

#ifdef TCPIPSERIALS
    uint8_t sockid = _conn_alloc();
    if (sockid == SIM7020_NO_SOCKET)
        return -ENOMEM;
    sim7020_socket_t *sock = &sim7020_sockets[sockid];
    sock_udp_ep_t sock_udp_any = SOCK_IPV6_EP_ANY;
    sock->local = sock_udp_any;
    sock->remote = sock_udp_any;    
    sock->recv_callback = recv_callback;
    sock->recv_callback_arg = recv_callback_arg;
    return sockid;
#else
    int res;
    SIM_LOCK();
    res = at_send_cmd_get_resp(&at_dev, "AT+CSOC=1,2,1", resp, sizeof(resp), 120*US_PER_SEC);
    SIM_UNLOCK();

    if (res > 0) {
        uint8_t sockid;

        if (1 == (sscanf(resp, "+CSOC: %" SCNu8, &sockid))) {
            assert(sockid < SIM7020_MAX_SOCKETS);
            sim7020_socket_t *sock = &sim7020_sockets[sockid];
            sock->recv_callback = recv_callback;
            sock->recv_callback_arg = recv_callback_arg;
            return sockid;
        }
        else {
            if (strcmp("ERROR", resp) == 0)
                return -ENOMEM;
            printf("CSOC parse error: '%s'\n", resp);
        }
    }
    else {
        printf("CSOC failed: %d\n", res);
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        at_drain(&at_dev);
        _module_reset();
    }        
    return res;
#endif /* TCPIPSERIALS */
}

static int _sock_close(uint8_t sockid) {

    int res;
    char cmd[64];

    assert(sockid < SIM7020_MAX_SOCKETS);
#ifdef TCPIPSERIALS
    /* Quick close */
    sprintf(cmd, "AT+CIPCLOSE=%d,1", sockid);
#else
    sprintf(cmd, "AT+CSOCL=%d", sockid);
#endif /* TCPIPSERIALS */
    if ((thread_getpid() == sim7020_pid)) {
        res = at_send_cmd_wait_ok(&at_dev, cmd, 10*US_PER_SEC);
    }
    else {
        TRANS_LOCK();
        char resp[sizeof("NN, CLOSE OK")];
        snprintf(resp, sizeof(resp), "%u, CLOSE OK", sockid);
        res = async_at_send_cmd_wait_resp(&at_dev, cmd, resp, 10*US_PER_SEC);
        TRANS_UNLOCK();
    }
    return res;
}

int sim7020_close(uint8_t sockid) {

    printf("sim7020_close %d\n", sockid);
    int res = _sock_close(sockid);
    sim7020_socket_t *sock = &sim7020_sockets[sockid];
    sock->recv_callback = NULL;
    return res;
}


#ifdef TCPIPSERIALS
int _sock_connect(uint8_t sockid, const sock_udp_ep_t *remote) {

    int res;
    char cmd[64];

    char *c = cmd;
    int len = sizeof(cmd);
    int n = snprintf(c, len, "AT+CIPSTART=%u,\"UDP\",", sockid);
    //int n = snprintf(c, len, "AT+CIPSTART= \"UDP\",");
    c += n; len -= n;
    ipv6_addr_t *v6addr = (ipv6_addr_t *) &remote->addr.ipv6;
    ipv4_addr_t *v4addr = IPV4_ADDR_IPV6_MAPPED(v6addr);
    if (NULL == ipv4_addr_to_str(c, v4addr, len)) {
        printf("_sock_connect: bad IPv4 mapped address: ");
        ipv6_addr_print((ipv6_addr_t *) v6addr);
        printf("\n");
        return -1;
    }
    len -= strlen(c);
    c += strlen(c);
    snprintf(c, len, ",%u", remote->port);
    /* Create a socket: IPv4, UDP, 1 */

    char resp[sizeof("NN, CONNECT OK")];
    snprintf(resp, sizeof(resp), "%u, CONNECT OK", sockid);
    /* Should perhaps also check for "ALREADY CONNECT" */
    TRANS_LOCK();
    res = async_at_send_cmd_wait_resp(&at_dev, cmd, resp, 10*US_PER_SEC);
    TRANS_UNLOCK();
    if (res < 0) {
        return res;
    }
    else
        return 0;
}

int sim7020_connect(uint8_t sockid, const sock_udp_ep_t *remote) {

    assert(sockid < SIM7020_MAX_SOCKETS);

    if (remote->family != AF_INET6) {
        return -EAFNOSUPPORT;
    }
    if (!ipv6_addr_is_ipv4_mapped((ipv6_addr_t *) &remote->addr.ipv6)) {
        printf("sim7020_connect: not ipv6 mapped ipv4: ");
        ipv6_addr_print((ipv6_addr_t *) &remote->addr.ipv6);
        printf("\n");
        return -EINVAL;
    }
    if (remote->port == 0) {
        return -EINVAL;
    }
    sim7020_socket_t *sock = &sim7020_sockets[sockid];
    if (memcmp(&sock->remote, remote, sizeof(sock_udp_ep_t)) != 0)
        sock->flags |= MODULE_OOS;
    sock->remote = *remote;
    return 0;
}

#else
int sim7020_connect(uint8_t sockid, const sock_udp_ep_t *remote) {

    int res;
    char cmd[64];

    assert(sockid < SIM7020_MAX_SOCKETS);

    if (remote->family != AF_INET6) {
        return -EAFNOSUPPORT;
    }
    if (!ipv6_addr_is_ipv4_mapped((ipv6_addr_t *) &remote->addr.ipv6)) {
        printf("sim7020_connect: not ipv6 mapped ipv4: ");
        ipv6_addr_print((ipv6_addr_t *) &remote->addr.ipv6);
        printf("\n");
        return -1;
    }
    if (remote->port == 0) {
        return -EINVAL;
    }

    char *c = cmd;
    int len = sizeof(cmd);
    int n = snprintf(c, len, "AT+CSOCON=%d,%d,", sockid, remote->port);
    c += n; len -= n;
    ipv6_addr_t *v6addr = (ipv6_addr_t *) &remote->addr.ipv6;
    ipv4_addr_t *v4addr = (ipv4_addr_t *) &v6addr->u32[3];
    if (NULL == ipv4_addr_to_str(c, v4addr, len)) {
        printf("connect: bad IPv4 mapped address: ");
        ipv6_addr_print((ipv6_addr_t *) &remote->addr.ipv6);
        printf("\n");
        return -1;
    }

    /* Create a socket: IPv4, UDP, 1 */
    SIM_LOCK();
    res = at_send_cmd_wait_ok(&at_dev, cmd, 120*US_PER_SEC);
    SIM_UNLOCK();
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        _module_reset();
    }
    return res;
}
#endif

static int _sock_bind(uint8_t sockid, const sock_udp_ep_t *local) {

#ifdef SIM7020_BIND_ENABLE
    printf("binding to to ipv6 mapped ");
    ipv6_addr_print((ipv6_addr_t *) &local->addr.ipv6);
    printf(":%u\n", local->port);

    int res;
    char cmd[64];

    TRANS_LOCK();
#ifdef TCPIPSERIALS    
    snprintf(cmd, sizeof(cmd), "AT+CLPORT=%hu,\"UDP\",%hu",sockid, local->port);
    res = async_at_send_cmd_wait_ok(&at_dev, cmd, 10*US_PER_SEC);
    TRANS_UNLOCK();

    if (res < 0) {
        return res;
    }
    return 0;
#else
    int res;
    char cmd[64];
    char *c = cmd;
    int len = sizeof(cmd);
    int n = snprintf(c, len, "AT+CSOB=%d,%u", sockid, local->port);

    c += n; len -= n;
    if (!ipv6_addr_is_unspecified((ipv6_addr_t *) &local->addr.ipv6)) {
        ipv6_addr_t *v6addr = (ipv6_addr_t *) &local->addr.ipv6;
        ipv4_addr_t *v4addr = (ipv4_addr_t *) &v6addr->u32[3];
        n = snprintf(c, len, ",");
        c += n; len -= n;
        if (NULL == ipv4_addr_to_str(c, v4addr, len)) {
            printf("bind: bad IPv4 mapped address: ");
            ipv6_addr_print((ipv6_addr_t *) &local->addr.ipv6);
            printf("\n");
            return -1;
        }
    }
    else {
        printf("Bind: unspecified\n");
    }
    SIM_LOCK();

    res = at_send_cmd_wait_ok(&at_dev, cmd, 120*US_PER_SEC);
    (void) at_send_cmd_wait_ok(&at_dev, "AT+CSOCON?", 120*US_PER_SEC);
    res = at_send_cmd_wait_ok(&at_dev, "AT+CSOB?", 120*US_PER_SEC);

    SIM_UNLOCK();
    printf("socket bound: %d\n", res);
    
    return res;
#endif /* TCPIPSERIALS */
#else
    (void) sockid;
    return 0;
#endif /* SIM7020_BIND_ENABLE */
}

int sim7020_bind(uint8_t sockid, const sock_udp_ep_t *local) {

    assert(sockid < SIM7020_MAX_SOCKETS);
    if (local->family != AF_INET6) {
        return -EAFNOSUPPORT;
    }

    if (local->port == 0) {
        return -EINVAL;
    }
    
    sim7020_socket_t *sock = &sim7020_sockets[sockid];
    if (sock->local.port != local->port) {
        sock->flags |= MODULE_OOS;
        sock->local = *local;
    }
    return 0;
}

uint32_t longest_send;

int sim7020_send(uint8_t sockid, uint8_t *data, size_t datalen) {
    int res;

    char cmd[32];
    size_t len = datalen;
    
    if (status.state != AT_RADIO_STATE_ACTIVE)
        return -ENETUNREACH;

    if (len > SIM7020_MAX_SEND_LEN)
        return -EINVAL;

    assert(sockid < SIM7020_MAX_SOCKETS);

    /* On-demand synchronization of socket state and
     * module state. Bind and connect if required
     */
    sim7020_socket_t *sock = &sim7020_sockets[sockid];
    if (sock->flags & MODULE_OOS) {
        if ((res = _sock_bind(sockid, &sock->local)) < 0)
            goto fail;
        if ((res = _sock_connect(sockid, &sock->remote)) < 0)
            goto fail;
        sock->flags &= ~MODULE_OOS;
    }

#ifdef TCPIPSERIALS

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u", sockid, len);
    //snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", len);
#else
    snprintf(cmd, sizeof(cmd), "AT+CSODSEND=%d,%d", sockid, len);
#endif
    TRANS_LOCK();
    res = async_at_send_cmd_wait_resp(&at_dev, cmd, "> ", 10*US_PER_SEC);
    if (res != 0) {
        SIM_UNLOCK();
        printf("AA sendwait fail: %d\n", res);
        goto fail;
    }
#ifdef TCPIPSERIALS
    SIM_LOCK();
    at_send_bytes(&at_dev, (char *) data, len);
    SIM_UNLOCK();
    {
        uint32_t start, stop;
        start = xtimer_now_usec();

        async_at_t async_at;
        snprintf(cmd, sizeof(cmd), "%u, SEND OK", sockid);
        async_at_setup(&async_at, async_at_null_cb, NULL, cmd, 10*US_PER_SEC);
        async_at.cmd = "<SEND>";
        res = async_at_wait(&async_at);
        async_at_stop(&async_at);
        stop = xtimer_now_usec();
        if (res >= 0) {
            uint32_t wait;
            wait = stop - start;
            if (wait > longest_send) {
                longest_send = wait;
                printf("Longest send %" PRIu32 "\n", longest_send);
            }
        }
    }
    if (res >= 0) {
        res = len;
        netstats.tx_success++;
        netstats.tx_unicast_count++;
        netstats.tx_bytes += datalen;
        goto out;
    }
    /* else fall through */
    printf("res %d\n", res);
#else
    SIM_LOCK();
    at_send_bytes(&at_dev, (char *) data, len);
    /* Skip empty line */
    (void) at_readline(&at_dev, resp, sizeof(resp), 0, 10*US_PER_SEC);
    /* Then look for DATA ACCEPT confirmarion */
    res = at_readline(&at_dev, resp, sizeof(resp), 0, 10*US_PER_SEC);
    if (res < 0) {
        printf("Timeout waiting for DATA ACCEPT confirmation\n");
        goto fail;
    }
    unsigned uint16_t nsent;
    if (1 == (sscanf(resp, "DATA ACCEPT: %" SCNu16, &nsent))) {
        res = nsent;
        netstats.tx_success++;
        netstats.tx_unicast_count++;
        netstats.tx_bytes += datalen;
        goto out;
    }
    printf("No accept: %s\n", resp);
    /* else fall through */
#endif
fail:
    netstats.tx_failed++;
    netstats.commfail_count++;
    printf("%d: COMMFAIL\n", __LINE__);

    res = async_at_send_cmd_wait_ok(&at_dev, "AT", 1*US_PER_SEC);
    if (res == -1)
        _module_reset();
    else
        printf("All good %d\n", res);
out:
    TRANS_UNLOCK();
    return res;

}

static mutex_t resolve_mutex; /* Resolve one domain name at a time */

#define URC_POLL_MSECS 100

static void _async_resolve_cb(void *async_at, void *arg, const char *code) {
    async_at_t *aap = (async_at_t *) async_at;
    char *result = arg;
    const char *resp = code;
    char buf[64];

    at_readline(&at_dev, buf, sizeof(buf), true, URC_POLL_MSECS*(uint32_t) 1000);
    if (1 == sscanf(buf, " 1,\"%*[^\"]\",\"%[^\"]", result)) {
        aap->state = R_DONE;
    }
    else {
        uint8_t errcode;
        if (1 == sscanf(resp, "+CDNSGIP: 0,%" SCNu8, &errcode)) {
            printf("Resolve error %" PRIu8, errcode);
        }
        aap->state = R_ERROR;
    }
}

static ipv4_addr_t get_resolver(void) {
    char buf[64];
    ipv4_addr_t v4addr = {.u32 = 0};
    int res;
    TRANS_LOCK();
    SIM_LOCK();
    res = at_send_cmd_get_resp(&at_dev, "AT+CDNSCFG?", buf, sizeof(buf), 5*US_PER_SEC);
    at_drain(&at_dev);
    SIM_UNLOCK();
    TRANS_UNLOCK();
    if (res >= 0) {
        res = sscanf(buf, "PrimaryDns: %" SCNu8 ".%" SCNu8 ".%" SCNu8 ".%" SCNu8,
                     &v4addr.u8[0], &v4addr.u8[1], &v4addr.u8[2], &v4addr.u8[3]);
        if (res < 0)
            printf("Resolver ail\n");
    }
    return v4addr;
}

static int set_resolver(ipv4_addr_t *resolveaddr) {
    char cmd[64];
    char v4str[sizeof("255.255.255.255")];
    at_drain(&at_dev);
    snprintf(cmd, sizeof(cmd), "AT+CDNSCFG=\"%s\",\"0.0.0.0\"", ipv4_addr_to_str(v4str, resolveaddr, sizeof(v4str)));
    return at_send_cmd_wait_ok(&at_dev, cmd, 5*US_PER_SEC);
}

static ipv4_addr_t default_resolver = {.u8[0] = 8, .u8[1] = 8, .u8[2] = 8, .u8[3] = 8};

int sim7020_resolve(const char *domain, char *result) {
    int res;
    async_at_t async_at;

    if (status.state != AT_RADIO_STATE_ACTIVE)
        return -ENETUNREACH;

    /* Only one at a time */
    mutex_lock(&resolve_mutex);

#if 0
    SIM_LOCK();
    /* Check signal quality */
    (void) at_send_cmd_wait_ok(&at_dev, "AT+CSQ", 10*US_PER_SEC);
    (void) at_send_cmd_wait_ok(&at_dev, "AT+CDNSCFG?", 10*US_PER_SEC);
    at_drain(&at_dev);
    SIM_UNLOCK();
#endif
    ipv4_addr_t resolveaddr = get_resolver();
    if (resolveaddr.u32.u32 == (uint32_t) 0) {
        set_resolver(&default_resolver);
    }
    async_at.cmd = "<CDNSGIP>";
    TRANS_LOCK();
    async_at_setup(&async_at, _async_resolve_cb, result, "+CDNSGIP:", AT_RADIO_RESOLVE_TIMEOUT*US_PER_SEC);
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CDNSGIP=%s", domain);
    res = async_at_send_cmd_wait_ok(&at_dev, cmd, 6*US_PER_SEC);
    if (res < 0) {
        netstats.commfail_count++;
        printf("%d: COMMFAIL\n", __LINE__);
        _module_reset();
        goto out;
    }
    res = async_at_wait(&async_at);

out:
    TRANS_UNLOCK();
    mutex_unlock(&resolve_mutex);
    async_at_stop(&async_at);
    return res;
}

static uint8_t recv_buf[AT_RADIO_MAX_RECV_LEN];
#ifdef TCPIPSERIALS
static void _receive_cb(void *arg, const char *code) {
    (void) arg;

    sock_udp_ep_t remote = { .family = AF_INET6};
    ipv6_addr_t *v6addr = (ipv6_addr_t *) &remote.addr.ipv6;
    *v6addr = ipv6_addr_ipv4_mapped_prefix;
    ipv4_addr_t *v4addr = IPV4_ADDR_IPV6_MAPPED(v6addr);
    uint8_t sockid;
    uint16_t len;
    int res;

    /*
     * Process something like "+RECEIVE,0,14,192.16.125.232:10000"
     */

    (void) code; /* Matched with "+RECEIVE," */
    char buf[64];
    at_readline(&at_dev, buf, sizeof(buf), 0, URC_POLL_MSECS*(uint32_t) 1000);
    res = sscanf(buf, "%" SCNu8 ",%" SCNu16 ",%" SCNu8 ".%" SCNu8 ".%" SCNu8 ".%" SCNu8 ":%" SCNu16,
                 &sockid, &len,
                 &v4addr->u8[0], &v4addr->u8[1], &v4addr->u8[2], &v4addr->u8[3], &remote.port);
    if (res == 7) {
#ifdef SIM7020_RECVHEX
        res = at_readline(&at_dev, (char *) recv_buf, sizeof(recv_buf), 0, 10*US_PER_SEC);
        /* Data is encoded as hex string, so
         * data length is half the string length */ 
        int rcvlen = len >> 1;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        char *ptr = (char *) recv_buf;
        /* Copy into receive buffer */
        for (int i = 0; i < rcvlen; i++) {
            char hexstr[3];
            hexstr[0] = *ptr++; hexstr[1] = *ptr++; hexstr[2] = '\0';
            recv_buf[i] = (uint8_t) strtoul(hexstr, NULL, 16);
        }
#else
        /* Data is binary */
        int rcvlen = len;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        uint8_t *ptr = recv_buf;
        /* Copy into receive buffer */
        memcpy(recv_buf, ptr, rcvlen);
#endif /* SIM7020_RECVHEX */

        if (sockid >= SIM7020_MAX_SOCKETS) {
            printf("Illegal sim socket %" PRIu8 "\n", sockid);
            return;
        }
        sim7020_socket_t *sock = &sim7020_sockets[sockid];
        if (sock->recv_callback != NULL) {
            sock->recv_callback(sock->recv_callback_arg, recv_buf, rcvlen);
        }
        else {
            printf("sockid %d: no callback\n", sockid);
        }
        netstats.rx_count++;
        netstats.rx_bytes += rcvlen;
    }
    else
        printf("receive_cb res %d\n", res);
}
#else
static void _csonmi_cb(void *arg, const char *code) {
    (void) arg;
    uint8_t sockid;
    uint16_t len;
    int res;

    res = sscanf(code, "+CSONMI: %" SCNu8 ",%" SCNu16 ",", &sockid, &len);
    if (res == 2) {

#ifdef SIM7020_RECVHEX

        /* Data is encoded as hex string, so
         * data length is half the string length */ 
        int rcvlen = len >> 1;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        /* Find first char after second comma */
        char *ptr = strchr(strchr(code, ',')+1, ',')+1;

        /* Copy into receive buffer */
        for (int i = 0; i < rcvlen; i++) {
            char hexstr[3];
            hexstr[0] = *ptr++; hexstr[1] = *ptr++; hexstr[2] = '\0';
            recv_buf[i] = (uint8_t) strtoul(hexstr, NULL, 16);
        }
#else
        /* Data is binary */
        int rcvlen = len;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        /* Find first char after second comma */
        char *ptr = strchr(strchr(code, ',')+1, ',')+1;

        /* Copy into receive buffer */
        memcpy(recv_buf, ptr, rcvlen);
#endif /* SIM7020_RECVHEX */

#if 0
        for (int i = 0; i < rcvlen; i++) {
            if (isprint(recv_buf[i]))
                putchar(recv_buf[i]);
            else
                printf("x%02x", recv_buf[i]);
            putchar(' ');
        }
        putchar('\n');
#endif
        if (sockid >= SIM7020_MAX_SOCKETS) {
            printf("Illegal sim socket %d\n", sockid);
            return;
        }
        sim7020_socket_t *sock = &sim7020_sockets[sockid];
        if (sock->recv_callback != NULL) {
            sock->recv_callback(sock->recv_callback_arg, recv_buf, rcvlen);
        }
        else {
            printf("sockid %d: no callback\n", sockid);
        }
        netstats.rx_count++;
        netstats.rx_bytes += rcvlen;
    }
    else
        printf("csonmi_cb res %d\n", res);
}

#endif /* TCPIPSERIALS */

static int led(int on) {
    if (on)
        LED0_ON;
    else
        LED0_OFF;
    return on;
}

static void _recv_loop(void) {
    at_urc_t urc;

#ifdef TCPIPSERIALS
    urc.cb = _receive_cb;
    urc.code = "+RECEIVE,";
#else
    urc.cb = _csonmi_cb;
    urc.code = "+CSONMI:";
#endif 
    urc.arg = NULL;
    at_add_urc(&at_dev, &urc);
    int ledon = 0;
    while (status.state == AT_RADIO_STATE_ACTIVE) {
        void at_process_urc_byte(at_dev_t *dev, uint32_t timeout);
        ledon = led(!ledon);
        SIM_LOCK();
        at_process_urc_byte(&at_dev, 2*URC_POLL_MSECS*(uint32_t) 1000);
        SIM_UNLOCK();
    }
    at_remove_urc(&at_dev, &urc);
}



static void blink(int blinks) {
    led(0);
    for (int i = 0; i < blinks; i++) {
        led(1);
        xtimer_usleep(US_PER_SEC/5);
        led(0);
        xtimer_usleep(US_PER_SEC/5);
    }
}
static void *sim7020_thread(void *arg) {
    (void) arg;

    while (1) {
        if (stopping) {
            printf("***stopping\n");
            status.state = AT_RADIO_STATE_STOPPED;
            stopping = 0;
        }
        switch (status.state) {
        case AT_RADIO_STATE_STOPPED:
            xtimer_sleep(1);
            break;
        case AT_RADIO_STATE_RESET:
            printf("***module reset:\n");
            _module_reset();
            break;
        case AT_RADIO_STATE_NONE:
            blink(1);
            _acttimer_start();
            status.state = AT_RADIO_STATE_INIT;
            break;
        case AT_RADIO_STATE_INIT:
            blink(2);
            printf("***module init:\n");
            _module_init();
            break;
        case AT_RADIO_STATE_IDLE:
            blink(3);
            printf("***register:\n");
            sim7020_register();
            break;
        case AT_RADIO_STATE_REGISTERED:
            blink(4);
            printf("***activate:\n");
            sim7020_activate();
            if (status.state == AT_RADIO_STATE_ACTIVE) {
                netstats.activation_count++;
                sim7020_activation_usecs = xtimer_now_usec() - sim7020_activation_usecs;
                sim7020_active_stamp = xtimer_now_usec64();
                _acttimer_stop();
                SIM_UNLOCK();
            }
            break;
        case AT_RADIO_STATE_ACTIVE:
            printf("***recv loop:\n");
            _recv_loop();
            break;
        }
        if (status.state != AT_RADIO_STATE_ACTIVE && _acttimer_expired()) {
            netstats.activation_fail_count++;
            printf("timer expired during activatiion\n");
            _module_reset();
        }

    }
    return NULL;
}

int sim7020_active(void) {
    return status.state == AT_RADIO_STATE_ACTIVE;
}

int sim7020_at(const char *cmd) {
    printf("Do command '%s'\n", cmd);
    if (sim7020_lock.queue.next) {
        printf("Warning: mutex locked %u (last unlock %u)\n", mlockline, munlockline);
    }
    //SIM_LOCK();
    at_send_cmd(&at_dev, cmd, 10*US_PER_SEC);
    //SIM_UNLOCK();
    return 0;
}

//int sim7020cmd_test(uint8_t sockid, int count) {
int sim7020_test(void) {
    return 0;
}
