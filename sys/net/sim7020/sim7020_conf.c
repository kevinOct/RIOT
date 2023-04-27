/*
 * Copyright (C) 2023 Peter Sjödin, KTH
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

#include <avr/eeprom.h>

#include "net/af.h"
#include "net/ipv4/addr.h"
#include "net/ipv6/addr.h"
#include "net/sock/udp.h"

#include "async_at.h"
#include "xtimer.h"
#include "cond.h"
#include "periph/uart.h"

#include "net/sim7020.h"
#include "net/sim7020_conf.h"
//#include "net/sim7020_powerkey.h"

#ifdef BOARD_AVR_RSS2
#include "pstr_print.h"
#endif

#include "eedata.h"

sim7020_conf_t conf = {
    .flags = 0,
    .apn = "4g.tele2.se",
    .operator = "24007"
};

static EEMEM struct {
    eehash_t ee_hash;
    sim7020_conf_t ee_conf;
} ee_data;

    
#include <string.h>
#include "hashes.h"

#if 0
static void hdump(uint8_t data[], size_t len) {
    size_t i;
    for (i = 0; i < len; i++)
        printf("%02x ", (uint8_t) data[i]);
}
#endif

static void printconf(void) {
    printf("apn: %s\n", conf.apn);
    printf("operator: %s\n", conf.operator);
}

static void _apply(void) {
    sim7020_setconf(&conf);
}

void sim7020_conf_init(void) {
    sim7020_conf_t confdata;
    int n = read_eeprom(&confdata, &ee_data, sizeof(confdata));
    if (n != 0) {
        memcpy(&conf, &confdata, sizeof(conf));
        printf("Apply SIM config: \n");
        printconf();
        _apply();
    }
    else {
      printf("No SIM config\n");
    }
}

/* 
 * Operator MCCMNC (mobile country code and mobile network code) 
 */
char *sim7020_conf_operator(void) {
    return "24007";
}

#define MINMATCH 2
int cmd_sim7020_conf(int argc, char **argv) {
    if (argc == 1) {
        printconf();
        return 1;
    }
    else if (argc >= 2) {
        if (strncmp(argv[1], "apn", MINMATCH) == 0) {
            strncpy(conf.apn, argv[2], sizeof(conf.apn));
            conf.flags |= SIM7020_CONF_MANUAL_APN;
        }
        else if (strncmp(argv[1], "clear", MINMATCH) == 0) {
            erase_eeprom(&ee_data, sizeof(ee_data));
        }
        else if (strncmp(argv[1], "operator", MINMATCH) == 0) {
            strncpy(conf.operator, argv[2], sizeof(conf.operator));
            conf.flags |= SIM7020_CONF_MANUAL_OPERATOR;
        }
        else if (strncmp(argv[1], "restore", MINMATCH) == 0) {
            read_eeprom(&conf, &ee_data, sizeof(conf));
        }
        else if (strncmp(argv[1], "save", MINMATCH) == 0) {
            update_eeprom(&conf, &ee_data, sizeof(conf));
        }
        else
            goto usage;
        return 1;
    }
    
usage:
    printf("Usage:\n");
    char *indent = "  ";
    printf("%s%s apn <apn>\n", indent, argv[0]);
    printf("%s%s operator <operator>\n", indent, argv[0]);
    printf("%s%s restore\n", indent, argv[0]);
    printf("%s%s save\n", indent, argv[0]);
    return -1;  
}



