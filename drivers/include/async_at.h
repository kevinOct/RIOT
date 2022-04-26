/*
 * Copyright (C) 2022 Peter Sjödin, KTH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#ifndef ASYNC_AT_H
#define ASYNC_AT_H

#include "at.h"
#include "xtimer.h"

typedef void (*async_cb_t)(void /* async_at_t */ *aap, void *arg, const char *code);

struct async_at {
    const char *cmd;
    at_urc_t urc;
    enum {
        R_WAIT, R_DONE, R_TIMEOUT, R_ERROR
    } state;
    async_cb_t cb;
    void *arg;
    xtimer_t timeout_timer;
    kernel_pid_t pid;
    uint8_t seqno; /* debugging */
};
typedef struct async_at async_at_t;

void async_at_setup(async_at_t *aap, async_cb_t cb, void *arg, const char *code, uint32_t offset);
void async_at_stop(async_at_t *aap);
int async_at_wait(async_at_t *aap);
int async_at_send_cmd_wait_resp(at_dev_t *dev, const char *command, const char *resp, uint32_t timeout);
int async_at_send_cmd_wait_ok(at_dev_t *dev, const char *command, uint32_t timeout);
void async_at_null_cb(void *async_at, void *arg, const char *code);

#endif /* ASYNC_AT_H */
