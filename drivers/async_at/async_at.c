/*
 * Copyright (C) 2022 Peter Sjödin, KTH
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

extern at_dev_t at_dev; /* For now */
extern mutex_t sim7020_lock; /* For now */
#define SIM_LOCK() {mutex_lock(&sim7020_lock);}
#define SIM_UNLOCK() {mutex_unlock(&sim7020_lock);}

static void async_at_cb(void *arg, const char *code) {
    async_at_t *aap = (async_at_t *) arg;
    xtimer_remove(&aap->timeout_timer);
    aap->cb(aap, aap->arg, code);
    thread_wakeup(aap->pid);
}

static void async_at_timeout_cb(void *arg)
{
    async_at_t *aap = (async_at_t *) arg;
    aap->state = R_TIMEOUT;
    thread_wakeup(aap->pid);
}

void async_at_setup(async_at_t *aap, async_cb_t cb, void *arg, const char *code, uint32_t offset) {
    aap->state = R_WAIT;
    aap->cb = cb;
    aap->arg = arg;
    aap->pid = thread_getpid();
    aap->cmd = NULL;
    {
        static int seqno = 0;
        aap->seqno = seqno++;
    }
    aap->urc.cb = async_at_cb;
    aap->urc.arg = aap;
    aap->urc.code = code;
    at_add_urc(&at_dev, &aap->urc);

    aap->timeout_timer.callback = async_at_timeout_cb;
    aap->timeout_timer.arg = aap;
    xtimer_set(&aap->timeout_timer, offset);
}

void async_at_stop(async_at_t *aap) {
    xtimer_remove(&aap->timeout_timer);
    at_remove_urc(&at_dev, &aap->urc);
}

int async_at_wait(async_at_t *aap) {
    while (aap->state == R_WAIT)
        thread_sleep();
    if (aap->state != R_DONE) {
        //printf("AA_WAIT -> %d, pid %d urc %s\n", (int) aap->state, aap->pid, aap->urc.code);
        return -1;
    }
    else 
        return 0;
}

void async_at_null_cb(void *async_at, __attribute__((unused)) void *arg, __attribute__((unused)) const char *code) {
    async_at_t *aap = (async_at_t *) async_at;
    aap->state = R_DONE;
}

int async_at_send_cmd_wait_resp(at_dev_t *dev, const char *command, const char *resp, uint32_t timeout) {
    async_at_t async_at;
    int res;

    async_at_setup(&async_at, async_at_null_cb, NULL, resp, timeout);
    async_at.cmd = command;
    SIM_LOCK();
    at_drain(dev);
    at_send_bytes(dev, command, strlen(command));
    at_send_bytes(dev, CONFIG_AT_SEND_EOL, AT_SEND_EOL_LEN);
    SIM_UNLOCK();
    res = async_at_wait(&async_at);
    async_at_stop(&async_at);
    return res;
}

int async_at_send_cmd_wait_ok(at_dev_t *dev, const char *command, uint32_t timeout) {
    return async_at_send_cmd_wait_resp(dev, command, "OK", timeout);
}
