/*
 * Copyright (c) 2026, The OpenThread Authors.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * OTNS OpenThread CLI bridge: executes the CLI lines OTNS sends as UART events and returns the output as UART events.
 * Runs on the OpenThread thread (see otns_host_msg_process()), so no thread or queue of its own is needed.
 */

#define LOG_MODULE_NAME otns_cli
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <openthread/cli.h>
#include <openthread/instance.h>
#include <openthread/link.h>

#include <nsi_main.h>

#include "ieee802154_priv.h"

#define CLI_LINE_MAX 384
#define OUTPUT_BUF_MAX 512

static char line[CLI_LINE_MAX];
static int  line_len;
static bool cli_initialized;

static int cli_output_cb(void *context, const char *format, va_list arg)
{
    char buf[OUTPUT_BUF_MAX];

    ARG_UNUSED(context);

    int len = vsnprintf(buf, sizeof(buf), format, arg);

    if (len <= 0)
    {
        return 0;
    }
    if (len > (int)sizeof(buf))
    {
        len = sizeof(buf);
    }

    nsi_otns_bottom_send_uart((const uint8_t *)buf, len);
    return len;
}

#ifdef CONFIG_IEEE802154_OTNS_CLI_LEGACY_CSL_PERIOD
static void translate_legacy_csl_period(void)
{
    static const char prefix[] = "csl period ";
    char             *end;
    unsigned long     period;

    if (strncmp(line, prefix, sizeof(prefix) - 1) != 0)
    {
        return;
    }

    period = strtoul(line + sizeof(prefix) - 1, &end, 10);
    if (end == line + sizeof(prefix) - 1 || *end != '\0')
    {
        return;
    }

    snprintf(line, sizeof(line), "%s%lu", prefix, period * OT_US_PER_TEN_SYMBOLS);
}
#endif

static void run_line(void)
{
    line[line_len] = '\0';
    line_len       = 0;

    if (strcmp(line, "exit") == 0)
    {
        nsi_exit(0);
    }

    nsi_otns_bottom_send_uart((const uint8_t *)line, strlen(line));
    nsi_otns_bottom_send_uart((const uint8_t *)"\r\n", 2);

#ifdef CONFIG_IEEE802154_OTNS_CLI_LEGACY_CSL_PERIOD
    translate_legacy_csl_period();
#endif

    otCliInputLine(line);
}

void otns_cli_input(otInstance *aInstance, const uint8_t *buf, uint16_t len)
{
    if (!cli_initialized)
    {
        otCliInit(aInstance, cli_output_cb, NULL);
        cli_initialized = true;
        LOG_INF("OTNS OpenThread CLI bridge ready");
    }

    for (uint16_t i = 0; i < len; i++)
    {
        char c = (char)buf[i];

        if (c == '\n')
        {
            if (line_len > 0)
            {
                run_line();
            }
        }
        else if (c != '\r' && line_len < CLI_LINE_MAX - 1)
        {
            line[line_len++] = c;
        }
    }
}
