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
 */

#include <string.h>

#include <zephyr/sys/util.h>
#include <openthread/cli.h>

#include "switch_input.h"
#include "switch_light_protocol.h"

static otError switchCommand(void *aContext, uint8_t aArgsLength, char *aArgs[])
{
    (void)aContext;

    if (aArgsLength != 2 || strcmp(aArgs[0], "press") != 0)
    {
        return OT_ERROR_INVALID_ARGS;
    }

    if (!strcmp(aArgs[1], "on"))
    {
        switch_input_press(LIGHT_CMD_ON);
    }
    else if (!strcmp(aArgs[1], "off"))
    {
        switch_input_press(LIGHT_CMD_OFF);
    }
    else if (!strcmp(aArgs[1], "toggle"))
    {
        switch_input_press(LIGHT_CMD_TOGGLE);
    }
    else
    {
        return OT_ERROR_INVALID_ARGS;
    }

    return OT_ERROR_NONE;
}

static const otCliCommand sSwitchCommands[] = {
    {"switch", switchCommand},
};

void otCliVendorSetUserCommands(void) { otCliSetUserCommands(sSwitchCommands, ARRAY_SIZE(sSwitchCommands), NULL); }
