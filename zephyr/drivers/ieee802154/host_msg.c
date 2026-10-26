/*
 * Copyright (c) 2026, The OTNS Authors.
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
 * Dispatches the messages the runner side queues for the OpenThread thread: CLI input (UART events) and,
 * on border routers, IP/UDP datagrams from the host.
 */

#include <zephyr/kernel.h>

#include <openthread/instance.h>

#include "ieee802154_priv.h"

void otns_host_msg_process(otInstance *aInstance)
{
    const struct otns_host_msg *msg;

#if defined(CONFIG_OPENTHREAD_BORDER_ROUTING) && defined(CONFIG_OPENTHREAD_UDP_FORWARD)
    otns_host_forward_init(aInstance);
#endif

    while (nsi_otns_bottom_peek_host_msg(&msg))
    {
        if (msg->type == OT_SIM_EVENT_UART_WRITE)
        {
            otns_cli_input(aInstance, msg->data, msg->len);
        }
#if defined(CONFIG_OPENTHREAD_BORDER_ROUTING) && defined(CONFIG_OPENTHREAD_UDP_FORWARD)
        else
        {
            otns_host_forward_msg(aInstance, msg);
        }
#endif
        nsi_otns_bottom_pop_host_msg();
    }
}
