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
 */

#include <string.h>

#include <openthread-system.h>
#include <openthread/ip6.h>
#include <openthread/message.h>
#include <openthread/udp.h>

#include "ieee802154_priv.h"

#define LOG_MODULE_NAME ieee802154_otns
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(LOG_MODULE_NAME);

#define OTNS_HOST_MAX_DATAGRAM_LEN 1280
#define OTNS_IP6_HEADER_LEN 40
#define OTNS_IP6_NEXT_HDR_TCP 6
#define OTNS_IP6_NEXT_HDR_UDP 17
#define OTNS_TMF_PORT 61631 /* mesh-local TMF messages are never forwarded to the host */

static bool                 s_registered;
static uint8_t              s_host_buf[sizeof(struct MsgToHostEventData) + OTNS_HOST_MAX_DATAGRAM_LEN];
static otIp6ReceiveCallback s_downstream_cb;
static void                *s_downstream_ctx;
static bool                 s_ip6_cb_installed;

static bool is_link_local(const uint8_t *addr)
{
    return (addr[0] == 0xfe && (addr[1] & 0xc0) == 0x80) || (addr[0] == 0xff && (addr[1] & 0x0f) == 0x02);
}

static uint8_t mcast_scope(const uint8_t *addr) { return addr[0] == 0xff ? (addr[1] & 0x0f) : 0xff; }

static bool parse_ip6(const otMessage *aMessage, struct MsgToHostEventData *aInfo)
{
    uint8_t hdr[OTNS_IP6_HEADER_LEN];
    uint8_t ports[4];

    if (otMessageRead(aMessage, 0, hdr, sizeof(hdr)) != sizeof(hdr))
    {
        return false;
    }

    memcpy(aInfo->mSrcIp6, &hdr[8], OT_IP6_ADDRESS_SIZE);
    memcpy(aInfo->mDstIp6, &hdr[24], OT_IP6_ADDRESS_SIZE);
    aInfo->mSrcPort = 0;
    aInfo->mDstPort = 0;

    if ((hdr[6] == OTNS_IP6_NEXT_HDR_UDP || hdr[6] == OTNS_IP6_NEXT_HDR_TCP) &&
        otMessageRead(aMessage, OTNS_IP6_HEADER_LEN, ports, sizeof(ports)) == sizeof(ports))
    {
        aInfo->mSrcPort = (uint16_t)((ports[0] << 8) | ports[1]);
        aInfo->mDstPort = (uint16_t)((ports[2] << 8) | ports[3]);
    }

    return true;
}

static bool should_forward_to_host(const otMessage *aMessage, const struct MsgToHostEventData *aInfo)
{
    return otMessageIsLoopbackToHostAllowed(aMessage) && aInfo->mSrcPort > 0 && aInfo->mDstPort > 0 &&
           aInfo->mDstPort != OTNS_TMF_PORT && !is_link_local(aInfo->mSrcIp6) && !is_link_local(aInfo->mDstIp6) &&
           mcast_scope(aInfo->mDstIp6) >= 0x4;
}

static void send_to_host(uint8_t type, const struct MsgToHostEventData *info, const otMessage *aMessage)
{
    uint16_t msgLen = otMessageGetLength(aMessage);

    if (msgLen > OTNS_HOST_MAX_DATAGRAM_LEN)
    {
        LOG_WRN("Datagram too large for host (%u bytes)", msgLen);
        return;
    }

    memcpy(s_host_buf, info, sizeof(*info));
    (void)otMessageRead(aMessage, 0, s_host_buf + sizeof(*info), msgLen);
    nsi_otns_bottom_send_msg_to_host(type, s_host_buf, sizeof(*info) + msgLen);
}

static void forward_ip6_to_host(otMessage *aMessage)
{
    struct MsgToHostEventData info;

    if (parse_ip6(aMessage, &info) && should_forward_to_host(aMessage, &info))
    {
        send_to_host(OT_SIM_EVENT_IP6_TO_HOST, &info, aMessage);
    }
}

static void ip6_receive_trampoline(otMessage *aMessage, void *aContext)
{
    (void)aContext;

    forward_ip6_to_host(aMessage);

    if (s_downstream_cb != NULL)
    {
        s_downstream_cb(aMessage, s_downstream_ctx);
    }
    else
    {
        otMessageFree(aMessage);
    }
}

extern void __real_openthread_set_receive_cb(otIp6ReceiveCallback aCallback, void *aContext);

void __wrap_openthread_set_receive_cb(otIp6ReceiveCallback aCallback, void *aContext)
{
    s_downstream_cb    = aCallback;
    s_downstream_ctx   = aContext;
    s_ip6_cb_installed = true;
    __real_openthread_set_receive_cb(ip6_receive_trampoline, NULL);
}

static void handle_udp_to_host(otMessage    *aMessage,
                               uint16_t      aPeerPort,
                               otIp6Address *aPeerAddr,
                               uint16_t      aSockPort,
                               void         *aContext)
{
    struct MsgToHostEventData info;

    info.mSrcPort = aSockPort;
    info.mDstPort = aPeerPort;
    memset(info.mSrcIp6, 0, sizeof(info.mSrcIp6));
    memcpy(info.mDstIp6, aPeerAddr, OT_IP6_ADDRESS_SIZE);

    send_to_host(OT_SIM_EVENT_UDP_TO_HOST, &info, aMessage);
    otMessageFree(aMessage);
}

static void inject_ip6_from_host(otInstance                      *aInstance,
                                 const struct MsgToHostEventData *aHdr,
                                 const uint8_t                   *aPayload,
                                 uint16_t                         aPayloadLen)
{
    const otIp6Address *srcIp6 = (const otIp6Address *)aHdr->mSrcIp6;
    const otIp6Address *dstIp6 = (const otIp6Address *)aHdr->mDstIp6;
    otMessage          *msg    = otIp6NewMessageFromBuffer(aInstance, aPayload, aPayloadLen, NULL);
    otError             error  = OT_ERROR_NONE;

    if (msg == NULL)
    {
        LOG_WRN("Dropping IP6_FROM_HOST datagram: no buffers");
        return;
    }

    if (otIp6IsAddressUnspecified(dstIp6))
    {
        otUdpForwardReceive(aInstance, msg, aHdr->mSrcPort, srcIp6, aHdr->mDstPort);
    }
    else
    {
        error = otIp6Send(aInstance, msg);
        if (error != OT_ERROR_NONE)
        {
            LOG_WRN("Dropping IP6_FROM_HOST datagram: %d", error);
        }
    }
}

static void inject_udp_from_host(otInstance                      *aInstance,
                                 const struct MsgToHostEventData *aHdr,
                                 const uint8_t                   *aPayload,
                                 uint16_t                         aPayloadLen)
{
    otMessage *udp = otUdpNewMessage(aInstance, NULL);

    if (udp == NULL)
    {
        LOG_WRN("Dropping UDP_FROM_HOST datagram: no buffers");
        return;
    }

    if (otMessageAppend(udp, aPayload, aPayloadLen) != OT_ERROR_NONE)
    {
        otMessageFree(udp);
        LOG_WRN("Dropping UDP_FROM_HOST datagram: append failed");
        return;
    }

    otUdpForwardReceive(aInstance, udp, aHdr->mSrcPort, (const otIp6Address *)aHdr->mSrcIp6, aHdr->mDstPort);
}

void otns_host_forward_process(otInstance *aInstance)
{
    const struct otns_host_msg *msg;

    if (!s_registered)
    {
        s_registered = true;
        otIp6SetReceiveFilterEnabled(aInstance, true);
        if (!s_ip6_cb_installed)
        {
            s_ip6_cb_installed = true;
            otIp6SetReceiveCallback(aInstance, ip6_receive_trampoline, NULL);
        }
        otUdpForwardSetForwarder(aInstance, handle_udp_to_host, aInstance);
    }

    while (nsi_otns_bottom_peek_host_msg(&msg))
    {
        if (msg->len >= sizeof(struct MsgToHostEventData))
        {
            const struct MsgToHostEventData *hdr        = (const struct MsgToHostEventData *)msg->data;
            const uint8_t                   *payload    = msg->data + sizeof(*hdr);
            uint16_t                         payloadLen = msg->len - sizeof(*hdr);

            if (msg->type == OT_SIM_EVENT_IP6_FROM_HOST)
            {
                inject_ip6_from_host(aInstance, hdr, payload, payloadLen);
            }
            else if (msg->type == OT_SIM_EVENT_UDP_FROM_HOST)
            {
                inject_udp_from_host(aInstance, hdr, payload, payloadLen);
            }
        }

        nsi_otns_bottom_pop_host_msg();
    }
}
