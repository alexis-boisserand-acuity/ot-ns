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
 * See ieee802154_isr.h.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/sys/byteorder.h>

#include <openthread-system.h>

#include "ieee802154_data.h"
#include "ieee802154_frame.h"
#include "ieee802154_isr.h"
#include "ieee802154_priv.h"
#include "radio.h"

#define LOG_MODULE_NAME ieee802154_otns
#if defined(CONFIG_IEEE802154_OTNS_LOG_LEVEL)
#define LOG_LEVEL CONFIG_IEEE802154_OTNS_LOG_LEVEL
#else
#define LOG_LEVEL LOG_LEVEL_INF
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(LOG_MODULE_NAME);

/* ------------------------------------------------------------------------- */
/* RX frame delivery to the net stack                                        */
/* ------------------------------------------------------------------------- */

struct net_pkt *rx_pkt_new(const uint8_t *psdu, uint16_t psdu_len, int8_t rssi_dbm, uint64_t timestamp_us)
{
    uint16_t mac_len = IS_ENABLED(CONFIG_IEEE802154_L2_PKT_INCL_FCS) ? psdu_len : psdu_len - FCS_SIZE;

    struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(data.iface, mac_len, NET_AF_UNSPEC, 0, K_NO_WAIT);
    if (pkt == NULL)
    {
        LOG_WRN("No RX pkt available, dropping frame");
        return NULL;
    }

    if (net_pkt_write(pkt, psdu, mac_len) < 0)
    {
        LOG_WRN("Failed to write RX frame");
        net_pkt_unref(pkt);
        return NULL;
    }

    net_pkt_set_ieee802154_lqi(pkt, LQI_PERFECT);
    net_pkt_set_ieee802154_rssi_dbm(pkt, rssi_dbm);
    net_pkt_set_timestamp_ns(pkt, timestamp_us * NSEC_PER_USEC);
    return pkt;
}

static void deliver_rx(const struct otns_radio_event *ev, bool acked_with_fpb, const struct ack_security_info *security)
{
    if (ev->psdu_len < FCS_SIZE)
    {
        return;
    }

    struct net_pkt *pkt = rx_pkt_new(ev->psdu, ev->psdu_len, ev->data.mPower, ev->timestamp_us);
    if (pkt == NULL)
    {
        return;
    }

    net_pkt_set_ieee802154_ack_fpb(pkt, acked_with_fpb);
    net_pkt_set_ieee802154_ack_seb(pkt, security != NULL && security->secured);
    net_pkt_set_ieee802154_ack_keyid(pkt, security != NULL ? security->key_id : 0);
    net_pkt_set_ieee802154_ack_fc(pkt, security != NULL ? security->frame_counter : 0);

    if (net_recv_data(data.iface, pkt) < 0)
    {
        LOG_DBG("RX frame dropped by net stack");
        net_pkt_unref(pkt);
    }
}

static void handle_rx_done(const struct otns_radio_event *ev)
{
    if (ev->psdu_len < MIN_FRAME_SIZE)
    {
        return;
    }

    if (frame_is_ack(ev->psdu, ev->psdu_len))
    {
        uint16_t l = ev->psdu_len;

        if (l > sizeof(data.rx_ack_psdu))
        {
            l = sizeof(data.rx_ack_psdu);
        }
        memcpy(data.rx_ack_psdu, ev->psdu, l);
        data.rx_ack_len          = l;
        data.rx_ack_timestamp_us = ev->timestamp_us;
        return;
    }

    struct frame_addr_info info;

    if (!parse_frame(ev->psdu, ev->psdu_len, &info))
    {
        return;
    }

    if (info.dst_off != FRAME_OFF_NONE && !info.dst_broadcast && (info.fcf & FCF_ACK_REQ_BIT))
    {
        struct ack_security_info security;
        bool                     scheduled = schedule_ack(ev->psdu, ev->psdu_len, ev->data.mPower, &security);
        bool                     fpb       = scheduled && frame_pending_for(ev->psdu, ev->psdu_len);

        deliver_rx(ev, fpb, scheduled ? &security : NULL);
    }
    else
    {
        deliver_rx(ev, false, NULL);
    }
}

/* ------------------------------------------------------------------------- */
/* ISR / event handling                                                      */
/* ------------------------------------------------------------------------- */

static int err_to_errno(uint8_t error)
{
    switch (error)
    {
    case OT_ERROR_NONE:
        return 0;
    case OT_ERROR_CHANNEL_ACCESS_FAILURE:
        return -EBUSY;
    case OT_ERROR_NO_ACK:
        return -ENOMSG;
    case OT_ERROR_ABORT:
        return -EIO;
    default:
        return -EIO;
    }
}

static void handle_tx_done(const struct otns_radio_event *ev)
{
    data.tx_result = err_to_errno(ev->data.mError);
    k_sem_give(&data.tx_wait);
}

static void handle_chan_sample(const struct otns_radio_event *ev)
{
    energy_scan_done_cb_t cb = data.ed_done_cb;

    if (cb != NULL)
    {
        data.ed_done_cb = NULL;
        cb(radio_dev, ev->data.mPower);
        return;
    }
    data.cca_channel_free = ev->data.mError == OT_ERROR_NONE && (ev->data.mPower == (int8_t)OT_RADIO_RSSI_INVALID ||
                                                                 ev->data.mPower < nsi_otns_bottom_get_cca_threshold());
    k_sem_give(&data.cca_wait);
}

void isr(const void *arg)
{
    struct otns_radio_event ev;

    ARG_UNUSED(arg);

    if (nsi_otns_bottom_host_msg_pending())
    {
        otSysEventSignalPending();
    }

    while (nsi_otns_bottom_get_event(&ev))
    {
        switch (ev.type)
        {
        case OT_SIM_EVENT_RADIO_RX_DONE:
            handle_rx_done(&ev);
            break;

        case OT_SIM_EVENT_RADIO_TX_DONE:
            handle_tx_done(&ev);
            break;

        case OT_SIM_EVENT_RADIO_CHAN_SAMPLE:
            handle_chan_sample(&ev);
            break;

        case OTNS_RADIO_PREPARE_ACK:
        {
            int ret = refresh_ack(ev.psdu, ev.psdu_len);

            if (ret == 0)
            {
                ret = nsi_otns_bottom_update_ack(ev.psdu, ev.psdu_len);
            }
            if (ret < 0)
            {
                LOG_ERR("Failed to refresh ACK: %d", ret);
            }
            break;
        }

        default:
            break;
        }
    }
}
