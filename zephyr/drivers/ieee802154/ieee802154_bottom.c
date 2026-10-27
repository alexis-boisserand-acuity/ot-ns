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
 * Runner ("bottom") side of the OTNS virtual IEEE 802.15.4 radio driver.
 *
 * This file is compiled as part of the native simulator runner (host code). It
 * owns the real Unix domain socket connection to OTNS, implements the OTNS
 * simulation-event wire protocol, and — crucially — synchronizes the native
 * simulator virtual clock with the OTNS virtual time using the native simulator
 * hardware-event scheduler (a "pacer" NSI_HW_EVENT).
 */

/* The runner is built in strict mode, which hides the <endian.h> conversion macros. */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#include <endian.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/un.h>

#include "irq_ctrl.h"
#include "nsi_cmdline.h"
#include "nsi_hw_scheduler.h"
#include "nsi_hws_models_if.h"
#include "nsi_tasks.h"
#include "nsi_tracing.h"

#include "event_sim_ref.h"
#include "ieee802154_frame.h"
#include "ieee802154_priv.h"

#define PHY_BITRATE_DEFAULT 250000U
#define RADIO_MSG_HDR sizeof(((struct RadioMessage *)0)->mChannel)
#define RADIO_COMM_EVENT_DATA_SIZE sizeof(struct RadioCommEventData)
#define MAX_SLEEP_US 3600000000ULL
#define DRIFT_ACCUM_PER_US 1000000
#define US_PER_MS 1000
#define US_PER_SEC 1000000U
#define BITS_PER_BYTE 8
#define BYTE_MASK 0xff
#define RADIO_MIN_FRAME_SIZE 3
#define TX_INTERFERER_LEVEL_MAX 100
#define INTERFERER_DELAY_RANGE_US 1000000
#define INTERFERER_LEVEL_STEP_US 10000
#define PACER_PRIO 950
#define RADIO_TIMER_PRIO 940
#define REGISTER_CMDLINE_ORDER 200
#define BOOT_ORDER 500
#define CLEANUP_ORDER 100
#define RFSIM_PARAM_RSP_DATA_SIZE (sizeof(uint8_t) + sizeof(int32_t))

/* ------------------------------------------------------------------------- */
/* State                                                                     */
/* ------------------------------------------------------------------------- */

static uint32_t    cmd_node_id;
static const char *cmd_socket;
static int32_t     cmd_seed;

static bool connected;
static bool disabled;

static uint64_t pacer_time;

static struct Event pending_ev;
static bool         have_pending;

static struct otns_radio_event deliver_ev;
static bool                    deliver_ready;

#define HOST_MSG_QUEUE_LEN 8
static struct otns_host_msg host_msgs[HOST_MSG_QUEUE_LEN];
static unsigned             host_msg_head;
static unsigned             host_msg_count;
static uint8_t              ack_psdu[OT_RADIO_FRAME_MAX_SIZE];
static uint16_t             ack_psdu_len;

struct radio_report
{
    uint8_t       state;
    uint8_t       channel;
    RadioSubState sub_state;
    uint64_t      event_time;
    int8_t        power;
    int8_t        sensitivity;
};

static uint8_t       radio_state      = OT_RADIO_STATE_DISABLED;
static uint8_t       radio_channel    = OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MIN;
static uint8_t       current_channel  = OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MIN;
static RadioSubState radio_sub_state  = RFSIM_RADIO_SUBSTATE_READY;
static uint64_t      radio_event_time = NSI_NEVER;
static int8_t        radio_power;

static struct radio_report reported = {.state      = OT_RADIO_STATE_INVALID,
                                       .channel    = OT_RADIO_STATE_INVALID,
                                       .sub_state  = RFSIM_RADIO_SUBSTATE_INVALID,
                                       .event_time = NSI_NEVER};

static bool                    tx_pending;
static bool                    tx_wait;
static bool                    tx_cca;
static bool                    tx_wants_ack;
static uint8_t                 tx_psdu[OT_RADIO_FRAME_MAX_SIZE];
static uint16_t                tx_len;
static uint64_t                last_tx_ifs = OT_RADIO_SIFS_TIME_US;
static bool                    sample_pending;
static bool                    energy_scan;
static uint64_t                sample_duration;
static struct otns_radio_event sample_result;
static bool                    sleep_pending;
static uint64_t                rx_timestamp_us;
static uint8_t                 tx_interferer_level;
static bool                    interferer_cca_failed;

static void report_state(void);
static void radio_timer_fired(void);
static void start_pending_operation(void);
static void wake_pacer_now(void);
static void complete_tx(uint8_t error);
static void cancel_sample(void);
static void interferer_timer_fired(void);
static void interferer_deliver(const struct otns_radio_event *ev);

static uint8_t  own_short_addr[SHORT_ADDR_SIZE];
static uint8_t  own_ext_addr[OT_EXT_ADDRESS_SIZE];
static uint16_t own_pan_id;
static bool     ext_addr_announced;

static int8_t   s_rx_sensitivity  = RFSIM_RX_SENSITIVITY_DEFAULT_DBM;
static int8_t   s_cca_threshold   = RFSIM_CCA_ED_THRESHOLD_DEFAULT_DBM;
static uint64_t s_phy_bitrate     = PHY_BITRATE_DEFAULT;
static int16_t  s_clock_drift_ppm = 0;
static uint8_t  s_csl_accuracy    = RFSIM_CSL_ACCURACY_DEFAULT_PPM;
static uint8_t  s_csl_uncertainty = RFSIM_CSL_UNCERTAINTY_DEFAULT_10US;

static int64_t  s_drift_ps        = 0;
static int64_t  s_drift_us_total  = 0;
static uint64_t s_drift_last_time = 0;

/* ------------------------------------------------------------------------- */
/* Little-endian (de)serialization helpers                                   */
/* ------------------------------------------------------------------------- */

static inline int32_t get_le32(const uint8_t *p)
{
    uint32_t value;

    memcpy(&value, p, sizeof(value));
    return (int32_t)le32toh(value);
}

static inline uint64_t get_le64(const uint8_t *p)
{
    uint64_t value;

    memcpy(&value, p, sizeof(value));
    return le64toh(value);
}

static void set_sub_state(RadioSubState state, uint64_t duration)
{
    radio_sub_state  = state;
    radio_event_time = duration == NSI_NEVER ? NSI_NEVER : nsi_hws_get_time() + duration;
    nsi_hws_find_next_event();
    wake_pacer_now();
}

static void update_alarm_drift_offset(uint64_t now)
{
    uint64_t elapsed = now - s_drift_last_time;

    s_drift_last_time = now;

    if (s_clock_drift_ppm == 0 || elapsed == 0)
    {
        return;
    }

    s_drift_ps += (int64_t)s_clock_drift_ppm * (int64_t)elapsed;
    if (s_drift_ps >= DRIFT_ACCUM_PER_US || s_drift_ps <= -DRIFT_ACCUM_PER_US)
    {
        int64_t whole_us = s_drift_ps / DRIFT_ACCUM_PER_US;

        s_drift_ps -= whole_us * DRIFT_ACCUM_PER_US;
        s_drift_us_total += whole_us;
    }
}

uint64_t nsi_otns_bottom_get_time_us(void)
{
    uint64_t now = nsi_hws_get_time();

    update_alarm_drift_offset(now);
    return (uint64_t)((int64_t)now + s_drift_us_total);
}

uint8_t nsi_otns_bottom_get_csl_accuracy(void) { return s_csl_accuracy; }

/* ------------------------------------------------------------------------- */
/* Socket I/O                                                                */
/* ------------------------------------------------------------------------- */

static int read_full(void *buf, size_t n)
{
    size_t got = 0;

    while (got < n)
    {
        ssize_t r = read(gSockFd, (char *)buf + got, n - got);

        if (r > 0)
        {
            got += (size_t)r;
            continue;
        }
        if (r == 0)
        {
            fprintf(stderr, "[otns] socket closed by OTNS (EOF) after %zu/%zu bytes\n", got, n);
            return -1;
        }
        if (errno == EINTR)
        {
            continue;
        }
        fprintf(stderr, "[otns] socket read error: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int read_event(struct Event *ev)
{
    if (read_full(ev, sizeof(struct EventHeader)) < 0)
    {
        return -1;
    }

    if (ev->mDataLength > 0)
    {
        if (ev->mDataLength > sizeof(ev->mData))
        {
            fprintf(stderr, "[otns] event datalen %u too large (ev=%u)\n", ev->mDataLength, ev->mEvent);
            return -1;
        }
        if (read_full(ev->mData, ev->mDataLength) < 0)
        {
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Connection / handshake                                                    */
/* ------------------------------------------------------------------------- */

static void disconnect(void)
{
    complete_tx(OT_ERROR_ABORT);
    cancel_sample();
    if (gSockFd > 0)
    {
        close(gSockFd);
        gSockFd = 0;
    }
    connected        = false;
    disabled         = true;
    pacer_time       = NSI_NEVER;
    radio_event_time = NSI_NEVER;
    tx_pending       = false;
    sample_pending   = false;
    nsi_print_warning("ieee802154_otns: disconnected from OTNS\n");
}

static void announce_ext_addr(void)
{
    otExtAddress ext_addr_be;

    ext_addr_reverse(ext_addr_be.m8, own_ext_addr);
    otSimSendExtAddrEvent(&ext_addr_be);
}

static int try_connect(void)
{
    if (cmd_socket == NULL || cmd_socket[0] == '\0')
    {
        return -1;
    }

    size_t             path_len = strlen(cmd_socket);
    struct sockaddr_un addr;
    if (path_len >= sizeof(addr.sun_path))
    {
        nsi_print_warning("ieee802154_otns: socket path too long\n");
        return -1;
    }

    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0)
    {
        nsi_print_warning("ieee802154_otns: socket() failed\n");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, cmd_socket, path_len);

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        nsi_print_warning("ieee802154_otns: cannot connect to OTNS at %s\n", cmd_socket);
        close(sock);
        return -1;
    }

    gSockFd   = sock;
    connected = true;

    otSimSendNodeInfoEvent(cmd_node_id);

    if (ext_addr_announced)
    {
        announce_ext_addr();
    }

    nsi_print_trace("ieee802154_otns: connected to OTNS (node %u) at %s\n", cmd_node_id, cmd_socket);
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Pacer (virtual-time lock-step)                                            */
/* ------------------------------------------------------------------------- */

static uint64_t next_other_event(void)
{
    uint64_t saved = pacer_time;

    pacer_time = NSI_NEVER;
    nsi_hws_find_next_event();
    uint64_t next = nsi_hws_get_next_event_time();
    pacer_time    = saved;

    return next;
}

static void raise_embedded(const struct otns_radio_event *ev)
{
    deliver_ev    = *ev;
    deliver_ready = true;

    hw_irq_ctrl_raise_im(IEEE802154_OTNS_IRQ);
}

static void queue_host_msg(const struct Event *raw)
{
    if (host_msg_count == HOST_MSG_QUEUE_LEN)
    {
        fprintf(stderr, "[otns] host message queue full, dropping ev=%u\n", raw->mEvent);
        return;
    }

    struct otns_host_msg *slot = &host_msgs[(host_msg_head + host_msg_count) % HOST_MSG_QUEUE_LEN];

    slot->type = raw->mEvent;
    slot->len  = raw->mDataLength;
    memcpy(slot->data, raw->mData, raw->mDataLength);
    host_msg_count++;

    hw_irq_ctrl_raise_im(IEEE802154_OTNS_IRQ);
}

static void complete_tx(uint8_t error)
{
    struct otns_radio_event result = {0};

    if (!tx_pending)
    {
        return;
    }
    tx_pending           = false;
    tx_wait              = false;
    radio_state          = OT_RADIO_STATE_RECEIVE;
    result.type          = OT_SIM_EVENT_RADIO_TX_DONE;
    result.data.mChannel = radio_channel;
    result.data.mError   = error;
    raise_embedded(&result);
}

static void cancel_sample(void)
{
    struct otns_radio_event result = {0};

    if (!sample_pending)
    {
        return;
    }
    sample_pending       = false;
    result.type          = OT_SIM_EVENT_RADIO_CHAN_SAMPLE;
    result.data.mChannel = radio_channel;
    result.data.mPower   = (int8_t)OT_RADIO_RSSI_INVALID;
    result.data.mError   = OT_ERROR_ABORT;
    raise_embedded(&result);
}

static uint64_t tx_ifs_time(void) { return last_tx_ifs; }

static void handle_comm_start_event(const struct otns_radio_event *ev)
{
    if ((radio_state == OT_RADIO_STATE_RECEIVE || radio_state == OT_RADIO_STATE_TRANSMIT) &&
        ev->data.mError == OT_ERROR_NONE &&
        (radio_sub_state == RFSIM_RADIO_SUBSTATE_READY || radio_sub_state == RFSIM_RADIO_SUBSTATE_IFS_WAIT ||
         radio_sub_state == RFSIM_RADIO_SUBSTATE_TX_AIFS_WAIT))
    {
        set_sub_state(radio_sub_state == RFSIM_RADIO_SUBSTATE_TX_AIFS_WAIT ? RFSIM_RADIO_SUBSTATE_TX_ACK_RX_ONGOING
                                                                           : RFSIM_RADIO_SUBSTATE_RX_FRAME_ONGOING,
                      ev->data.mDuration + FAILSAFE_TIME_US);
        rx_timestamp_us = nsi_otns_bottom_get_time_us() + OT_RADIO_SHR_DURATION_US;
    }
}

static void radio_receive(const struct otns_radio_event *ev, const struct otns_radio_event *received, bool accepted)
{
    bool is_ack = frame_is_ack(ev->psdu, ev->psdu_len);

    if (tx_wait && tx_wants_ack)
    {
        bool is_awaited_ack_received = accepted && is_ack && ev->data.mError == OT_ERROR_NONE &&
                                       ev->psdu_len >= ACK_FRAME_SIZE &&
                                       ev->psdu[SEQ_NUM_OFFSET] == tx_psdu[SEQ_NUM_OFFSET];

        if (is_awaited_ack_received)
        {
            raise_embedded(received);
        }
        complete_tx(is_awaited_ack_received ? OT_ERROR_NONE : OT_ERROR_NO_ACK);
    }
    else if (accepted && !is_ack && ev->data.mError == OT_ERROR_NONE)
    {
        raise_embedded(received);
    }
}

static void apply_radio_delayed_sleep(void)
{
    if (sleep_pending)
    {
        sleep_pending = false;
        radio_state   = OT_RADIO_STATE_SLEEP;
    }
}

static void handle_rx_done_event(const struct otns_radio_event *ev)
{
    struct otns_radio_event received = *ev;
    struct frame_addr_info  info;

    if (radio_sub_state != RFSIM_RADIO_SUBSTATE_RX_FRAME_ONGOING &&
        radio_sub_state != RFSIM_RADIO_SUBSTATE_TX_ACK_RX_ONGOING)
    {
        return;
    }

    bool is_ack           = frame_is_ack(ev->psdu, ev->psdu_len);
    bool is_ack_requested = frame_is_ack_requested(ev->psdu, ev->psdu_len);
    bool is_addr_match    = frame_addr_matches(ev->psdu, ev->psdu_len, &info, own_pan_id, own_short_addr, own_ext_addr);

    received.timestamp_us = rx_timestamp_us;

    if (radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_FRAME_ONGOING && ev->data.mError == OT_ERROR_NONE &&
        is_ack_requested && !is_ack && is_addr_match && info.dst_off != FRAME_OFF_NONE && !info.dst_broadcast)
    {
        ack_psdu_len = 0;
        set_sub_state(RFSIM_RADIO_SUBSTATE_RX_AIFS_WAIT, OT_RADIO_AIFS_TIME_US);
    }
    else if (radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_FRAME_ONGOING)
    {
        set_sub_state(RFSIM_RADIO_SUBSTATE_IFS_WAIT, RFSIM_TURNAROUND_TIME_US);
        apply_radio_delayed_sleep();
    }
    else
    {
        set_sub_state(RFSIM_RADIO_SUBSTATE_IFS_WAIT, tx_ifs_time());
    }

    radio_receive(ev, &received, is_addr_match);
}

static void handle_tx_done_event(const struct otns_radio_event *ev)
{
    if (radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_ACK_TX_ONGOING)
    {
        set_sub_state(RFSIM_RADIO_SUBSTATE_RX_TX_TO_RX, RFSIM_TURNAROUND_TIME_US);
        apply_radio_delayed_sleep();
    }
    else if (radio_sub_state == RFSIM_RADIO_SUBSTATE_TX_FRAME_ONGOING)
    {
        if (!tx_wants_ack || ev->data.mError != OT_ERROR_NONE)
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_TX_TX_TO_RX, RFSIM_TURNAROUND_TIME_US);
            complete_tx(ev->data.mError);
        }
        else
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_TX_TX_TO_AIFS, RFSIM_TURNAROUND_TIME_US);
        }
    }
}

static void handle_chan_sample_event(const struct otns_radio_event *ev)
{
    if (radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_ENERGY_SCAN)
    {
        sample_pending = false;
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
        radio_channel = current_channel;
        raise_embedded(ev);
    }
    else if (radio_sub_state == RFSIM_RADIO_SUBSTATE_TX_CCA)
    {
        bool clear = ev->data.mError == OT_ERROR_NONE &&
                     (ev->data.mPower == (int8_t)OT_RADIO_RSSI_INVALID || ev->data.mPower < s_cca_threshold);

        if (clear)
        {
            sample_result = *ev;
            set_sub_state(RFSIM_RADIO_SUBSTATE_TX_CCA_TO_TX, RFSIM_TURNAROUND_TIME_US);
        }
        else
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
            radio_channel = current_channel;
            if (tx_pending)
            {
                complete_tx(OT_ERROR_CHANNEL_ACCESS_FAILURE);
            }
            else
            {
                sample_pending = false;
                raise_embedded(ev);
            }
        }
    }
}

static void deliver_to_embedded(const struct otns_radio_event *ev)
{
    if (radio_event_time <= nsi_hws_get_time() && radio_sub_state == RFSIM_RADIO_SUBSTATE_TX_TX_TO_AIFS)
    {
        radio_timer_fired();
    }

    if (tx_interferer_level > 0)
    {
        interferer_deliver(ev);
        return;
    }

    if (ev->data.mChannel != radio_channel)
    {
        return;
    }

    switch (ev->type)
    {
    case OT_SIM_EVENT_RADIO_COMM_START:
        handle_comm_start_event(ev);
        return;

    case OT_SIM_EVENT_RADIO_RX_DONE:
        handle_rx_done_event(ev);
        return;

    case OT_SIM_EVENT_RADIO_TX_DONE:
        handle_tx_done_event(ev);
        return;

    case OT_SIM_EVENT_RADIO_CHAN_SAMPLE:
        handle_chan_sample_event(ev);
        return;

    default:
        return;
    }
}

static bool parse_radio_event(struct otns_radio_event *out, const struct Event *raw)
{
    size_t min_len = RADIO_COMM_EVENT_DATA_SIZE;

    if (raw->mEvent == OT_SIM_EVENT_RADIO_RX_DONE)
    {
        min_len += RADIO_MSG_HDR + FCF_SIZE + FCS_SIZE;
    }
    if (raw->mDataLength < min_len || raw->mDataLength > RADIO_COMM_EVENT_DATA_SIZE + RADIO_MSG_HDR + sizeof(out->psdu))
    {
        nsi_print_warning("ieee802154_otns: invalid radio event length %u (ev=%u)\n", raw->mDataLength, raw->mEvent);
        return false;
    }

    out->type           = raw->mEvent;
    out->data.mChannel  = raw->mData[0];
    out->data.mPower    = (int8_t)raw->mData[1];
    out->data.mError    = raw->mData[2];
    out->data.mDuration = get_le64(&raw->mData[3]);
    out->timestamp_us   = 0;
    out->psdu_len       = 0;

    if (raw->mEvent == OT_SIM_EVENT_RADIO_RX_DONE || raw->mEvent == OT_SIM_EVENT_RADIO_COMM_START)
    {
        if (raw->mDataLength > RADIO_COMM_EVENT_DATA_SIZE + RADIO_MSG_HDR)
        {
            uint16_t len = raw->mDataLength - RADIO_COMM_EVENT_DATA_SIZE - RADIO_MSG_HDR;

            memcpy(out->psdu, &raw->mData[RADIO_COMM_EVENT_DATA_SIZE + RADIO_MSG_HDR], len);
            out->psdu_len = len;
        }
    }
    return true;
}

static int event_is_for_embedded(uint8_t type)
{
    return type == OT_SIM_EVENT_RADIO_COMM_START || type == OT_SIM_EVENT_RADIO_RX_DONE ||
           type == OT_SIM_EVENT_RADIO_TX_DONE || type == OT_SIM_EVENT_RADIO_CHAN_SAMPLE;
}

static void set_tx_interferer_level(int32_t value)
{
    uint8_t level = (value < 0) ? 0 : ((value > TX_INTERFERER_LEVEL_MAX) ? TX_INTERFERER_LEVEL_MAX : (uint8_t)value);

    if (level == tx_interferer_level)
    {
        return;
    }

    bool was_active     = tx_interferer_level > 0;
    tx_interferer_level = level;

    if (level > 0 && !was_active)
    {
        complete_tx(OT_ERROR_ABORT);
        cancel_sample();
        interferer_cca_failed = false;
        radio_channel         = current_channel;
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, 1);
    }
    else if (level == 0)
    {
        radio_channel = current_channel;
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
    }
}

static void handle_rfsim_param_event(uint8_t event_type, const uint8_t *data, uint16_t datalen)
{
    if (datalen < RFSIM_PARAM_RSP_DATA_SIZE)
    {
        return;
    }

    uint8_t param = data[0];
    int32_t value = get_le32(&data[1]);

    if (event_type == OT_SIM_EVENT_RFSIM_PARAM_SET)
    {
        switch (param)
        {
        case RFSIM_PARAM_RX_SENSITIVITY:
            s_rx_sensitivity = (int8_t)value;
            break;
        case RFSIM_PARAM_CCA_THRESHOLD:
            nsi_otns_bottom_set_cca_threshold((int8_t)value);
            break;
        case RFSIM_PARAM_PHY_BITRATE:
            s_phy_bitrate = (value < 1) ? 1 : (uint64_t)value;
            break;
        case RFSIM_PARAM_CLOCK_DRIFT:
            update_alarm_drift_offset(nsi_hws_get_time());
            s_clock_drift_ppm = (int16_t)value;
            s_drift_last_time = nsi_hws_get_time();
            break;
        case RFSIM_PARAM_CSL_ACCURACY:
            s_csl_accuracy = (uint8_t)value;
            break;
        case RFSIM_PARAM_CSL_UNCERTAINTY:
            s_csl_uncertainty = (uint8_t)value;
            break;
        case RFSIM_PARAM_TX_INTERFERER:
            set_tx_interferer_level(value);
            break;
        default:
            break;
        }
    }

    switch (param)
    {
    case RFSIM_PARAM_RX_SENSITIVITY:
        value = (int32_t)s_rx_sensitivity;
        break;
    case RFSIM_PARAM_CCA_THRESHOLD:
        value = (int32_t)nsi_otns_bottom_get_cca_threshold();
        break;
    case RFSIM_PARAM_PHY_BITRATE:
        value = (int32_t)s_phy_bitrate;
        break;
    case RFSIM_PARAM_CLOCK_DRIFT:
        value = (int32_t)s_clock_drift_ppm;
        break;
    case RFSIM_PARAM_CSL_ACCURACY:
        value = (int32_t)s_csl_accuracy;
        break;
    case RFSIM_PARAM_CSL_UNCERTAINTY:
        value = (int32_t)s_csl_uncertainty;
        break;
    case RFSIM_PARAM_TX_INTERFERER:
        value = (int32_t)tx_interferer_level;
        break;
    default:
        param = RFSIM_PARAM_UNKNOWN;
        value = 0;
        break;
    }

    otSimSendRfSimParamRespEvent(param, value);
}

static void deliver_event(const struct Event *raw)
{
    if (event_is_for_embedded(raw->mEvent))
    {
        struct otns_radio_event ev;

        if (parse_radio_event(&ev, raw))
        {
            deliver_to_embedded(&ev);
        }
    }
    else if (raw->mEvent == OT_SIM_EVENT_UART_WRITE)
    {
        queue_host_msg(raw);
    }
    else if (raw->mEvent == OT_SIM_EVENT_RFSIM_PARAM_GET || raw->mEvent == OT_SIM_EVENT_RFSIM_PARAM_SET)
    {
        handle_rfsim_param_event(raw->mEvent, raw->mData, raw->mDataLength);
    }
    else if (raw->mEvent == OT_SIM_EVENT_UDP_FROM_HOST || raw->mEvent == OT_SIM_EVENT_IP6_FROM_HOST)
    {
        queue_host_msg(raw);
    }
}

static void report_state(void)
{
    struct RadioStateEventData data;
    struct radio_report        current;

    memset(&current, 0, sizeof(current));
    current.state       = radio_state;
    current.channel     = radio_channel;
    current.sub_state   = radio_sub_state;
    current.event_time  = radio_event_time;
    current.power       = radio_power;
    current.sensitivity = s_rx_sensitivity;

    if (memcmp(&current, &reported, sizeof(current)) == 0)
    {
        return;
    }

    uint8_t  energy_state = radio_state;
    uint64_t now          = nsi_hws_get_time();
    uint64_t delay        = (radio_event_time != NSI_NEVER && radio_event_time > now) ? radio_event_time - now : 0;

    if (radio_sub_state == RFSIM_RADIO_SUBSTATE_TX_FRAME_ONGOING ||
        radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_ACK_TX_ONGOING)
    {
        energy_state = OT_RADIO_STATE_TRANSMIT;
    }
    else if (radio_state == OT_RADIO_STATE_TRANSMIT || radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_FRAME_ONGOING)
    {
        energy_state = OT_RADIO_STATE_RECEIVE;
    }

    data.mChannel       = radio_channel;
    data.mTxPower       = radio_power;
    data.mRxSensitivity = s_rx_sensitivity;
    data.mEnergyState   = energy_state;
    data.mSubState      = (uint8_t)radio_sub_state;
    data.mState         = radio_state;
    data.mRadioTime     = nsi_otns_bottom_get_time_us();

    otSimSendRadioStateEvent(&data, delay);

    reported = current;
}

static void pacer(void)
{
    if (disabled)
    {
        pacer_time = NSI_NEVER;
        return;
    }

    if (!connected)
    {
        if (try_connect() < 0)
        {
            disabled   = true;
            pacer_time = NSI_NEVER;
            return;
        }
    }

    uint64_t now = nsi_hws_get_time();

    update_alarm_drift_offset(now);

    if (have_pending)
    {
        have_pending = false;
        deliver_event(&pending_ev);
    }

    for (;;)
    {
        uint64_t next = next_other_event();
#ifdef OTNS_TRACE
        fprintf(stderr, "[otns] pacer now=%llu next=%llu\n", (unsigned long long)now, (unsigned long long)next);
#endif

        if (next != NSI_NEVER && next <= now)
        {
            pacer_time = now;
            return;
        }

        uint64_t delay = (next == NSI_NEVER) ? MAX_SLEEP_US : (next - now);

        report_state();

        otns_event_sim_set_sleep_delay(delay);
        otSimSendSleepEvent();

        struct Event raw;

        if (read_event(&raw) < 0)
        {
            fprintf(stderr, "[otns] read event failed\n");
            disconnect();
            return;
        }
        gLastRecvEvent.mMsgId = raw.mMsgId;
#ifdef OTNS_TRACE
        fprintf(stderr, "[otns] recv ev=%u delay=%llu\n", raw.mEvent, (unsigned long long)raw.mDelay);
#endif

        if (raw.mDelay != 0)
        {
            pending_ev   = raw;
            have_pending = true;
            pacer_time   = now + raw.mDelay;
            return;
        }

        deliver_event(&raw);
        if (!event_is_for_embedded(raw.mEvent))
        {
            pacer_time = now;
            return;
        }
    }
}

NSI_HW_EVENT(pacer_time, pacer, PACER_PRIO);

/* ------------------------------------------------------------------------- */
/* Command line options                                                      */
/* ------------------------------------------------------------------------- */

static void register_cmdline_opts(void)
{
    static struct args_struct_t cmdline_options[] = {
        {
            .option   = "otns-node-id",
            .name     = "id",
            .type     = 'u',
            .dest     = (void *)&cmd_node_id,
            .descript = "OTNS node id assigned to this node (>= 1)",
        },
        {
            .option   = "otns-socket",
            .name     = "path",
            .type     = 's',
            .dest     = (void *)&cmd_socket,
            .descript = "Path of the OTNS dispatcher Unix domain socket",
        },
        {
            .option   = "otns-seed",
            .name     = "seed",
            .type     = 'i',
            .dest     = (void *)&cmd_seed,
            .descript = "Optional OTNS random seed",
        },
        ARG_TABLE_ENDMARKER,
    };

    nsi_add_command_line_opts(cmdline_options);
}

NSI_TASK(register_cmdline_opts, PRE_BOOT_1, REGISTER_CMDLINE_ORDER);

static void boot(void)
{
    if (cmd_socket != NULL && cmd_socket[0] != '\0')
    {
        pacer_time = 0;

        /* The CLI output goes to OTNS as UART events, so keep stdout free of anything else. */
        if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0)
        {
            nsi_print_warning("ieee802154_otns: dup2 failed\n");
        }
    }
    else
    {
        disabled   = true;
        pacer_time = NSI_NEVER;
    }
}

NSI_TASK(boot, HW_INIT, BOOT_ORDER);

static void cleanup(void)
{
    if (gSockFd > 0)
    {
        close(gSockFd);
        gSockFd = 0;
    }
}

NSI_TASK(cleanup, ON_EXIT_PRE, CLEANUP_ORDER);

/* ------------------------------------------------------------------------- */
/* Boundary functions called from the embedded side                         */
/* ------------------------------------------------------------------------- */

int nsi_otns_bottom_get_node_id(void) { return (int)cmd_node_id; }

bool nsi_otns_bottom_is_connected(void) { return connected; }

static void wake_pacer_now(void)
{
    if (disabled || !connected)
    {
        return;
    }
    if (have_pending)
    {
        return;
    }
    pacer_time = nsi_hws_get_time();
    nsi_hws_find_next_event();
}

static int send_frame(const uint8_t *psdu, uint16_t len, bool ack)
{
    if (!connected)
    {
        return -1;
    }
    if (len > OT_RADIO_FRAME_MAX_SIZE)
    {
        return -1;
    }

    uint64_t duration = (uint64_t)(OT_RADIO_SHR_PHR_LENGTH_BYTES + len) * BITS_PER_BYTE * US_PER_SEC / s_phy_bitrate;

    struct RadioCommEventData comm = {
        .mChannel = radio_channel, .mPower = radio_power, .mError = OT_ERROR_NONE, .mDuration = duration};
    struct RadioMessage msg;

    msg.mChannel = radio_channel;
    memcpy(msg.mPsdu, psdu, len);

    if (!ack)
    {
        last_tx_ifs = len > OT_RADIO_aMaxSifsFrameSize ? OT_RADIO_LIFS_TIME_US : OT_RADIO_SIFS_TIME_US;
    }
    set_sub_state(ack ? RFSIM_RADIO_SUBSTATE_RX_ACK_TX_ONGOING : RFSIM_RADIO_SUBSTATE_TX_FRAME_ONGOING,
                  duration + FAILSAFE_TIME_US);
    report_state();

    otSimSendRadioCommEvent(&comm, (const uint8_t *)&msg, RADIO_MSG_HDR + len);

    wake_pacer_now();
    return 0;
}

static int send_sample(uint64_t duration)
{
    struct RadioCommEventData sample = {
        .mChannel = radio_channel, .mPower = 0, .mError = OT_ERROR_NONE, .mDuration = duration};

    report_state();
    otSimSendRadioChanSampleEvent(&sample);
    return 0;
}

static uint64_t interferer_rand_range(uint64_t lo, uint64_t hi) { return lo + (uint64_t)rand() % (hi - lo); }

static void send_interference(uint64_t duration)
{
    struct RadioCommEventData intf = {
        .mChannel = radio_channel, .mPower = radio_power, .mError = OT_TX_TYPE_INTF, .mDuration = duration};

    report_state();
    otSimSendRadioCommInterferenceEvent(&intf);
    wake_pacer_now();
}

static void interferer_timer_fired(void)
{
    switch (radio_sub_state)
    {
    case RFSIM_RADIO_SUBSTATE_READY:
        radio_channel = current_channel;
        if (interferer_cca_failed)
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_CW_BACKOFF,
                          interferer_rand_range(0, OT_RADIO_WIFI_CWMIN_SLOTS) * OT_RADIO_WIFI_SLOT_TIME_US);
        }
        else
        {
            uint64_t delay = 1;
            if (tx_interferer_level < TX_INTERFERER_LEVEL_MAX)
            {
                delay = interferer_rand_range(1, INTERFERER_DELAY_RANGE_US -
                                                     (uint64_t)tx_interferer_level * INTERFERER_LEVEL_STEP_US);
            }
            set_sub_state(RFSIM_RADIO_SUBSTATE_AWAIT_CCA, delay);
        }
        break;

    case RFSIM_RADIO_SUBSTATE_CW_BACKOFF:
    case RFSIM_RADIO_SUBSTATE_AWAIT_CCA:
        set_sub_state(RFSIM_RADIO_SUBSTATE_TX_CCA, OT_RADIO_WIFI_CCA_TIME_US + FAILSAFE_TIME_US);
        if (send_sample(OT_RADIO_WIFI_CCA_TIME_US) < 0)
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_READY, 1);
        }
        break;

    case RFSIM_RADIO_SUBSTATE_TX_CCA_TO_TX:
    {
        uint64_t duration =
            interferer_rand_range(OT_RADIO_WIFI_MAX_TXTIME_US / 6, (uint64_t)OT_RADIO_WIFI_MAX_TXTIME_US + 1);
        send_interference(duration);
        set_sub_state(RFSIM_RADIO_SUBSTATE_TX_FRAME_ONGOING, duration + FAILSAFE_TIME_US);
        break;
    }

    default:
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, 1);
        break;
    }
}

static void interferer_deliver(const struct otns_radio_event *ev)
{
    if (ev->type != OT_SIM_EVENT_RADIO_CHAN_SAMPLE || radio_sub_state != RFSIM_RADIO_SUBSTATE_TX_CCA ||
        ev->data.mChannel != radio_channel)
    {
        return;
    }

    bool clear = ev->data.mError == OT_ERROR_NONE &&
                 (ev->data.mPower == (int8_t)OT_RADIO_RSSI_INVALID || ev->data.mPower < s_cca_threshold);

    if (clear)
    {
        interferer_cca_failed = false;
        set_sub_state(RFSIM_RADIO_SUBSTATE_TX_CCA_TO_TX, OT_RADIO_WIFI_SLOT_TIME_US);
    }
    else
    {
        interferer_cca_failed = true;
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, 1);
    }
}

static void start_pending_operation(void)
{
    if (radio_sub_state != RFSIM_RADIO_SUBSTATE_READY)
    {
        return;
    }
    if (tx_pending)
    {
        radio_state   = OT_RADIO_STATE_TRANSMIT;
        radio_channel = current_channel;
        tx_wait       = true;
        if (tx_cca)
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_TX_CCA, OT_RADIO_CCA_TIME_US + FAILSAFE_TIME_US);
            if (send_sample(OT_RADIO_CCA_TIME_US) < 0)
            {
                set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
                complete_tx(OT_ERROR_ABORT);
            }
        }
        else
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_TX_CCA_TO_TX, RFSIM_TURNAROUND_TIME_US);
        }
    }
    else if (sample_pending)
    {
        radio_channel = current_channel;
        set_sub_state(energy_scan ? RFSIM_RADIO_SUBSTATE_RX_ENERGY_SCAN : RFSIM_RADIO_SUBSTATE_TX_CCA,
                      sample_duration + FAILSAFE_TIME_US);
        if (send_sample(sample_duration) < 0)
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
            cancel_sample();
        }
    }
}

static void radio_timer_fired(void)
{
    if (tx_interferer_level > 0)
    {
        interferer_timer_fired();
        return;
    }

    struct otns_radio_event result = {0};

    result.data.mChannel = radio_channel;
    result.data.mError   = OT_ERROR_ABORT;
    switch (radio_sub_state)
    {
    case RFSIM_RADIO_SUBSTATE_STARTUP:
    case RFSIM_RADIO_SUBSTATE_IFS_WAIT:
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
        radio_channel = current_channel;
        start_pending_operation();
        break;

    case RFSIM_RADIO_SUBSTATE_TX_CCA:
    case RFSIM_RADIO_SUBSTATE_RX_ENERGY_SCAN:
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
        radio_channel = current_channel;
        if (tx_pending)
        {
            complete_tx(OT_ERROR_CHANNEL_ACCESS_FAILURE);
        }
        else if (sample_pending)
        {
            cancel_sample();
        }
        break;

    case RFSIM_RADIO_SUBSTATE_TX_CCA_TO_TX:
        if (tx_pending)
        {
            if (send_frame(tx_psdu, tx_len, false) < 0)
            {
                set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
                complete_tx(OT_ERROR_ABORT);
            }
        }
        else
        {
            sample_pending = false;
            set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
            radio_channel = current_channel;
            raise_embedded(&sample_result);
        }
        break;

    case RFSIM_RADIO_SUBSTATE_TX_FRAME_ONGOING:
        set_sub_state(RFSIM_RADIO_SUBSTATE_TX_TX_TO_RX, RFSIM_TURNAROUND_TIME_US);
        complete_tx(OT_ERROR_ABORT);
        break;

    case RFSIM_RADIO_SUBSTATE_TX_TX_TO_RX:
        radio_channel = current_channel;
        set_sub_state(RFSIM_RADIO_SUBSTATE_IFS_WAIT, tx_ifs_time() - RFSIM_TURNAROUND_TIME_US);
        break;

    case RFSIM_RADIO_SUBSTATE_TX_TX_TO_AIFS:
        set_sub_state(RFSIM_RADIO_SUBSTATE_TX_AIFS_WAIT, OT_RADIO_MAX_ACK_WAIT_US);
        break;

    case RFSIM_RADIO_SUBSTATE_TX_AIFS_WAIT:
        set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
        radio_channel = current_channel;
        complete_tx(OT_ERROR_NO_ACK);
        break;

    case RFSIM_RADIO_SUBSTATE_TX_ACK_RX_ONGOING:
        set_sub_state(RFSIM_RADIO_SUBSTATE_IFS_WAIT, tx_ifs_time());
        complete_tx(OT_ERROR_NO_ACK);
        break;

    case RFSIM_RADIO_SUBSTATE_RX_FRAME_ONGOING:
        set_sub_state(RFSIM_RADIO_SUBSTATE_IFS_WAIT, RFSIM_TURNAROUND_TIME_US);
        apply_radio_delayed_sleep();
        break;

    case RFSIM_RADIO_SUBSTATE_RX_AIFS_WAIT:
        result.type     = OTNS_RADIO_PREPARE_ACK;
        result.psdu_len = ack_psdu_len;
        memcpy(result.psdu, ack_psdu, ack_psdu_len);
        ack_psdu_len = 0;
        if (result.psdu_len != 0)
        {
            raise_embedded(&result);
        }
        if (ack_psdu_len != 0 && send_frame(ack_psdu, ack_psdu_len, true) == 0)
        {
            break;
        }
        nsi_print_warning("ieee802154_otns: failed to prepare or transmit ACK\n");
        set_sub_state(RFSIM_RADIO_SUBSTATE_RX_TX_TO_RX, RFSIM_TURNAROUND_TIME_US);
        break;

    case RFSIM_RADIO_SUBSTATE_RX_ACK_TX_ONGOING:
        set_sub_state(RFSIM_RADIO_SUBSTATE_RX_TX_TO_RX, RFSIM_TURNAROUND_TIME_US);
        apply_radio_delayed_sleep();
        break;

    case RFSIM_RADIO_SUBSTATE_RX_TX_TO_RX:
        set_sub_state(RFSIM_RADIO_SUBSTATE_IFS_WAIT, RFSIM_TURNAROUND_TIME_US);
        break;

    default:
        radio_event_time = NSI_NEVER;
        break;
    }
}

NSI_HW_EVENT(radio_event_time, radio_timer_fired, RADIO_TIMER_PRIO);

int nsi_otns_bottom_tx(const uint8_t *psdu, uint16_t len, bool cca)
{
    if (!connected || radio_state == OT_RADIO_STATE_SLEEP || radio_state == OT_RADIO_STATE_DISABLED)
    {
        return -EIO;
    }
    if (tx_interferer_level > 0)
    {
        return -EBUSY;
    }
    if (len < RADIO_MIN_FRAME_SIZE || len > OT_RADIO_FRAME_MAX_SIZE)
    {
        return -EINVAL;
    }
    if (tx_pending || sample_pending ||
        (radio_sub_state != RFSIM_RADIO_SUBSTATE_READY && radio_sub_state != RFSIM_RADIO_SUBSTATE_IFS_WAIT &&
         radio_sub_state != RFSIM_RADIO_SUBSTATE_STARTUP && radio_sub_state != RFSIM_RADIO_SUBSTATE_TX_TX_TO_RX))
    {
        return -EBUSY;
    }
    memcpy(tx_psdu, psdu, len);
    tx_len       = len;
    tx_wants_ack = (psdu[0] & (FCF_ACK_REQ_BIT & BYTE_MASK)) != 0;
    tx_cca       = cca;
    tx_pending   = true;
    radio_state  = OT_RADIO_STATE_TRANSMIT;
    start_pending_operation();
    wake_pacer_now();
    return 0;
}

int nsi_otns_bottom_tx_after(const uint8_t *psdu, uint16_t len, uint32_t delay_us)
{
    if (!connected)
    {
        return -1;
    }
    if (len > OT_RADIO_FRAME_MAX_SIZE)
    {
        return -1;
    }

    memcpy(ack_psdu, psdu, len);
    ack_psdu_len = len;
    set_sub_state(RFSIM_RADIO_SUBSTATE_RX_AIFS_WAIT, delay_us);
    return 0;
}

int nsi_otns_bottom_update_ack(const uint8_t *psdu, uint16_t len)
{
    if (radio_sub_state != RFSIM_RADIO_SUBSTATE_RX_AIFS_WAIT || len < ACK_FRAME_SIZE || len > OT_RADIO_FRAME_MAX_SIZE)
    {
        return -EINVAL;
    }
    memcpy(ack_psdu, psdu, len);
    ack_psdu_len = len;
    return 0;
}

static int request_sample(uint64_t duration, bool scan)
{
    if (!connected)
    {
        return -EIO;
    }
    if (tx_interferer_level > 0)
    {
        return -EBUSY;
    }
    if (tx_pending || sample_pending ||
        (radio_sub_state != RFSIM_RADIO_SUBSTATE_READY && radio_sub_state != RFSIM_RADIO_SUBSTATE_IFS_WAIT &&
         radio_sub_state != RFSIM_RADIO_SUBSTATE_STARTUP))
    {
        return -EBUSY;
    }
    sample_pending  = true;
    energy_scan     = scan;
    sample_duration = duration;
    start_pending_operation();
    wake_pacer_now();
    return 0;
}

int nsi_otns_bottom_cca(void) { return request_sample(OT_RADIO_CCA_TIME_US, false); }

int nsi_otns_bottom_energy_scan(uint16_t duration_ms)
{
    return request_sample((uint64_t)duration_ms * US_PER_MS, true);
}

void nsi_otns_bottom_set_channel(uint8_t channel) { current_channel = channel; }

uint8_t nsi_otns_bottom_get_channel(void) { return current_channel; }

void nsi_otns_bottom_set_state(uint8_t state)
{
    if (tx_interferer_level > 0)
    {
        sleep_pending = false;
        radio_state   = state;
        return;
    }
    if (state == OT_RADIO_STATE_RECEIVE && tx_pending && radio_state == OT_RADIO_STATE_TRANSMIT)
    {
        return;
    }
    if (state == OT_RADIO_STATE_SLEEP && (radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_FRAME_ONGOING ||
                                          radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_AIFS_WAIT ||
                                          radio_sub_state == RFSIM_RADIO_SUBSTATE_RX_ACK_TX_ONGOING))
    {
        sleep_pending = true;
        return;
    }
    if (state == OT_RADIO_STATE_RECEIVE && radio_state == OT_RADIO_STATE_DISABLED)
    {
        set_sub_state(RFSIM_RADIO_SUBSTATE_STARTUP, RFSIM_STARTUP_TIME_US);
    }
    else if (state == OT_RADIO_STATE_RECEIVE && radio_state == OT_RADIO_STATE_SLEEP &&
             radio_sub_state != RFSIM_RADIO_SUBSTATE_STARTUP)
    {
        set_sub_state(RFSIM_RADIO_SUBSTATE_STARTUP, RFSIM_RAMPUP_TIME_US);
    }
    if (state == OT_RADIO_STATE_SLEEP || state == OT_RADIO_STATE_DISABLED)
    {
        complete_tx(OT_ERROR_ABORT);
        if (sample_pending)
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_READY, NSI_NEVER);
            cancel_sample();
        }
        if (state == OT_RADIO_STATE_DISABLED)
        {
            set_sub_state(RFSIM_RADIO_SUBSTATE_STARTUP, RFSIM_STARTUP_TIME_US);
        }
    }
    sleep_pending = false;
    radio_state   = state;
    if (radio_sub_state == RFSIM_RADIO_SUBSTATE_READY || radio_sub_state == RFSIM_RADIO_SUBSTATE_STARTUP)
    {
        radio_channel = current_channel;
    }
    wake_pacer_now();
}

void nsi_otns_bottom_set_txpower(int8_t power)
{
    radio_power = power;
    wake_pacer_now();
}

int8_t nsi_otns_bottom_get_cca_threshold(void) { return s_cca_threshold; }

void nsi_otns_bottom_set_cca_threshold(int8_t threshold) { s_cca_threshold = threshold; }

void nsi_otns_bottom_send_uart(const uint8_t *buf, uint16_t len)
{
    if (!connected || buf == NULL || len == 0)
    {
        return;
    }

    otSimSendUartWriteEvent(buf, len);
    wake_pacer_now();
}

void nsi_otns_bottom_send_ext_addr(void)
{
    ext_addr_announced = true;

    if (!connected)
    {
        return;
    }

    announce_ext_addr();
    wake_pacer_now();
}

void nsi_otns_bottom_set_short_addr(const uint8_t *short_addr) { memcpy(own_short_addr, short_addr, SHORT_ADDR_SIZE); }

void nsi_otns_bottom_set_ext_addr(const uint8_t *ext_addr) { memcpy(own_ext_addr, ext_addr, OT_EXT_ADDRESS_SIZE); }

void nsi_otns_bottom_get_ext_addr(uint8_t *ext_addr) { memcpy(ext_addr, own_ext_addr, OT_EXT_ADDRESS_SIZE); }

void nsi_otns_bottom_set_pan_id(uint16_t pan_id) { own_pan_id = pan_id; }

void nsi_otns_bottom_send_status(const char *status, uint16_t len)
{
    if (!connected || status == NULL || len == 0)
    {
        return;
    }

    otSimSendOtnsStatusPushEvent(status, len);
    wake_pacer_now();
}

bool nsi_otns_bottom_get_event(struct otns_radio_event *ev)
{
    if (!deliver_ready)
    {
        return false;
    }

    *ev           = deliver_ev;
    deliver_ready = false;
    return true;
}

void nsi_otns_bottom_send_msg_to_host(uint8_t type, const uint8_t *data, uint16_t len)
{
    if (!connected || len < sizeof(struct MsgToHostEventData))
    {
        return;
    }

    otSimSendMsgToHostEvent(type, (struct MsgToHostEventData *)data,
                            (uint8_t *)data + sizeof(struct MsgToHostEventData),
                            len - sizeof(struct MsgToHostEventData));
    wake_pacer_now();
}

bool nsi_otns_bottom_host_msg_pending(void) { return host_msg_count > 0; }

bool nsi_otns_bottom_peek_host_msg(const struct otns_host_msg **msg)
{
    if (host_msg_count == 0)
    {
        return false;
    }

    *msg = &host_msgs[host_msg_head];
    return true;
}

void nsi_otns_bottom_pop_host_msg(void)
{
    if (host_msg_count > 0)
    {
        host_msg_head = (host_msg_head + 1) % HOST_MSG_QUEUE_LEN;
        host_msg_count--;
    }
}
