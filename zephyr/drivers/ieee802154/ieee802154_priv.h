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

#ifndef IEEE802154_OTNS_PRIV_H__
#define IEEE802154_OTNS_PRIV_H__

#include <stdbool.h>
#include <stdint.h>

#include "ot-rfsim/src/event-sim.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IEEE802154_OTNS_IRQ 4
#define IEEE802154_OTNS_ALARM_MILLI_IRQ 6
#define IEEE802154_OTNS_ALARM_MICRO_IRQ 7

#define OTNS_RADIO_PREPARE_ACK 0xff

enum otns_alarm
{
    OTNS_ALARM_MILLI,
    OTNS_ALARM_MICRO,
    OTNS_ALARM_COUNT,
};

struct otns_radio_event
{
    uint8_t                   type;
    struct RadioCommEventData data;
    uint64_t                  timestamp_us;
    uint16_t                  psdu_len;
    uint8_t                   psdu[OT_RADIO_FRAME_MAX_SIZE];
};

struct otns_host_msg
{
    uint8_t  type;
    uint16_t len;
    uint8_t  data[OT_EVENT_DATA_MAX_SIZE];
};

int nsi_otns_bottom_get_node_id(void);

bool nsi_otns_bottom_is_connected(void);

int nsi_otns_bottom_tx(const uint8_t *psdu, uint16_t len, bool cca);

int nsi_otns_bottom_tx_after(const uint8_t *psdu, uint16_t len, uint32_t delay_us);

int nsi_otns_bottom_update_ack(const uint8_t *psdu, uint16_t len);

int nsi_otns_bottom_cca(void);

int nsi_otns_bottom_energy_scan(uint16_t duration_ms);

void nsi_otns_bottom_set_channel(uint8_t channel);

uint8_t nsi_otns_bottom_get_channel(void);

void nsi_otns_bottom_set_state(uint8_t state);

void nsi_otns_bottom_set_txpower(int8_t power);

int8_t nsi_otns_bottom_get_cca_threshold(void);

void nsi_otns_bottom_set_cca_threshold(int8_t threshold);

void nsi_otns_bottom_send_uart(const uint8_t *buf, uint16_t len);

void nsi_otns_bottom_set_short_addr(const uint8_t *short_addr);

void nsi_otns_bottom_set_ext_addr(const uint8_t *ext_addr);

void nsi_otns_bottom_get_ext_addr(uint8_t *ext_addr);

void nsi_otns_bottom_set_pan_id(uint16_t pan_id);

void nsi_otns_bottom_send_ext_addr(void);

void nsi_otns_bottom_send_status(const char *status, uint16_t len);

void nsi_otns_bottom_send_msg_to_host(uint8_t type, const uint8_t *data, uint16_t len);

bool nsi_otns_bottom_host_msg_pending(void);

bool nsi_otns_bottom_peek_host_msg(const struct otns_host_msg **msg);

void nsi_otns_bottom_pop_host_msg(void);

bool nsi_otns_bottom_get_event(struct otns_radio_event *ev);

uint8_t nsi_otns_bottom_get_csl_accuracy(void);

uint64_t nsi_otns_bottom_get_time_us(void);

void nsi_otns_bottom_alarm_set_at(enum otns_alarm alarm, uint64_t delay_us);

void nsi_otns_bottom_alarm_cancel(enum otns_alarm alarm);

struct otInstance;

void otns_host_msg_process(struct otInstance *aInstance);

void otns_cli_input(struct otInstance *aInstance, const uint8_t *buf, uint16_t len);

void otns_host_forward_init(struct otInstance *aInstance);

void otns_host_forward_msg(struct otInstance *aInstance, const struct otns_host_msg *msg);

#ifdef __cplusplus
}
#endif

#endif /* IEEE802154_OTNS_PRIV_H__ */
