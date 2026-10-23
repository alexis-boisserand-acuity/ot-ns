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

#ifndef IEEE802154_OTNS_FRAME_H__
#define IEEE802154_OTNS_FRAME_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radio.h"

#define FCS_SIZE 2
#define FCF_SIZE 2
#define MIN_FRAME_SIZE 3
#define SEQ_NUM_OFFSET 2
#define SHORT_ADDR_SIZE 2
#define PAN_ID_SIZE sizeof(otPanId)
#define ACK_FRAME_SIZE 5
#define FCF_FRAME_TYPE_MASK 0x0007
#define FCF_FRAME_PENDING_BIT 0x0010
#define FCF_ACK_REQ_BIT 0x0020
#define FCF_BYTE0_FRAME_PENDING_BIT (FCF_FRAME_PENDING_BIT & 0xff)

#define ADDR_MODE_SHORT 2
#define ADDR_MODE_EXT 3
#define FRAME_OFF_NONE (-1)

struct frame_addr_info
{
    uint16_t fcf;
    int      version;
    uint8_t  seq;
    int      dst_mode;
    int      dst_off;
    bool     dst_broadcast;
    bool     dst_pan_present;
    uint16_t dst_pan;
    int      src_mode;
    int      src_off;
    bool     src_pan_present;
    uint16_t src_pan;
};

struct ack_security_info
{
    bool     secured;
    uint8_t  key_id;
    uint32_t frame_counter;
};

uint16_t crc16(const uint8_t *data, size_t len);

bool parse_frame(const uint8_t *psdu, uint16_t len, struct frame_addr_info *info);
bool frame_is_for_me(const uint8_t *psdu, uint16_t len, struct frame_addr_info *info);

bool frame_addr_matches(const uint8_t          *psdu,
                        uint16_t                len,
                        struct frame_addr_info *info,
                        uint16_t                pan_id,
                        const uint8_t          *own_short_addr,
                        const uint8_t          *own_ext_addr);
bool frame_is_ack(const uint8_t *psdu, uint16_t len);
bool frame_is_ack_requested(const uint8_t *psdu, uint16_t len);

bool frame_pending_for(const uint8_t *psdu, uint16_t len);

int  fpb_short_add(uint16_t addr);
int  fpb_short_remove(uint16_t addr);
void fpb_short_clear(void);
int  fpb_ext_add(const uint8_t *addr);
int  fpb_ext_remove(const uint8_t *addr);
void fpb_ext_clear(void);

bool schedule_ack(const uint8_t *rx_psdu, uint16_t rx_len, int8_t rssi, struct ack_security_info *security);

int refresh_ack(uint8_t *psdu, uint16_t len);

struct ieee802154_config;
int configure_enh_ack_ie(const struct ieee802154_config *config);

struct ieee802154_key;

void set_mac_keys(const struct ieee802154_key *keys);

void set_frame_counter(uint32_t counter, bool only_if_larger);

struct net_pkt;

int encrypt_tx_frame(uint8_t *psdu, uint16_t len, struct net_pkt *pkt);

#endif /* IEEE802154_OTNS_FRAME_H__ */
