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
 * Portable (dependency-free) 802.15.4 frame parsing and address/ACK-bit
 * matching.
 *
 * This file is compiled into BOTH the embedded Zephyr/OpenThread image
 * (ieee802154.c, ieee802154_isr.c) and the host-side native_simulator
 * executable (ieee802154_bottom.c) - see ../../CMakeLists.txt. It must
 * therefore only use standard C and the portable OpenThread public API
 * (reached via "radio.h" -> <openthread/platform/radio.h>), never Zephyr-
 * or host-specific headers. This lets both sides make the exact same
 * "is this frame an ACK / ACK-requested / addressed to me" decision from a
 * single source of truth, without duplicating the logic.
 */

#include <string.h>

#include "ieee802154_frame.h"

static inline uint16_t frame_le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

static bool dst_pan_id_present_2015(uint8_t dst_mode, uint8_t src_mode, bool pc)
{
    if (dst_mode == ADDR_MODE_NONE)
    {
        return src_mode == ADDR_MODE_NONE && pc;
    }
    if (src_mode == ADDR_MODE_NONE || (src_mode == ADDR_MODE_EXT && dst_mode == ADDR_MODE_EXT))
    {
        return !pc;
    }
    return true;
}

static bool src_pan_id_present_2015(uint8_t dst_mode, uint8_t src_mode, bool pc)
{
    if (dst_mode == ADDR_MODE_EXT && src_mode == ADDR_MODE_EXT)
    {
        return false;
    }
    return src_mode != ADDR_MODE_NONE && !pc;
}

bool parse_frame(const uint8_t *psdu, uint16_t len, struct frame_addr_info *info)
{
    if (len < MIN_FRAME_SIZE)
    {
        return false;
    }

    memset(info, 0, sizeof(*info));
    info->fcf      = frame_le16(psdu);
    info->version  = (info->fcf & FCF_VERSION_MASK) >> FCF_VERSION_SHIFT;
    info->dst_mode = (info->fcf & FCF_DST_MODE_MASK) >> FCF_DST_MODE_SHIFT;
    info->src_mode = (info->fcf & FCF_SRC_MODE_MASK) >> FCF_SRC_MODE_SHIFT;
    info->dst_off  = FRAME_OFF_NONE;
    info->src_off  = FRAME_OFF_NONE;

    bool pc  = (info->fcf & FCF_PAN_COMPR_BIT) != 0;
    int  off = FCF_SIZE;

    if (!(info->version == IEEE802154_VERSION_2015 && (info->fcf & FCF_SEQ_SUPPR_BIT)))
    {
        if (off >= len)
        {
            return false;
        }
        info->seq = psdu[off];
        off += SEQ_NUM_SIZE;
    }

    bool dst_pan_present;
    bool src_pan_present;
    if (info->version == IEEE802154_VERSION_2015)
    {
        dst_pan_present = dst_pan_id_present_2015(info->dst_mode, info->src_mode, pc);
        src_pan_present = src_pan_id_present_2015(info->dst_mode, info->src_mode, pc);
    }
    else
    {
        dst_pan_present = info->dst_mode != ADDR_MODE_NONE;
        src_pan_present = info->src_mode != ADDR_MODE_NONE && !pc;
    }

    if (dst_pan_present)
    {
        if (off + PAN_ID_SIZE > len)
        {
            return false;
        }
        info->dst_pan         = frame_le16(&psdu[off]);
        info->dst_pan_present = true;
        off += PAN_ID_SIZE;
    }

    if (info->dst_mode != ADDR_MODE_NONE)
    {
        info->dst_off = off;
        off += (info->dst_mode == ADDR_MODE_EXT) ? OT_EXT_ADDRESS_SIZE : SHORT_ADDR_SIZE;
        if (off > len)
        {
            return false;
        }
        info->dst_broadcast =
            info->dst_mode == ADDR_MODE_SHORT && frame_le16(&psdu[info->dst_off]) == OT_RADIO_BROADCAST_SHORT_ADDR;
    }

    if (info->src_mode != ADDR_MODE_NONE)
    {
        if (src_pan_present)
        {
            if (off + PAN_ID_SIZE > len)
            {
                return false;
            }
            info->src_pan         = frame_le16(&psdu[off]);
            info->src_pan_present = true;
            off += PAN_ID_SIZE;
        }
        info->src_off = off;
        off += (info->src_mode == ADDR_MODE_EXT) ? OT_EXT_ADDRESS_SIZE : SHORT_ADDR_SIZE;
        if (off > len)
        {
            return false;
        }
    }

    return true;
}

bool frame_addr_matches(const uint8_t          *psdu,
                        uint16_t                len,
                        struct frame_addr_info *info,
                        uint16_t                pan_id,
                        const uint8_t          *own_short_addr,
                        const uint8_t          *own_ext_addr)
{
    if (!parse_frame(psdu, len, info))
    {
        return false;
    }
    if (info->dst_pan_present && info->dst_pan != OT_PANID_BROADCAST && info->dst_pan != pan_id)
    {
        return false;
    }

    if (info->dst_mode == ADDR_MODE_SHORT)
    {
        if (info->dst_off + SHORT_ADDR_SIZE > len)
        {
            return false;
        }
        return info->dst_broadcast || memcmp(&psdu[info->dst_off], own_short_addr, SHORT_ADDR_SIZE) == 0;
    }

    if (info->dst_mode == ADDR_MODE_EXT)
    {
        if (info->dst_off + OT_EXT_ADDRESS_SIZE > len)
        {
            return false;
        }
        return memcmp(&psdu[info->dst_off], own_ext_addr, OT_EXT_ADDRESS_SIZE) == 0;
    }

    return info->dst_mode == ADDR_MODE_NONE;
}

bool frame_is_ack(const uint8_t *psdu, uint16_t len)
{
    if (len < FCF_SIZE)
    {
        return false;
    }

    return (frame_le16(psdu) & FCF_FRAME_TYPE_MASK) == FCF_FRAME_TYPE_ACK;
}

bool frame_is_ack_requested(const uint8_t *psdu, uint16_t len)
{
    if (len < FCF_SIZE)
    {
        return false;
    }

    return (frame_le16(psdu) & FCF_ACK_REQ_BIT) != 0;
}
