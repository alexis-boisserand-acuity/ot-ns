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

#include <errno.h>
#include <string.h>

#include <zephyr/irq.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include <openthread-core-config.h>
#include <openthread/link.h>
#include <openthread/platform/crypto.h>
#include <openthread/platform/time.h>

#include "utils/mac_frame.h"

#include "ieee802154_data.h"
#include "ieee802154_frame.h"
#include "ieee802154_priv.h"

#define LOG_MODULE_NAME ieee802154_otns
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(LOG_MODULE_NAME);

#define HEADER_IE_HDR_SIZE 2
#define HEADER_IE_LEN_MASK 0x7f
#define HEADER_IE_ID_SHIFT 7
#define HEADER_IE_ID_CSL 0x1a

#define LM_TOKEN_RSSI 0x01
#define LM_TOKEN_MARGIN 0x02
#define LM_TOKEN_LQI 0x03
#define LM_VENDOR_IE_TOKEN_OFFSET 4
#define LM_METRIC_RANGE_DBM 130

#define AIFS_TURNAROUND_US ((uint32_t)OT_RADIO_AIFS_TIME_US)

#define MAC_KEY_CURRENT 1
#define KEY_ID_MODE_1 1
#define MAX_ENH_ACK_IES_SIZE (MAX_ACK_IES * (HEADER_IE_HDR_SIZE + ACK_IE_MAX_CONTENT))

static uint8_t  ack_rx_psdu[OT_RADIO_FRAME_MAX_SIZE];
static uint16_t ack_rx_len;

/* ------------------------------------------------------------------------- */
/* OpenThread radio frame wrapper                                            */
/* ------------------------------------------------------------------------- */

static otRadioFrame radio_frame(const uint8_t *psdu, uint16_t length)
{
    otRadioFrame frame;

    memset(&frame, 0, sizeof(frame));
    frame.mPsdu    = (uint8_t *)psdu;
    frame.mLength  = length;
    frame.mChannel = nsi_otns_bottom_get_channel();

    return frame;
}

/* ------------------------------------------------------------------------- */
/* Auto-ACK frame-pending-bit (FPB) address list                             */
/* ------------------------------------------------------------------------- */

static struct fpb_list *fpb_list_for(bool extended) { return extended ? &data.fpb_ext : &data.fpb_short; }

static size_t fpb_addr_size(bool extended) { return extended ? OT_EXT_ADDRESS_SIZE : SHORT_ADDR_SIZE; }

static int fpb_find(bool extended, const uint8_t *addr)
{
    const struct fpb_list *list = fpb_list_for(extended);

    for (int i = 0; i < list->count; i++)
    {
        if (memcmp(list->entries[i], addr, fpb_addr_size(extended)) == 0)
        {
            return i;
        }
    }
    return -1;
}

int fpb_configure(bool extended, const uint8_t *addr, bool enabled)
{
    struct fpb_list *list = fpb_list_for(extended);
    int              idx;

    if (addr == NULL)
    {
        list->count = 0;
        return 0;
    }

    idx = fpb_find(extended, addr);

    if (enabled)
    {
        if (idx >= 0)
        {
            return 0;
        }
        if (list->count >= MAX_FPB_ENTRIES)
        {
            return -ENOMEM;
        }
        memcpy(list->entries[list->count++], addr, fpb_addr_size(extended));
        return 0;
    }

    if (idx < 0)
    {
        return -ENOENT;
    }
    memcpy(list->entries[idx], list->entries[--list->count], sizeof(list->entries[idx]));
    return 0;
}

bool frame_pending_for(const uint8_t *psdu, uint16_t len)
{
    otRadioFrame frame = radio_frame(psdu, len);
    otMacAddress src;
    uint8_t      addr_le[OT_EXT_ADDRESS_SIZE];

    if (!data.auto_ack_fpb_enabled)
    {
        return true;
    }

    if (otMacFrameGetSrcAddr(&frame, &src) != OT_ERROR_NONE)
    {
        return false;
    }

    switch (src.mType)
    {
    case OT_MAC_ADDRESS_TYPE_SHORT:
        sys_put_le16(src.mAddress.mShortAddress, addr_le);
        return fpb_find(false, addr_le) >= 0;

    case OT_MAC_ADDRESS_TYPE_EXTENDED:
        ext_addr_reverse(addr_le, src.mAddress.mExtAddress.m8);
        return fpb_find(true, addr_le) >= 0;

    default:
        return false;
    }
}

/* ------------------------------------------------------------------------- */
/* Enhanced-ACK building (CSL phase, Thread Link-Metrics IEs)                */
/* ------------------------------------------------------------------------- */

static __maybe_unused uint16_t csl_phase(uint32_t delay_us)
{
    uint32_t period_us = data.csl_period * OT_US_PER_TEN_SYMBOLS;
    uint32_t phr_us    = OT_RADIO_SYMBOLS_PER_OCTET * OT_RADIO_SYMBOL_TIME;
    uint32_t anchor_us = (uint32_t)(data.csl_expected_rx_time_ns / NSEC_PER_USEC) + phr_us;
    uint32_t tx_mhr_us = (uint32_t)otPlatTimeGet() + delay_us + OT_RADIO_SHR_PHR_DURATION_US;
    uint32_t diff      = ((anchor_us % period_us) - (tx_mhr_us % period_us) + period_us) % period_us;

    if (diff % OT_US_PER_TEN_SYMBOLS > 0)
    {
        diff += OT_US_PER_TEN_SYMBOLS;
    }
    return (uint16_t)(diff / OT_US_PER_TEN_SYMBOLS);
}

static void set_tx_csl_ie(otRadioFrame *frame, uint32_t delay_us)
{
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (data.csl_period != 0)
    {
        otMacFrameSetCslIe(frame, (uint16_t)data.csl_period, csl_phase(delay_us));
    }
#else
    ARG_UNUSED(frame);
    ARG_UNUSED(delay_us);
#endif
}

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
static bool ack_ie_matches_dst(const struct enh_ack_ie *ie, const otMacAddress *src)
{
    bool has_short_filter = ie->short_addr != OT_RADIO_BROADCAST_SHORT_ADDR;

    if (!has_short_filter && !ie->has_ext_filter)
    {
        return true;
    }
    if (has_short_filter && src->mType == OT_MAC_ADDRESS_TYPE_SHORT && src->mAddress.mShortAddress == ie->short_addr)
    {
        return true;
    }
    if (ie->has_ext_filter && src->mType == OT_MAC_ADDRESS_TYPE_EXTENDED)
    {
        return memcmp(src->mAddress.mExtAddress.m8, ie->ext_addr_be, OT_EXT_ADDRESS_SIZE) == 0;
    }
    return false;
}

static void fill_csl_ie_content(uint8_t *content) { sys_put_le16(csl_phase(AIFS_TURNAROUND_US), &content[0]); }

static void fill_link_metrics_ie_content(uint8_t *content, uint8_t content_len, int8_t rssi)
{
    for (int j = LM_VENDOR_IE_TOKEN_OFFSET; j < content_len; j++)
    {
        if (content[j] == LM_TOKEN_LQI)
        {
            content[j] = LQI_PERFECT;
        }
        else if (content[j] == LM_TOKEN_RSSI)
        {
            int r = rssi < -LM_METRIC_RANGE_DBM ? -LM_METRIC_RANGE_DBM : (rssi > 0 ? 0 : rssi);

            content[j] = (uint8_t)((r + LM_METRIC_RANGE_DBM) * UINT8_MAX / LM_METRIC_RANGE_DBM);
        }
        else if (content[j] == LM_TOKEN_MARGIN)
        {
            int margin = (int)rssi - RFSIM_RX_SENSITIVITY_DEFAULT_DBM;

            margin     = margin < 0 ? 0 : (margin > LM_METRIC_RANGE_DBM ? LM_METRIC_RANGE_DBM : margin);
            content[j] = (uint8_t)(margin * UINT8_MAX / LM_METRIC_RANGE_DBM);
        }
    }
}

static uint8_t build_enh_ack_ies(const otRadioFrame *rx_frame, int8_t rssi, uint8_t *out, uint8_t out_max)
{
    otMacAddress src;
    uint8_t      total = 0;

    if (otMacFrameGetSrcAddr(rx_frame, &src) != OT_ERROR_NONE)
    {
        return 0;
    }

    for (int i = 0; i < MAX_ACK_IES; i++)
    {
        struct enh_ack_ie *ie = &data.ack_ies[i];
        uint8_t            content[ACK_IE_MAX_CONTENT];
        uint16_t           hdr;

        if (!ie->valid || !ack_ie_matches_dst(ie, &src))
        {
            continue;
        }
        if (ie->element_id == HEADER_IE_ID_CSL && data.csl_period == 0)
        {
            continue;
        }
        if ((uint16_t)(total + sizeof(hdr) + ie->content_len) > out_max)
        {
            break;
        }

        memcpy(content, ie->content, ie->content_len);

        if (ie->element_id == HEADER_IE_ID_CSL)
        {
            fill_csl_ie_content(content);
        }
        else
        {
            fill_link_metrics_ie_content(content, ie->content_len, rssi);
        }

        hdr = (ie->content_len & HEADER_IE_LEN_MASK) | ((uint16_t)ie->element_id << HEADER_IE_ID_SHIFT);
        sys_put_le16(hdr, &out[total]);
        memcpy(&out[total + sizeof(hdr)], content, ie->content_len);
        total += sizeof(hdr) + ie->content_len;
    }

    return total;
}
#endif

/* ------------------------------------------------------------------------- */
/* Software transmit security (IEEE802154_HW_TX_SEC)                        */
/* ------------------------------------------------------------------------- */

void set_mac_keys(const struct ieee802154_key *keys)
{
    /* The OpenThread platform supplies previous, current, then next key. */
    for (int i = 0; i < MAX_MAC_KEYS; i++)
    {
        data.mac_keys[i].valid = false;
    }

    for (int i = 0; keys != NULL && i < MAX_MAC_KEYS && keys[i].key_value != NULL; i++)
    {
        data.mac_keys[i].key_id_mode = keys[i].key_id_mode;
        data.mac_keys[i].key_id      = (keys[i].key_id_mode != 0) ? *keys[i].key_id : 0;
        memcpy(data.mac_keys[i].key, keys[i].key_value, MAC_KEY_SIZE);
        data.mac_keys[i].valid = true;
    }
}

void set_frame_counter(uint32_t counter, bool only_if_larger)
{
    if (!only_if_larger || counter > data.mac_frame_counter)
    {
        data.mac_frame_counter = counter;
    }
}

static const uint8_t *find_mac_key(uint8_t key_id_mode, uint8_t key_id)
{
    for (int i = 0; i < MAX_MAC_KEYS; i++)
    {
        if (data.mac_keys[i].valid && data.mac_keys[i].key_id_mode == key_id_mode && data.mac_keys[i].key_id == key_id)
        {
            return data.mac_keys[i].key;
        }
    }
    return NULL;
}

static int aes_ccm_encrypt(otRadioFrame *frame, const uint8_t *key)
{
    otMacKeyMaterial material;
    otExtAddress     ext_addr;
    uint8_t          ext_addr_le[OT_EXT_ADDRESS_SIZE];
    int              ret = 0;

    memset(&material, 0, sizeof(material));

#if OPENTHREAD_CONFIG_PLATFORM_KEY_REFERENCES_ENABLE
    otCryptoKeyRef key_ref = 0;

    if (otPlatCryptoImportKey(&key_ref, OT_CRYPTO_KEY_TYPE_AES, OT_CRYPTO_KEY_ALG_AES_ECB, OT_CRYPTO_KEY_USAGE_ENCRYPT,
                              OT_CRYPTO_KEY_STORAGE_VOLATILE, key, MAC_KEY_SIZE) != OT_ERROR_NONE)
    {
        LOG_ERR("aes_ccm_encrypt: failed to import MAC key");
        return -EIO;
    }
    material.mKeyMaterial.mKeyRef = key_ref;
#else
    memcpy(material.mKeyMaterial.mKey.m8, key, MAC_KEY_SIZE);
#endif

    /* The CCM nonce is built from the big-endian extended address; the driver stores it in over-the-air order. */
    nsi_otns_bottom_get_ext_addr(ext_addr_le);
    ext_addr_reverse(ext_addr.m8, ext_addr_le);

    frame->mInfo.mTxInfo.mAesKey = &material;
    otMacFrameProcessTransmitAesCcm(frame, &ext_addr);

    if (!frame->mInfo.mTxInfo.mIsSecurityProcessed)
    {
        LOG_ERR("aes_ccm_encrypt: AES-CCM processing failed");
        ret = -EIO;
    }
    frame->mInfo.mTxInfo.mAesKey = NULL;

#if OPENTHREAD_CONFIG_PLATFORM_KEY_REFERENCES_ENABLE
    otPlatCryptoDestroyKey(key_ref);
#endif

    return ret;
}

static int secure_tx_frame(otRadioFrame *frame, bool header_updated)
{
    uint8_t        key_id  = otMacFrameGetKeyId(frame);
    bool           is_ack  = otMacFrameIsAck(frame);
    uint32_t       counter = 0;
    unsigned int   lock    = irq_lock();
    const uint8_t *mac_key;
    uint8_t        key[MAC_KEY_SIZE];

    if (!is_ack && !header_updated)
    {
        key_id = data.mac_keys[MAC_KEY_CURRENT].valid ? data.mac_keys[MAC_KEY_CURRENT].key_id : 0;
    }
    mac_key = key_id != 0 ? find_mac_key(KEY_ID_MODE_1, key_id) : NULL;

    if (mac_key != NULL)
    {
        memcpy(key, mac_key, sizeof(key));
        if (!header_updated)
        {
            counter = data.mac_frame_counter++;
        }
    }
    irq_unlock(lock);

    if (mac_key == NULL)
    {
        LOG_ERR("secure_tx_frame: no MAC key for key_id=%u", key_id);
        return -ENOENT;
    }

    if (!header_updated)
    {
        otMacFrameSetKeyId(frame, key_id);
        otMacFrameSetFrameCounter(frame, counter);
    }

    return aes_ccm_encrypt(frame, key);
}

int encrypt_tx_frame(uint8_t *psdu, uint16_t len, struct net_pkt *pkt)
{
    otRadioFrame frame          = radio_frame(psdu, len + FCS_SIZE);
    bool         header_updated = pkt != NULL && net_pkt_ieee802154_mac_hdr_rdy(pkt);
    otMacAddress src;
    int          ret;

    if (pkt != NULL && net_pkt_ieee802154_frame_secured(pkt))
    {
        return 0;
    }

    if (otMacFrameGetSrcAddr(&frame, &src) != OT_ERROR_NONE)
    {
        LOG_ERR("encrypt_tx_frame: unparsable frame len=%u", len);
        return -EINVAL;
    }

    if (!header_updated)
    {
        set_tx_csl_ie(&frame, AIFS_TURNAROUND_US);
    }

    if (!otMacFrameIsSecurityEnabled(&frame) || !otMacFrameIsKeyIdMode1(&frame))
    {
        return 0;
    }

    ret = secure_tx_frame(&frame, header_updated);

    if (ret == 0 && pkt != NULL)
    {
        net_pkt_set_ieee802154_mac_hdr_rdy(pkt, true);
        net_pkt_set_ieee802154_frame_secured(pkt, true);
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* ACK generation                                                            */
/* ------------------------------------------------------------------------- */

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
static int generate_enh_ack(otRadioFrame             *rx_frame,
                            bool                      pending,
                            int8_t                    rssi,
                            otRadioFrame             *ack_frame,
                            struct ack_security_info *security)
{
    uint8_t ies[MAX_ENH_ACK_IES_SIZE];
    uint8_t ies_len = build_enh_ack_ies(rx_frame, rssi, ies, sizeof(ies));

    if (otMacFrameGenerateEnhAck(rx_frame, pending, ies, ies_len, ack_frame) != OT_ERROR_NONE)
    {
        return -EINVAL;
    }

    if (otMacFrameIsSecurityEnabled(ack_frame))
    {
        int ret = secure_tx_frame(ack_frame, false);

        if (ret < 0)
        {
            return ret;
        }
        security->secured       = true;
        security->key_id        = otMacFrameGetKeyId(ack_frame);
        security->frame_counter = otMacFrameGetFrameCounter(ack_frame);
    }

    return 0;
}
#endif

static int generate_ack(otRadioFrame             *rx_frame,
                        int8_t                    rssi,
                        otRadioFrame             *ack_frame,
                        struct ack_security_info *security)
{
    bool pending = frame_pending_for(rx_frame->mPsdu, rx_frame->mLength);
    int  ret     = 0;

    ARG_UNUSED(rssi);
    ARG_UNUSED(security);

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    if (otMacFrameIsVersion2015(rx_frame))
    {
        ret = generate_enh_ack(rx_frame, pending, rssi, ack_frame, security);
    }
    else
#endif
    {
        otMacFrameGenerateImmAck(rx_frame, pending, ack_frame);
    }

    if (ret == 0)
    {
        uint16_t fcs_off = ack_frame->mLength - FCS_SIZE;

        sys_put_le16(crc16_ccitt(0, ack_frame->mPsdu, fcs_off), &ack_frame->mPsdu[fcs_off]);
    }

    return ret;
}

bool schedule_ack(const uint8_t *rx_psdu, uint16_t rx_len, int8_t rssi, struct ack_security_info *security)
{
    uint8_t      ack[OT_RADIO_FRAME_MAX_SIZE] = {0};
    otRadioFrame rx_frame                     = radio_frame(rx_psdu, rx_len);
    otRadioFrame ack_frame                    = radio_frame(ack, 0);
    int          ret;

    memset(security, 0, sizeof(*security));

    if (rx_len > sizeof(ack_rx_psdu))
    {
        LOG_ERR("Cannot schedule ACK for oversized frame: %u", rx_len);
        return false;
    }

    ret = generate_ack(&rx_frame, rssi, &ack_frame, security);
    if (ret < 0)
    {
        memset(security, 0, sizeof(*security));
        LOG_WRN("Failed to build ACK: %d", ret);
        return false;
    }

    ret = nsi_otns_bottom_tx_after(ack, ack_frame.mLength, AIFS_TURNAROUND_US);
    if (ret != 0)
    {
        memset(security, 0, sizeof(*security));
        LOG_WRN("Failed to schedule ACK: %d", ret);
        return false;
    }
    memcpy(ack_rx_psdu, rx_psdu, rx_len);
    ack_rx_len = rx_len;
    return true;
}

int refresh_ack(uint8_t *psdu, uint16_t len)
{
    otRadioFrame ack_frame = radio_frame(psdu, len);

    if (len < ACK_FRAME_SIZE || !otMacFrameIsAck(&ack_frame))
    {
        LOG_ERR("Cannot refresh invalid ACK");
        return -EINVAL;
    }

    if (frame_pending_for(ack_rx_psdu, ack_rx_len))
    {
        psdu[0] |= FCF_BYTE0_FRAME_PENDING_BIT;
    }
    else
    {
        psdu[0] &= ~FCF_BYTE0_FRAME_PENDING_BIT;
    }
    set_tx_csl_ie(&ack_frame, 0);

    if (otMacFrameIsSecurityEnabled(&ack_frame))
    {
        int ret = secure_tx_frame(&ack_frame, true);

        if (ret < 0)
        {
            return ret;
        }
    }

    sys_put_le16(crc16_ccitt(0, psdu, len - FCS_SIZE), &psdu[len - FCS_SIZE]);
    return 0;
}

static bool ack_ie_has_filter(const struct enh_ack_ie *ie, uint16_t short_addr, const uint8_t *ext_addr_be)
{
    return ie->short_addr == short_addr && ie->has_ext_filter == (ext_addr_be != NULL) &&
           (ext_addr_be == NULL || memcmp(ie->ext_addr_be, ext_addr_be, OT_EXT_ADDRESS_SIZE) == 0);
}

int configure_enh_ack_ie(const struct ieee802154_config *config)
{
    const uint8_t     *raw         = (const uint8_t *)config->ack_ie.header_ie;
    uint16_t           short_addr  = config->ack_ie.short_addr;
    const uint8_t     *ext_addr_be = config->ack_ie.ext_addr;
    bool               remove_all  = config->ack_ie.purge_ie || raw == NULL;
    uint8_t            element_id  = 0;
    uint8_t            content_len = 0;
    struct enh_ack_ie *slot        = NULL;
    struct enh_ack_ie *free_slot   = NULL;

    if (!remove_all)
    {
        uint16_t hdr = sys_get_le16(raw);

        content_len = hdr & HEADER_IE_LEN_MASK;
        element_id  = (hdr >> HEADER_IE_ID_SHIFT) & 0xff;

        if (content_len > ACK_IE_MAX_CONTENT)
        {
            return -EINVAL;
        }
    }

    for (int i = 0; i < MAX_ACK_IES; i++)
    {
        struct enh_ack_ie *ie = &data.ack_ies[i];

        if (!ie->valid)
        {
            free_slot = free_slot != NULL ? free_slot : ie;
        }
        else if (ack_ie_has_filter(ie, short_addr, ext_addr_be))
        {
            if (remove_all)
            {
                ie->valid = false;
            }
            else if (ie->element_id == element_id)
            {
                slot = ie;
            }
        }
    }

    if (remove_all)
    {
        return 0;
    }

    if (content_len == 0)
    {
        if (slot != NULL)
        {
            slot->valid = false;
        }
        return 0;
    }

    slot = slot != NULL ? slot : free_slot;
    if (slot == NULL)
    {
        return -ENOMEM;
    }

    slot->element_id  = element_id;
    slot->content_len = content_len;
    memcpy(slot->content, raw + HEADER_IE_HDR_SIZE, content_len);
    slot->short_addr     = short_addr;
    slot->has_ext_filter = ext_addr_be != NULL;
    if (ext_addr_be != NULL)
    {
        memcpy(slot->ext_addr_be, ext_addr_be, OT_EXT_ADDRESS_SIZE);
    }
    slot->valid = true;

    return 0;
}
