/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Embedded (Zephyr CPU) side of the OTNS virtual IEEE 802.15.4 radio driver.
 *
 * This side implements the Zephyr ieee802154_radio_api and translates the radio
 * operations into calls to the runner ("bottom") side, which owns the real Unix
 * domain socket to OTNS. Received radio events are delivered from the runner
 * side through IEEE802154_OTNS_IRQ and processed by otns_isr().
 */

#define DT_DRV_COMPAT zephyr_ieee802154_otns

#define LOG_MODULE_NAME ieee802154_otns
#if defined(CONFIG_IEEE802154_OTNS_LOG_LEVEL)
#define LOG_LEVEL CONFIG_IEEE802154_OTNS_LOG_LEVEL
#else
#define LOG_LEVEL LOG_LEVEL_INF
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/sys/byteorder.h>

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <zephyr/net/openthread.h>
#endif

#include "ieee802154_otns_priv.h"

/*
 * ot-rfsim's own radio.h (and the radio-parameters.h / OpenThread radio.h it
 * pulls in), reused below for timing and channel/threshold constants instead
 * of redefining them. Embedded-side only; see zephyr/CMakeLists.txt.
 */
#include "radio.h"
#include <openthread/link.h>
#include <openthread/platform/time.h>

#if defined(CONFIG_OPENTHREAD_OTNS)
#include <openthread/platform/otns.h>

/*
 * OpenThread core emits status strings (role, rloc16, partition id, parent,
 * neighbour changes, ...) through otPlatOtnsStatus() whenever they change. OTNS
 * consumes these to visualise a node in its web UI; a node that never reports
 * its role stays greyed out ("disabled") even after it has attached. Override
 * the weak default in the OpenThread library and forward the status to OTNS as
 * an OT_SIM_EVENT_OTNS_STATUS_PUSH event via the runner side.
 */
void otPlatOtnsStatus(const char *aStatus)
{
	if (aStatus == NULL) {
		return;
	}

	nsi_otns_bottom_send_status(aStatus, (uint16_t)strlen(aStatus));
}
#endif /* CONFIG_OPENTHREAD_OTNS */

/* IEEE 802.15.4 MAC framing constants. */
#define FCS_SIZE           2
#define FCF_SIZE           2
#define MIN_FRAME_SIZE     3
#define SEQ_NUM_SIZE       1
#define SHORT_ADDR_SIZE    2
#define EXT_ADDR_SIZE      8
#define PAN_ID_SIZE        2
#define ACK_FRAME_SIZE     5
#define FCF_FRAME_TYPE_MASK 0x0007
#define FCF_FRAME_TYPE_ACK  0x0002
#define FCF_ACK_CONTROL_BYTE_0 0x02
#define FCF_ACK_CONTROL_BYTE_1 0x00
#define FCF_FRAME_PENDING_BIT 0x0010 /* bit 4 */
#define FCF_ACK_REQ_BIT     0x0020 /* bit 5 */
#define FCF_PAN_COMPR_BIT   0x0040 /* bit 6 */
#define FCF_DST_MODE_MASK   0x0c00 /* bits 10-11 */
#define FCF_DST_MODE_SHIFT  10
#define FCF_SRC_MODE_MASK   0xc000 /* bits 14-15 */
#define FCF_SRC_MODE_SHIFT  14
#define FCF_VERSION_MASK    0x3000 /* bits 12-13 */
#define FCF_VERSION_SHIFT   12
#define FCF_SEQ_SUPPR_BIT   0x0100 /* bit 8 (2015 frames) */
#define IEEE802154_VERSION_2015 2
#define BROADCAST_SHORT_ADDR 0xffff
#define BYTE_MASK           0xff
#define CRC_POLY_KERMIT     0x8408U
#define CRC_INIT            0

#define ADDR_MODE_NONE  0
#define ADDR_MODE_SHORT 2
#define ADDR_MODE_EXT   3

/* Header IE (Information Element), IEEE 802.15.4-2015 section 7.4.2.1. */
#define HEADER_IE_LEN_MASK  0x7f
#define HEADER_IE_ID_SHIFT  7
#define FCF_IE_PRESENT_BIT  0x0080 /* bit 7 */
#define HEADER_IE_ID_CSL    0x1a

/* Thread vendor-specific Enhanced-ACK Probing (Link Metrics) IE, Thread 1.2 4.11.3.4.4.6. */
#define LM_TOKEN_RSSI   0x01
#define LM_TOKEN_MARGIN 0x02
#define LM_TOKEN_LQI    0x03

/* Max stored Enhanced-ACK header IE templates (IEEE802154_CONFIG_ENH_ACK_HEADER_IE). */
#define MAX_ACK_IES 4
#define ACK_IE_MAX_CONTENT (OT_ACK_IE_MAX_SIZE - 2)

/* LQI and RSSI values. */
#define LQI_PERFECT         255
#define RSSI_ZERO_DBM       0

/* CCA timeout in milliseconds. */
#define CCA_TIMEOUT_MS      10

/* Max entries in each (short/extended) ACK frame-pending address table. */
#define MAX_FPB_ENTRIES 32

/* Node ID defaults. */
#define DEFAULT_NODE_ID     1
#define BIT_SHIFT_24        24
#define BIT_SHIFT_16        16
#define BIT_SHIFT_8         8

/* EUI-64 prefix for ot-rfsim compatibility. */
#define EUI64_BYTE_0        0x18
#define EUI64_BYTE_1        0xb4
#define EUI64_BYTE_2        0x30
#define EUI64_BYTE_3        0x00

/*
 * ACK turnaround (AIFS). IEEE 802.15.4 requires an acknowledgement to be sent
 * exactly aTurnaroundTime after the acknowledged frame ends on air.
 * Transmitting it sooner makes the peer - whose radio is still switching from
 * TX to RX during that turnaround - miss the ACK, so it keeps retransmitting
 * the frame. The ACK must therefore be delayed by this amount in virtual time.
 */
#define AIFS_TURNAROUND_US ((uint32_t)OT_RADIO_AIFS_TIME_US)

/*
 * Extra time to wait for an ACK once the transmitted frame is fully on air.
 * OTNS delivers the frame to the peer only after its whole air-time elapses (in
 * virtual time), then the peer's ACK needs its own air-time to travel back, so
 * the ACK-wait window must span the transmitted frame's air-time plus this
 * allowance (ACK air-time + margin). A too-short window would expire before the
 * frame even finishes transmitting, yielding spurious NoAck errors.
 */
#define ACK_ALLOWANCE_US 1000U

#define OPENTHREAD_MTU 1280

/*
 * A configured Enhanced-ACK header IE template (IEEE802154_CONFIG_ENH_ACK_HEADER_IE),
 * matched by destination address at ACK-build time. `content` is the raw
 * post-header bytes as delivered by configure(): for a CSL IE that's
 * phase+period (phase overwritten on-the-fly per ACK, see csl_phase()); for
 * the Link-Metrics vendor IE it's OUI+subtype+LM_TOKEN_* placeholders
 * (substituted with the real value per ACK, see build_enh_ack_ies()).
 */
struct enh_ack_ie {
	bool valid;
	uint8_t element_id;
	uint8_t content_len;
	uint8_t content[ACK_IE_MAX_CONTENT];
	bool has_short_filter;
	uint16_t short_addr;                /* native byte order */
	bool has_ext_filter;
	uint8_t ext_addr_be[EXT_ADDR_SIZE]; /* big-endian, as delivered by configure() */
};

struct ctx {
	struct net_if *iface;
	uint8_t mac_addr[8];

	struct k_sem tx_wait;
	struct k_sem cca_wait;

	volatile int tx_result;       /* 0 or -errno */
	volatile bool cca_channel_free;

	/* ACK reception state for an ongoing transmission. */
	bool tx_wants_ack;
	uint8_t tx_seq;
	uint8_t ack_psdu[OTNS_PSDU_MAX];
	volatile uint16_t ack_len;    /* incl. FCS, 0 if no ACK received */

	/* Filter state. */
	uint8_t pan_id[2];
	uint8_t short_addr[2];
	uint8_t ext_addr[8];

	/* ACK frame-pending source-match table (IEEE802154_CONFIG_ACK_FPB). */
	bool auto_ack_fpb_enabled;
	uint16_t fpb_short[MAX_FPB_ENTRIES];
	uint8_t fpb_short_count;
	uint8_t fpb_ext[MAX_FPB_ENTRIES][EXT_ADDR_SIZE];
	uint8_t fpb_ext_count;

	/* Energy-scan (ed_scan) request in flight, if any. */
	bool ed_scan_pending;
	energy_scan_done_cb_t ed_done_cb;

	/* Set while a deferred auto-ACK (schedule_ack()) has not transmitted yet. */
	bool ack_tx_pending;
	/* stop() was called while ack_tx_pending; apply once the ACK completes. */
	bool sleep_pending;

	/* CSL (Coordinated Sample Listening) receiver state. */
	uint32_t csl_period;             /* units of 10 symbols; 0 = disabled */
	int64_t csl_expected_rx_time_ns; /* IEEE802154_CONFIG_EXPECTED_RX_TIME, raw net_time_t */

	/* Configured Enhanced-ACK header IEs (CSL / Link-Metrics probing). */
	struct enh_ack_ie ack_ies[MAX_ACK_IES];

	uint8_t channel;
	int8_t txpower;
	bool started;
};

static struct ctx data;

/* Singleton device reference, used from the ISR context. */
static const struct device *radio_dev;

/* CRC-16/CCITT (KERMIT), reflected, poly 0x1021 -> 0x8408, init 0. */
static uint16_t crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = CRC_INIT;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++) { /* Process 8 bits per byte */
			crc = (crc & 1U) ? (crc >> 1) ^ CRC_POLY_KERMIT : (crc >> 1);
		}
	}

	return crc;
}

static int err_to_errno(uint8_t error)
{
	switch (error) {
	case OTNS_ERROR_NONE:
		return 0;
	case OTNS_ERROR_CHANNEL_ACCESS_FAILURE:
		return -EBUSY;
	case OTNS_ERROR_ABORT:
		return -EIO;
	default:
		return -EIO;
	}
}

/*
 * Result of parsing an 802.15.4 MAC header, enough for ACK generation:
 * addressing (both destination and source, needed since an ACK's destination
 * is the original frame's *source*) and PAN ids (needed to pick the ACK's own
 * PAN id per the 802.15.4-2015 rules, see build_enh_ack()).
 */
struct frame_addr_info {
	uint16_t fcf;
	int version;
	uint8_t seq;
	int dst_mode;
	int dst_off;         /* offset of dst address bytes, -1 if none */
	bool dst_pan_present;
	uint16_t dst_pan;
	int src_mode;
	int src_off;         /* offset of src address bytes, -1 if none */
	bool src_pan_present;
	uint16_t src_pan;
};

/*
 * Minimal 802.15.4 header parser sufficient for ACK generation. Assumes the
 * common frame shape (destination PAN id present whenever a destination
 * address is present); the full IEEE 802.15.4-2015 addressing-mode table has
 * additional edge cases not needed for the frames this driver deals with.
 */
static int parse_frame(const uint8_t *psdu, uint16_t len, struct frame_addr_info *info)
{
	int off;

	if (len < MIN_FRAME_SIZE) {
		return -1;
	}

	memset(info, 0, sizeof(*info));
	info->fcf = sys_get_le16(psdu);
	info->version = (info->fcf & FCF_VERSION_MASK) >> FCF_VERSION_SHIFT;
	info->dst_mode = (info->fcf & FCF_DST_MODE_MASK) >> FCF_DST_MODE_SHIFT;
	info->src_mode = (info->fcf & FCF_SRC_MODE_MASK) >> FCF_SRC_MODE_SHIFT;
	info->dst_off = -1;
	info->src_off = -1;

	off = FCF_SIZE;
	if (!(info->version == IEEE802154_VERSION_2015 && (info->fcf & FCF_SEQ_SUPPR_BIT))) {
		if (off >= len) {
			return -1;
		}
		info->seq = psdu[off];
		off += SEQ_NUM_SIZE;
	}

	if (info->dst_mode != ADDR_MODE_NONE) {
		if (off + PAN_ID_SIZE > len) {
			return -1;
		}
		info->dst_pan = sys_get_le16(&psdu[off]);
		info->dst_pan_present = true;
		off += PAN_ID_SIZE;

		info->dst_off = off;
		off += (info->dst_mode == ADDR_MODE_EXT) ? EXT_ADDR_SIZE : SHORT_ADDR_SIZE;
		if (off > len) {
			return -1;
		}
	}

	/* Source PAN id is elided when the PAN ID Compression bit is set. */
	if (info->src_mode != ADDR_MODE_NONE) {
		if (!(info->fcf & FCF_PAN_COMPR_BIT)) {
			if (off + PAN_ID_SIZE > len) {
				return -1;
			}
			info->src_pan = sys_get_le16(&psdu[off]);
			info->src_pan_present = true;
			off += PAN_ID_SIZE;
		}
		info->src_off = off;
		off += (info->src_mode == ADDR_MODE_EXT) ? EXT_ADDR_SIZE : SHORT_ADDR_SIZE;
		if (off > len) {
			return -1;
		}
	}

	return 0;
}

static bool frame_is_for_me(const uint8_t *psdu, uint16_t len, struct frame_addr_info *info)
{
	if (parse_frame(psdu, len, info) < 0) {
		return false;
	}

	if (info->dst_mode == ADDR_MODE_SHORT) {
		if (info->dst_off + SHORT_ADDR_SIZE > len) {
			return false;
		}
		/* Broadcast short address is never ACKed. */
		if (sys_get_le16(&psdu[info->dst_off]) == BROADCAST_SHORT_ADDR) {
			return false;
		}
		return memcmp(&psdu[info->dst_off], data.short_addr, SHORT_ADDR_SIZE) == 0;
	} else if (info->dst_mode == ADDR_MODE_EXT) {
		if (info->dst_off + EXT_ADDR_SIZE > len) {
			return false;
		}
		/*
		 * Both the on-air destination address and the extended address
		 * that OpenThread programs through the IEEE_ADDR filter are in
		 * little-endian (on-air) byte order, so they compare directly.
		 */
		return memcmp(&psdu[info->dst_off], data.ext_addr, EXT_ADDR_SIZE) == 0;
	}

	return false;
}

static bool fpb_short_contains(uint16_t addr)
{
	for (int i = 0; i < data.fpb_short_count; i++) {
		if (data.fpb_short[i] == addr) {
			return true;
		}
	}
	return false;
}

static bool fpb_ext_contains(const uint8_t *addr)
{
	for (int i = 0; i < data.fpb_ext_count; i++) {
		if (memcmp(data.fpb_ext[i], addr, EXT_ADDR_SIZE) == 0) {
			return true;
		}
	}
	return false;
}

static int fpb_short_add(uint16_t addr)
{
	if (fpb_short_contains(addr)) {
		return 0;
	}
	if (data.fpb_short_count >= MAX_FPB_ENTRIES) {
		return -ENOMEM;
	}
	data.fpb_short[data.fpb_short_count++] = addr;
	return 0;
}

static int fpb_short_remove(uint16_t addr)
{
	for (int i = 0; i < data.fpb_short_count; i++) {
		if (data.fpb_short[i] == addr) {
			data.fpb_short[i] = data.fpb_short[--data.fpb_short_count];
			return 0;
		}
	}
	return -ENOENT;
}

static int fpb_ext_add(const uint8_t *addr)
{
	if (fpb_ext_contains(addr)) {
		return 0;
	}
	if (data.fpb_ext_count >= MAX_FPB_ENTRIES) {
		return -ENOMEM;
	}
	memcpy(data.fpb_ext[data.fpb_ext_count++], addr, EXT_ADDR_SIZE);
	return 0;
}

static int fpb_ext_remove(const uint8_t *addr)
{
	for (int i = 0; i < data.fpb_ext_count; i++) {
		if (memcmp(data.fpb_ext[i], addr, EXT_ADDR_SIZE) == 0) {
			memcpy(data.fpb_ext[i], data.fpb_ext[--data.fpb_ext_count], EXT_ADDR_SIZE);
			return 0;
		}
	}
	return -ENOENT;
}

/*
 * Whether the ACK for a received frame should have its Frame-Pending bit set,
 * i.e. whether the frame's sender has indirect data queued. Mirrors
 * ot-rfsim's hasFramePending(): until AUTO_ACK_FPB is enabled, the bit is
 * always set (legacy behaviour, matches sSrcMatchEnabled == false there);
 * once enabled, it is set only for addresses in the FPB table.
 */
static bool frame_pending_for(const uint8_t *psdu, uint16_t len, const struct frame_addr_info *info)
{
	if (!data.auto_ack_fpb_enabled) {
		return true;
	}

	if (info->src_mode == ADDR_MODE_SHORT) {
		if (info->src_off + SHORT_ADDR_SIZE > len) {
			return false;
		}
		return fpb_short_contains(sys_get_le16(&psdu[info->src_off]));
	} else if (info->src_mode == ADDR_MODE_EXT) {
		if (info->src_off + EXT_ADDR_SIZE > len) {
			return false;
		}
		return fpb_ext_contains(&psdu[info->src_off]);
	}

	return false;
}

/*
 * CSL Phase for an Enhanced-ACK about to be sent AIFS_TURNAROUND_US from now.
 * Mirrors ot-rfsim's getCslPhase(): the phase is the time between the start of
 * this ACK's MHR and the next predicted CSL sample instant, in units of 10
 * symbols. csl_expected_rx_time_ns is reconstructed to that same "start of
 * MHR" reference by adding back the one-octet PHR duration that Zephyr's
 * generic OT radio layer subtracts before calling configure() (see
 * otPlatRadioUpdateCslSampleTime() in modules/openthread/platform/radio.c).
 */
static uint16_t csl_phase(void)
{
	uint32_t period_us = data.csl_period * OT_US_PER_TEN_SYMBOLS;
	uint32_t phr_us = OT_RADIO_SYMBOLS_PER_OCTET * OT_RADIO_SYMBOL_TIME;
	uint32_t anchor_us = (uint32_t)(data.csl_expected_rx_time_ns / NSEC_PER_USEC) + phr_us;
	uint32_t tx_mhr_us = (uint32_t)otPlatTimeGet() + AIFS_TURNAROUND_US + OT_RADIO_SHR_PHR_DURATION_US;
	uint32_t diff = ((anchor_us % period_us) - (tx_mhr_us % period_us) + period_us) % period_us;

	if (diff % OT_US_PER_TEN_SYMBOLS > 0) {
		diff += OT_US_PER_TEN_SYMBOLS;
	}
	return (uint16_t)(diff / OT_US_PER_TEN_SYMBOLS);
}

/* Whether a configured Enhanced-ACK header IE applies to the ACK's destination. */
static bool ack_ie_matches_dst(const struct enh_ack_ie *ie, const uint8_t *psdu,
				const struct frame_addr_info *info)
{
	if (!ie->has_short_filter && !ie->has_ext_filter) {
		return true; /* fallback: no filter configured, applies to all destinations */
	}
	if (ie->has_short_filter && info->src_mode == ADDR_MODE_SHORT &&
	    sys_get_le16(&psdu[info->src_off]) == ie->short_addr) {
		return true;
	}
	if (ie->has_ext_filter && info->src_mode == ADDR_MODE_EXT) {
		uint8_t ext_be[EXT_ADDR_SIZE];

		for (int i = 0; i < EXT_ADDR_SIZE; i++) {
			ext_be[i] = psdu[info->src_off + EXT_ADDR_SIZE - 1 - i];
		}
		return memcmp(ext_be, ie->ext_addr_be, EXT_ADDR_SIZE) == 0;
	}
	return false;
}

/*
 * Appends every configured Enhanced-ACK header IE (CSL / Link-Metrics probing
 * vendor IE) matching the ACK's destination into `out`. CSL's phase and the
 * Link-Metrics tokens (LM_TOKEN_*) are computed fresh for this specific ACK;
 * everything else is copied from the template stored by configure().
 */
static uint16_t build_enh_ack_ies(const uint8_t *rx_psdu, const struct frame_addr_info *info,
				   int8_t rssi, uint8_t *out, uint16_t out_max)
{
	uint16_t total = 0;

	for (int i = 0; i < MAX_ACK_IES; i++) {
		struct enh_ack_ie *ie = &data.ack_ies[i];
		uint8_t content[ACK_IE_MAX_CONTENT];
		uint16_t hdr;

		if (!ie->valid || !ack_ie_matches_dst(ie, rx_psdu, info)) {
			continue;
		}
		if (ie->element_id == HEADER_IE_ID_CSL && data.csl_period == 0) {
			continue; /* CSL was disabled after this template was stored */
		}
		if ((uint16_t)(total + 2 + ie->content_len) > out_max) {
			break; /* would overflow the ACK buffer; drop remaining IEs */
		}

		memcpy(content, ie->content, ie->content_len);

		if (ie->element_id == HEADER_IE_ID_CSL) {
			sys_put_le16(csl_phase(), &content[0]); /* content[2..3] = period, unchanged */
		} else {
			/* Vendor IE: content[0..2]=OUI, [3]=subtype, [4..]=LM_TOKEN_* placeholders. */
			for (int j = 4; j < ie->content_len; j++) {
				if (content[j] == LM_TOKEN_LQI) {
					content[j] = LQI_PERFECT;
				} else if (content[j] == LM_TOKEN_RSSI) {
					int r = rssi < -130 ? -130 : (rssi > 0 ? 0 : rssi);

					content[j] = (uint8_t)((r + 130) * 255 / 130);
				} else if (content[j] == LM_TOKEN_MARGIN) {
					int margin = (int)rssi - RFSIM_RX_SENSITIVITY_DEFAULT_DBM;

					margin = margin < 0 ? 0 : (margin > 130 ? 130 : margin);
					content[j] = (uint8_t)(margin * 255 / 130);
				}
			}
		}

		hdr = (ie->content_len & HEADER_IE_LEN_MASK) |
		      ((uint16_t)ie->element_id << HEADER_IE_ID_SHIFT);
		sys_put_le16(hdr, &out[total]);
		memcpy(&out[total + 2], content, ie->content_len);
		total += 2 + ie->content_len;
	}

	return total;
}

/*
 * Builds an IEEE 802.15.4-2015 Enhanced-ACK for a version-2015 received frame,
 * mirroring OT core's TxFrame::GenerateEnhAck(): destination = the received
 * frame's source (this ACK carries no source address of its own), Dst PAN id =
 * the received frame's source PAN id if present else its destination PAN id,
 * and no Header Termination IE (this ACK never carries IEs of any other kind
 * nor a MAC payload).
 */
static int build_enh_ack(const uint8_t *rx_psdu, uint16_t rx_len, const struct frame_addr_info *info,
			  int8_t rssi, uint8_t *ack, uint16_t *ack_len_out)
{
	uint16_t fcf;
	uint16_t off = FCF_SIZE + SEQ_NUM_SIZE;
	uint16_t dst_pan;
	uint16_t ie_len;
	uint16_t fcs;
	uint8_t addr_len;

	if (info->src_mode == ADDR_MODE_NONE || info->src_off < 0) {
		return -1; /* nothing to address the ACK to */
	}
	if (!info->src_pan_present && !info->dst_pan_present) {
		return -1;
	}
	dst_pan = info->src_pan_present ? info->src_pan : info->dst_pan;
	addr_len = (info->src_mode == ADDR_MODE_EXT) ? EXT_ADDR_SIZE : SHORT_ADDR_SIZE;

	fcf = FCF_FRAME_TYPE_ACK | ((uint16_t)IEEE802154_VERSION_2015 << FCF_VERSION_SHIFT) |
	      ((uint16_t)info->src_mode << FCF_DST_MODE_SHIFT);
	if (frame_pending_for(rx_psdu, rx_len, info)) {
		fcf |= FCF_FRAME_PENDING_BIT;
	}

	ack[FCF_SIZE] = info->seq;
	sys_put_le16(dst_pan, &ack[off]);
	off += PAN_ID_SIZE;
	memcpy(&ack[off], &rx_psdu[info->src_off], addr_len);
	off += addr_len;

	ie_len = build_enh_ack_ies(rx_psdu, info, rssi, &ack[off],
				   (uint16_t)(OTNS_PSDU_MAX - off - FCS_SIZE));
	if (ie_len > 0) {
		fcf |= FCF_IE_PRESENT_BIT;
	}
	off += ie_len;

	sys_put_le16(fcf, &ack[0]);

	fcs = crc16(ack, off);
	ack[off] = fcs & BYTE_MASK;
	ack[off + 1] = fcs >> 8;
	*ack_len_out = off + FCS_SIZE;
	return 0;
}

/*
 * Build the ACK for a received frame (Immediate for legacy versions, Enhanced
 * for IEEE 802.15.4-2015 ones, with CSL / Link-Metrics IE injection) and hand
 * it to the runner side to be transmitted AIFS_TURNAROUND_US later, so it
 * lands in the peer's ACK-wait window instead of during its TX->RX turnaround.
 * The delay is timed on the runner's microsecond-resolution HW clock rather
 * than with a k_timer, because the Zephyr kernel tick is too coarse to
 * represent a 192 us interval (and raising the tick rate would slow the
 * simulation down).
 */
static void schedule_ack(const uint8_t *rx_psdu, uint16_t rx_len, int8_t rssi,
			  const struct frame_addr_info *info)
{
	uint8_t ack[OTNS_PSDU_MAX];
	uint16_t ack_len;
	uint16_t fcs;

	if (info->version == IEEE802154_VERSION_2015) {
		if (build_enh_ack(rx_psdu, rx_len, info, rssi, ack, &ack_len) < 0) {
			return;
		}
	} else {
		ack[0] = FCF_ACK_CONTROL_BYTE_0; /* FCF: ACK */
		if (frame_pending_for(rx_psdu, rx_len, info)) {
			ack[0] |= FCF_FRAME_PENDING_BIT;
		}
		ack[1] = FCF_ACK_CONTROL_BYTE_1;
		ack[2] = info->seq;

		fcs = crc16(ack, MIN_FRAME_SIZE);
		ack[3] = fcs & BYTE_MASK;
		ack[4] = fcs >> 8;
		ack_len = ACK_FRAME_SIZE;
	}

	if (nsi_otns_bottom_tx_after(data.channel, data.txpower, ack, ack_len,
				     AIFS_TURNAROUND_US) == 0) {
		data.ack_tx_pending = true;
	}
}

static void deliver_rx(const struct otns_radio_event *ev)
{
	struct net_pkt *pkt;
	uint16_t mac_len; /* length delivered to the net stack */

	if (ev->psdu_len < FCS_SIZE) {
		return;
	}

	if (IS_ENABLED(CONFIG_IEEE802154_L2_PKT_INCL_FCS)) {
		mac_len = ev->psdu_len;
	} else {
		mac_len = ev->psdu_len - FCS_SIZE;
	}

	pkt = net_pkt_rx_alloc_with_buffer(data.iface, mac_len,
						   NET_AF_UNSPEC, 0, K_NO_WAIT);
	if (pkt == NULL) {
		LOG_WRN("No RX pkt available, dropping frame");
		return;
	}

	if (net_pkt_write(pkt, ev->psdu, mac_len) < 0) {
		LOG_WRN("Failed to write RX frame");
		net_pkt_unref(pkt);
		return;
	}

	net_pkt_set_ieee802154_lqi(pkt, LQI_PERFECT);
	net_pkt_set_ieee802154_rssi_dbm(pkt, ev->power);

	if (net_recv_data(data.iface, pkt) < 0) {
		LOG_DBG("RX frame dropped by net stack");
		net_pkt_unref(pkt);
	}
}

static void handle_rx(const struct otns_radio_event *ev)
{
	struct frame_addr_info info;
	uint16_t fcf;
	bool for_me;

	if (ev->psdu_len < MIN_FRAME_SIZE) {
		return;
	}

	fcf = sys_get_le16(ev->psdu);

	/* Is this an ACK frame? */
	if ((fcf & FCF_FRAME_TYPE_MASK) == FCF_FRAME_TYPE_ACK) {
		if (data.tx_wants_ack && ev->psdu[2] == data.tx_seq) {
			uint16_t l = ev->psdu_len;

			if (l > sizeof(data.ack_psdu)) {
				l = sizeof(data.ack_psdu);
			}
			memcpy(data.ack_psdu, ev->psdu, l);
			data.ack_len = l;
			k_sem_give(&data.tx_wait);
		}
		return;
	}

	/* Regular frame: deliver up the stack. */
	for_me = frame_is_for_me(ev->psdu, ev->psdu_len, &info);

	deliver_rx(ev);

	/* Auto-acknowledge unicast frames addressed to us that request an ACK. */
	if (for_me && (info.fcf & FCF_ACK_REQ_BIT)) {
		schedule_ack(ev->psdu, ev->psdu_len, ev->power, &info);
	}
}

/* Interrupt handler: drain radio events posted by the runner side. */
static void isr(const void *arg)
{
	struct otns_radio_event ev;

	ARG_UNUSED(arg);

	while (nsi_otns_bottom_get_event(&ev)) {
		switch (ev.type) {
		case OTNS_EVENT_RADIO_RX_DONE:
			handle_rx(&ev);
			break;

		case OTNS_EVENT_RADIO_TX_DONE:
			if (data.ack_tx_pending) {
				/* The deferred auto-ACK (schedule_ack()) has been sent. */
				data.ack_tx_pending = false;
				if (data.sleep_pending) {
					data.sleep_pending = false;
					nsi_otns_bottom_set_state(OTNS_RADIO_STATE_SLEEP,
								   data.channel);
				}
				break;
			}
			data.tx_result = err_to_errno(ev.error);
			if (!data.tx_wants_ack || data.tx_result != 0) {
				k_sem_give(&data.tx_wait);
			}
			break;

		case OTNS_EVENT_RADIO_CHAN_SAMPLE:
			if (data.ed_scan_pending) {
				energy_scan_done_cb_t cb = data.ed_done_cb;

				data.ed_scan_pending = false;
				data.ed_done_cb = NULL;
				if (cb != NULL) {
					cb(radio_dev, ev.power);
				}
				break;
			}
			data.cca_channel_free =
				(ev.power == (int8_t)OTNS_RSSI_INVALID) ||
				(ev.power < RFSIM_CCA_ED_THRESHOLD_DEFAULT_DBM);
			k_sem_give(&data.cca_wait);
			break;

		default:
			break;
		}
	}
}

/* ------------------------------------------------------------------------- */
/* ieee802154_radio_api                                                      */
/* ------------------------------------------------------------------------- */

static enum ieee802154_hw_caps get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	/*
	 * OpenThread requires automatic ACK handling (IEEE802154_HW_TX_RX_ACK).
	 * This driver generates ACKs on RX and matches ACKs on TX itself.
	 * IEEE802154_HW_ENERGY_SCAN backs both otPlatRadioEnergyScan() and
	 * otPlatRadioGetRssi() (see energy_scan() below).
	 */
	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER | IEEE802154_HW_ENERGY_SCAN |
	       IEEE802154_HW_TX_RX_ACK | IEEE802154_HW_RX_TX_ACK;
}

static int energy_scan(const struct device *dev, uint16_t duration, energy_scan_done_cb_t done_cb)
{
	struct ctx *ctx = dev->data;

	/*
	 * The simulator has no notion of an integrated energy scan; a single
	 * channel sample (the same primitive used for CCA) is used as the scan
	 * result, so the requested duration is not otherwise observed.
	 */
	ARG_UNUSED(duration);

	if (!ctx->started || !nsi_otns_bottom_is_connected()) {
		return -EIO;
	}
	if (ctx->ed_scan_pending) {
		return -EBUSY;
	}

	ctx->ed_scan_pending = true;
	ctx->ed_done_cb = done_cb;

	if (nsi_otns_bottom_cca(ctx->channel) < 0) {
		ctx->ed_scan_pending = false;
		ctx->ed_done_cb = NULL;
		return -EIO;
	}

	return 0;
}

static int cca(const struct device *dev)
{
	struct ctx *ctx = dev->data;

	if (!ctx->started) {
		return -EIO;
	}
	if (!nsi_otns_bottom_is_connected()) {
		return -EIO;
	}

	k_sem_reset(&ctx->cca_wait);
	ctx->cca_channel_free = true;

	if (nsi_otns_bottom_cca(ctx->channel) < 0) {
		return -EIO;
	}

	if (k_sem_take(&ctx->cca_wait, K_MSEC(CCA_TIMEOUT_MS)) != 0) {
		/* No answer from the simulator; assume channel is clear. */
		return 0;
	}

	return ctx->cca_channel_free ? 0 : -EBUSY;
}

static int set_channel(const struct device *dev, uint16_t channel)
{
	struct ctx *ctx = dev->data;

	if (channel < kMinChannel || channel > kMaxChannel) {
		return channel < kMinChannel ? -ENOTSUP : -EINVAL;
	}

	ctx->channel = (uint8_t)channel;

	if (ctx->started) {
		nsi_otns_bottom_set_state(OTNS_RADIO_STATE_RECEIVE, ctx->channel);
	}

	return 0;
}

static int filter(const struct device *dev, bool set,
	       enum ieee802154_filter_type type,
	       const struct ieee802154_filter *filter)
{
	struct ctx *ctx = dev->data;

	if (!set) {
		return -ENOTSUP;
	}

	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR: {
		uint8_t ext_addr_be[EXT_ADDR_SIZE];

		memcpy(ctx->ext_addr, filter->ieee_addr, EXT_ADDR_SIZE);

		/*
		 * Report the extended address to OTNS so it can route unicast
		 * frames addressed by extended address to this node (e.g. the
		 * MLE Parent Response); without it such frames are never
		 * delivered and the link fails with repeated NoAck / endless
		 * Parent Requests.
		 *
		 * OpenThread programs the address here in on-air (little-endian)
		 * byte order (the same order it appears in a frame's address
		 * field, and the order used for ACK matching). OTNS, however,
		 * keys its node lookup on the big-endian value, so the bytes are
		 * reversed before being sent in the EXT_ADDR event.
		 */
		for (int i = 0; i < EXT_ADDR_SIZE; i++) {
			ext_addr_be[i] = ctx->ext_addr[EXT_ADDR_SIZE - 1 - i];
		}
		nsi_otns_bottom_send_ext_addr(ext_addr_be);
		return 0;
	}
	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		sys_put_le16(filter->short_addr, ctx->short_addr);
		return 0;
	case IEEE802154_FILTER_TYPE_PAN_ID:
		sys_put_le16(filter->pan_id, ctx->pan_id);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int set_txpower(const struct device *dev, int16_t dbm)
{
	struct ctx *ctx = dev->data;

	ctx->txpower = (int8_t)dbm;
	return 0;
}

static int tx(const struct device *dev, enum ieee802154_tx_mode mode,
		   struct net_pkt *pkt, struct net_buf *frag)
{
	struct ctx *ctx = dev->data;
	uint8_t psdu[OTNS_PSDU_MAX];
	uint16_t len = frag->len;
	uint16_t fcs;
	int ret;
	int rc = 0;

	ARG_UNUSED(pkt);

	if (mode != IEEE802154_TX_MODE_DIRECT && mode != IEEE802154_TX_MODE_CCA) {
		LOG_ERR("TX mode %d not supported", mode);
		return -ENOTSUP;
	}

	if (!ctx->started || !nsi_otns_bottom_is_connected()) {
		return -EIO;
	}

	if (len + FCS_SIZE > OTNS_PSDU_MAX) {
		return -EMSGSIZE;
	}

	/* Optional CCA before transmission. */
	if (mode == IEEE802154_TX_MODE_CCA) {
		ret = cca(dev);
		if (ret != 0) {
			return ret;
		}
	}

	/* Compose the PSDU: MAC frame + FCS. */
	memcpy(psdu, frag->data, len);
	fcs = crc16(psdu, len);
	psdu[len] = fcs & 0xff;
	psdu[len + 1] = fcs >> 8;

	ctx->tx_wants_ack = (len >= 1) && (frag->data[0] & (FCF_ACK_REQ_BIT & BYTE_MASK));
	ctx->tx_seq = (len >= MIN_FRAME_SIZE) ? frag->data[2] : 0;
	ctx->ack_len = 0;
	ctx->tx_result = 0;

	k_sem_reset(&ctx->tx_wait);

	/* Radio moves to TRANSMIT for the duration of the frame. */
	nsi_otns_bottom_set_state(OTNS_RADIO_STATE_TRANSMIT, ctx->channel);

	ret = nsi_otns_bottom_tx(ctx->channel, ctx->txpower, psdu, len + FCS_SIZE);
	if (ret < 0) {
		rc = -EIO;
		goto out;
	}

	/*
	 * The frame is on air (OTNS models it from the RADIO_COMM_START event).
	 * Return the radio to RECEIVE now so OTNS will deliver the incoming ACK:
	 * it only dispatches frames to nodes whose radio energy state is
	 * "receive". This mirrors the reference radio moving to its
	 * TX_ACK_RX_ONGOING substate (energy state = receive) right after
	 * transmitting an ACK-requesting frame.
	 */
	nsi_otns_bottom_set_state(OTNS_RADIO_STATE_RECEIVE, ctx->channel);

	if (ctx->tx_wants_ack) {
		uint32_t frame_us = (OT_RADIO_SHR_PHR_LENGTH_BYTES + len + FCS_SIZE) *
				    (OT_RADIO_SYMBOLS_PER_OCTET * OT_RADIO_SYMBOL_TIME);

		if (k_sem_take(&ctx->tx_wait,
			       K_USEC(frame_us + ACK_ALLOWANCE_US)) != 0) {
			/* TX done but no ACK arrived in time. */
			rc = -ENOMSG;
			goto out;
		}
	} else {
		(void)k_sem_take(&ctx->tx_wait, K_FOREVER);
	}

	if (ctx->tx_result != 0) {
		rc = ctx->tx_result;
		goto out;
	}

	if (ctx->tx_wants_ack) {
		if (ctx->ack_len == 0) {
			rc = -ENOMSG;
			goto out;
		}

		/* Hand the received ACK to the L2 (OpenThread stores it). */
		struct net_pkt *ack_pkt;
		uint16_t ack_mac_len =
			IS_ENABLED(CONFIG_IEEE802154_L2_PKT_INCL_FCS)
				? ctx->ack_len
				: ctx->ack_len - FCS_SIZE;

		ack_pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, ack_mac_len,
						       NET_AF_UNSPEC, 0,
						       K_NO_WAIT);
		if (ack_pkt != NULL) {
			if (net_pkt_write(ack_pkt, ctx->ack_psdu, ack_mac_len) == 0) {
				net_pkt_set_ieee802154_lqi(ack_pkt, LQI_PERFECT);
				net_pkt_set_ieee802154_rssi_dbm(ack_pkt, RSSI_ZERO_DBM);
				net_pkt_cursor_init(ack_pkt);
				if (ieee802154_handle_ack(ctx->iface, ack_pkt) != NET_OK) {
					LOG_DBG("ACK not handled");
				}
			}
			net_pkt_unref(ack_pkt);
		}
	}

out:
	/* Return to RECEIVE once the transmission (and ACK wait) is done. */
	nsi_otns_bottom_set_state(OTNS_RADIO_STATE_RECEIVE, ctx->channel);
	return rc;
}

static int start(const struct device *dev)
{
	struct ctx *ctx = dev->data;

	if (ctx->started) {
		return -EALREADY;
	}

	ctx->started = true;
	ctx->sleep_pending = false;
	nsi_otns_bottom_set_state(OTNS_RADIO_STATE_RECEIVE, ctx->channel);

	return 0;
}

static int stop(const struct device *dev)
{
	struct ctx *ctx = dev->data;

	if (!ctx->started) {
		return -EALREADY;
	}

	ctx->started = false;

	/*
	 * If a deferred auto-ACK (schedule_ack()) hasn't transmitted yet, don't
	 * tell the simulator we're asleep now: doing so could make OTNS drop the
	 * still-pending ACK. Report SLEEP once it completes (see isr()) instead.
	 * Mirrors ot-rfsim's sDelaySleep/applyRadioDelayedSleep().
	 */
	if (ctx->ack_tx_pending) {
		ctx->sleep_pending = true;
	} else {
		nsi_otns_bottom_set_state(OTNS_RADIO_STATE_SLEEP, ctx->channel);
	}

	return 0;
}

/*
 * Stores (or removes) an Enhanced-ACK header IE template, per
 * IEEE802154_CONFIG_ENH_ACK_HEADER_IE. header_ie is treated as a raw pointer
 * to [2-byte header][content bytes] rather than through its typed union,
 * matching how Zephyr's own generic OT radio layer constructs it for vendor
 * IEs (see set_vendor_ie_header_lm() in modules/openthread/platform/radio.c).
 */
static int configure_enh_ack_ie(const struct ieee802154_config *config)
{
	const uint8_t *raw = (const uint8_t *)config->ack_ie.header_ie;
	uint16_t short_addr = config->ack_ie.short_addr;
	const uint8_t *ext_addr_be = config->ack_ie.ext_addr;
	bool has_short_filter = short_addr != BROADCAST_SHORT_ADDR;
	bool has_ext_filter = ext_addr_be != NULL;
	bool remove_all_for_filter = config->ack_ie.purge_ie || raw == NULL;
	uint8_t element_id = 0;
	uint8_t content_len = 0;
	int slot = -1;
	int free_slot = -1;

	if (!remove_all_for_filter) {
		uint16_t hdr = sys_get_le16(raw);

		content_len = hdr & HEADER_IE_LEN_MASK;
		element_id = (hdr >> HEADER_IE_ID_SHIFT) & BYTE_MASK;
	}

	for (int i = 0; i < MAX_ACK_IES; i++) {
		struct enh_ack_ie *ie = &data.ack_ies[i];
		bool same_filter;

		if (!ie->valid) {
			if (free_slot < 0) {
				free_slot = i;
			}
			continue;
		}

		same_filter = ie->has_short_filter == has_short_filter &&
			      (!has_short_filter || ie->short_addr == short_addr) &&
			      ie->has_ext_filter == has_ext_filter &&
			      (!has_ext_filter ||
			       memcmp(ie->ext_addr_be, ext_addr_be, EXT_ADDR_SIZE) == 0);

		if (!same_filter) {
			continue;
		}
		if (remove_all_for_filter) {
			ie->valid = false;
		} else if (ie->element_id == element_id) {
			slot = i;
		}
	}

	if (remove_all_for_filter || content_len == 0) {
		if (slot >= 0) {
			data.ack_ies[slot].valid = false;
		}
		return 0;
	}

	if (content_len > ACK_IE_MAX_CONTENT) {
		return -EINVAL;
	}
	if (slot < 0) {
		slot = free_slot;
	}
	if (slot < 0) {
		return -ENOMEM;
	}

	struct enh_ack_ie *ie = &data.ack_ies[slot];

	ie->valid = true;
	ie->element_id = element_id;
	ie->content_len = content_len;
	memcpy(ie->content, raw + 2, content_len);
	ie->has_short_filter = has_short_filter;
	ie->short_addr = short_addr;
	ie->has_ext_filter = has_ext_filter;
	if (has_ext_filter) {
		memcpy(ie->ext_addr_be, ext_addr_be, EXT_ADDR_SIZE);
	}
	return 0;
}

static int configure(const struct device *dev,
			  enum ieee802154_config_type type,
			  const struct ieee802154_config *config)
{
	ARG_UNUSED(dev);

	switch (type) {
	case IEEE802154_CONFIG_AUTO_ACK_FPB:
		data.auto_ack_fpb_enabled = config->auto_ack_fpb.enabled;
		return 0;

	case IEEE802154_CONFIG_ACK_FPB:
		if (config->ack_fpb.addr == NULL) {
			/* No address: clear every entry of the given address type. */
			if (config->ack_fpb.extended) {
				data.fpb_ext_count = 0;
			} else {
				data.fpb_short_count = 0;
			}
			return 0;
		}
		if (config->ack_fpb.extended) {
			return config->ack_fpb.enabled ? fpb_ext_add(config->ack_fpb.addr)
							: fpb_ext_remove(config->ack_fpb.addr);
		}
		return config->ack_fpb.enabled
				? fpb_short_add(sys_get_le16(config->ack_fpb.addr))
				: fpb_short_remove(sys_get_le16(config->ack_fpb.addr));

	case IEEE802154_CONFIG_CSL_PERIOD:
		data.csl_period = config->csl_period;
		return 0;

	case IEEE802154_CONFIG_EXPECTED_RX_TIME:
		data.csl_expected_rx_time_ns = config->expected_rx_time;
		return 0;

	case IEEE802154_CONFIG_ENH_ACK_HEADER_IE:
		return configure_enh_ack_ie(config);

	default:
		/* No hardware to configure in the simulator; accept everything else. */
		return 0;
	}
}

IEEE802154_DEFINE_PHY_SUPPORTED_CHANNELS(drv_attr, kMinChannel, kMaxChannel);

static int attr_get(const struct device *dev, enum ieee802154_attr attr,
			 struct ieee802154_attr_value *value)
{
	ARG_UNUSED(dev);

	return ieee802154_attr_get_channel_page_and_range(
		attr, IEEE802154_ATTR_PHY_CHANNEL_PAGE_ZERO_OQPSK_2450_BPSK_868_915,
		&drv_attr.phy_supported_channels, value);
}

static void get_mac(struct ctx *ctx)
{
	int node_id = nsi_otns_bottom_get_node_id();

	if (node_id <= 0) {
		node_id = DEFAULT_NODE_ID;
	}

	/* Match the ot-rfsim EUI-64 layout so OTNS identifies the node. */
	ctx->mac_addr[0] = EUI64_BYTE_0;
	ctx->mac_addr[1] = EUI64_BYTE_1;
	ctx->mac_addr[2] = EUI64_BYTE_2;
	ctx->mac_addr[3] = EUI64_BYTE_3;
	ctx->mac_addr[4] = (node_id >> BIT_SHIFT_24) & BYTE_MASK;
	ctx->mac_addr[5] = (node_id >> BIT_SHIFT_16) & BYTE_MASK;
	ctx->mac_addr[6] = (node_id >> BIT_SHIFT_8) & BYTE_MASK;
	ctx->mac_addr[7] = node_id & BYTE_MASK;
}

static void iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct ctx *ctx = dev->data;

	get_mac(ctx);
	memcpy(ctx->ext_addr, ctx->mac_addr, EXT_ADDR_SIZE);

	net_if_set_link_addr(iface, ctx->mac_addr, EXT_ADDR_SIZE, NET_LINK_IEEE802154);

	ctx->iface = iface;
	radio_dev = dev;

	ieee802154_init(iface);
}

static int init(const struct device *dev)
{
	struct ctx *ctx = dev->data;

	k_sem_init(&ctx->tx_wait, 0, 1);
	k_sem_init(&ctx->cca_wait, 0, 1);

	ctx->channel = kMinChannel;
	ctx->txpower = RSSI_ZERO_DBM;
	ctx->started = false;

	IRQ_CONNECT(IEEE802154_OTNS_IRQ, 0, isr, NULL, 0);
	irq_enable(IEEE802154_OTNS_IRQ);

	LOG_INF("OTNS IEEE 802.15.4 driver initialized (node id %d)",
		nsi_otns_bottom_get_node_id());

	return 0;
}

/* otPlatRadioGetCslAccuracy() calls this unconditionally; no capability gate exists for it. */
static uint8_t get_sch_acc(const struct device *dev)
{
	ARG_UNUSED(dev);

	return RFSIM_CSL_ACCURACY_DEFAULT_PPM;
}

static const struct ieee802154_radio_api radio_api = {
	.iface_api.init = iface_init,

	.get_capabilities = get_capabilities,
	.cca = cca,
	.set_channel = set_channel,
	.filter = filter,
	.set_txpower = set_txpower,
	.tx = tx,
	.start = start,
	.stop = stop,
	.configure = configure,
	.attr_get = attr_get,
	.ed_scan = energy_scan,
	.get_sch_acc = get_sch_acc,
};

#if defined(CONFIG_NET_L2_IEEE802154)
#define RADIO_L2 IEEE802154_L2
#define RADIO_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define RADIO_MTU IEEE802154_MTU
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define RADIO_L2 OPENTHREAD_L2
#define RADIO_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define RADIO_MTU OPENTHREAD_MTU
#elif defined(CONFIG_NET_L2_CUSTOM_IEEE802154)
#define RADIO_L2 CUSTOM_IEEE802154_L2
#define RADIO_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(CUSTOM_IEEE802154_L2)
#define RADIO_MTU CONFIG_NET_L2_CUSTOM_IEEE802154_MTU
#endif

#if defined(CONFIG_NET_L2_PHY_IEEE802154)
NET_DEVICE_DT_INST_DEFINE(0, init, NULL, &data, NULL,
			  CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &radio_api,
			  RADIO_L2, RADIO_L2_CTX_TYPE, RADIO_MTU);
#endif
