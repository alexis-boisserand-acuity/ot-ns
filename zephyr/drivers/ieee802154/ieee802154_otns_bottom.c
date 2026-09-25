/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Runner ("bottom") side of the OTNS virtual IEEE 802.15.4 radio driver.
 *
 * This file is compiled as part of the native simulator runner (host code). It
 * owns the real Unix domain socket connection to OTNS, implements the OTNS
 * simulation-event wire protocol, and — crucially — synchronizes the native
 * simulator virtual clock with the OTNS virtual time using the native simulator
 * hardware-event scheduler (a "pacer" NSI_HW_EVENT).
 *
 * Lock-step principle (see ot-rfsim/src/system.c for the reference OT node):
 *   - Whenever the Zephyr node is idle, we report to OTNS how long we intend to
 *     sleep (an ALARM_FIRED event whose delay is the time until the next Zephyr
 *     kernel event). OTNS then advances global virtual time and sends us the
 *     next event with a delay telling us by how much time advanced. We advance
 *     the native simulator clock accordingly and, for radio events, hand the
 *     data to the embedded driver through IEEE802154_OTNS_IRQ.
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/un.h>

#include "nsi_cmdline.h"
#include "nsi_hw_scheduler.h"
#include "nsi_hws_models_if.h"
#include "nsi_tasks.h"
#include "nsi_tracing.h"
#include "irq_ctrl.h"

#include "ieee802154_otns_priv.h"

/* IEEE 802.15.4 timing used to compute frame durations. */
#define PHY_BITRATE       250000U /* bit/s (O-QPSK 2.4 GHz) */
#define SHR_PHR_OCTETS    6U       /* preamble + SFD + PHR */
#define CCA_DURATION_US   128U     /* 8 symbols * 16 us */

/* OTNS radio-message channel byte + PSDU representation used on the wire. */
#define RADIO_MSG_HDR 1U

#define EVENT_HDR_LEN 19U /* u64 delay + u8 event + u64 msgid + u16 datalen */

#define OT_EVENT_DATA_MAX 2048

/* RadioCommEventData structure size (channel + power + error + duration) */
#define RADIO_COMM_EVENT_DATA_LEN 11U
/* RadioStateData structure size */
#define RADIO_STATE_DATA_LEN 14U
/* Extended (MAC) address length in bytes */
#define EXTENDED_ADDR_LEN 8U
/* Default IEEE 802.15.4 radio channel */
#define DEFAULT_RADIO_CHANNEL 11U
/* Marker for invalid/uninitialized state */
#define INVALID_STATE_MARKER 0xffU
/* RX sensitivity in dBm */
#define RX_SENSITIVITY_DBM (-100)
/* Maximum sleep interval reported to OTNS when the node has no pending kernel event. */
#define MAX_SLEEP_US 3600000000ULL

struct raw_event {
	uint64_t delay;
	uint8_t  event;
	uint64_t msg_id;
	uint16_t datalen;
	uint8_t  data[OT_EVENT_DATA_MAX];
};

/* ------------------------------------------------------------------------- */
/* State                                                                     */
/* ------------------------------------------------------------------------- */

static uint32_t cmd_node_id;      /* --otns-node-id */
static const char *cmd_socket;    /* --otns-socket  */
static int32_t cmd_seed;          /* --otns-seed    */

static int fd = -1;
static bool connected;
static bool disabled;         /* set once when no socket configured/failed */
static uint64_t last_msg_id; /* last received msg id (echoed on sends) */

/* Pacer NSI_HW_EVENT timer (absolute simulated time in us). */
static uint64_t pacer_time;

/* Event pending delivery to the embedded side at pacer_time. */
static struct otns_radio_event pending_ev;
static bool have_pending;

/* Event currently offered to the embedded ISR through nsi_otns_bottom_get_event(). */
static struct otns_radio_event deliver_ev;
static bool deliver_ready;

/*
 * Deferred auto-ACK transmission, timed on the microsecond-resolution native
 * simulator HW clock. The Zephyr kernel tick is too coarse to represent the
 * 192 us AIFS turnaround, so the ACK is scheduled here as its own HW event
 * instead of with a k_timer on the embedded side. ack_time is the absolute HW
 * time (us) at which to emit the frame, or NSI_NEVER when idle.
 */
static uint64_t ack_time = NSI_NEVER;
static uint8_t  ack_channel;
static int8_t   ack_power;
static uint8_t  ack_psdu[OTNS_PSDU_MAX];
static uint16_t ack_psdu_len;

/*
 * Cached radio state, updated by the embedded driver and reported to OTNS as a
 * RADIO_STATE event before every sleep (with change detection), mirroring the
 * reference node's platformRadioReportStateToSimulator(false) call in
 * otSysProcessDrivers(). OTNS keeps a node at RadioDisabled — and delivers it
 * no frames — until it receives a RADIO_STATE event, and it must see the
 * RX/TX/sleep transitions to model reception correctly.
 */
static uint8_t radio_state = OTNS_RADIO_STATE_DISABLED;
static uint8_t radio_channel = DEFAULT_RADIO_CHANNEL;
static uint8_t reported_state = INVALID_STATE_MARKER;   /* force the first report */
static uint8_t reported_channel = INVALID_STATE_MARKER;

/*
 * Cached extended (MAC) address in on-air (little-endian) order. OpenThread
 * sets it once at boot — before the OTNS socket is connected — so it must be
 * stored here and (re)sent to OTNS as an EXT_ADDR event upon connection, so
 * OTNS can route unicast frames addressed by extended address to this node.
 */
static uint8_t ext_addr[EXTENDED_ADDR_LEN];
static bool ext_addr_valid;

/* ------------------------------------------------------------------------- */
/* Little-endian (de)serialization helpers                                   */
/* ------------------------------------------------------------------------- */

static inline void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
}

static inline void put_le64(uint8_t *p, uint64_t v)
{
	for (int i = 0; i < 8; i++) {
		p[i] = (v >> (8 * i)) & 0xff;
	}
}

static inline uint16_t get_le16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint64_t get_le64(const uint8_t *p)
{
	uint64_t v = 0;

	for (int i = 0; i < 8; i++) {
		v |= (uint64_t)p[i] << (8 * i);
	}
	return v;
}

/* ------------------------------------------------------------------------- */
/* Socket I/O                                                                */
/* ------------------------------------------------------------------------- */

static int read_full(void *buf, size_t n)
{
	size_t got = 0;

	while (got < n) {
		ssize_t r = read(fd, (char *)buf + got, n - got);

		if (r > 0) {
			got += (size_t)r;
			continue;
		}
		if (r == 0) {
			fprintf(stderr, "[otns] socket closed by OTNS (EOF) after %zu/%zu bytes\n",
				got, n);
			return -1;
		}
		if (errno == EINTR) {
			continue;
		}
		fprintf(stderr, "[otns] socket read error: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

static int read_event(struct raw_event *ev)
{
	uint8_t hdr[EVENT_HDR_LEN];

	if (read_full(hdr, EVENT_HDR_LEN) < 0) {
		return -1;
	}

	ev->delay = get_le64(hdr + 0);              /* hdr[0:8] */
	ev->event = hdr[8];                          /* hdr[8] */
	ev->msg_id = get_le64(hdr + 9);              /* hdr[9:17] */
	ev->datalen = get_le16(hdr + 17);            /* hdr[17:19] */

	if (ev->datalen > 0) {
		if (ev->datalen > sizeof(ev->data)) {
			fprintf(stderr, "[otns] event datalen %u too large (ev=%u)\n",
				ev->datalen, ev->event);
			return -1;
		}
		if (read_full(ev->data, ev->datalen) < 0) {
			return -1;
		}
	}
	return 0;
}

static int write_event(uint8_t type, uint64_t delay,
			    const uint8_t *data, uint16_t datalen)
{
	uint8_t buf[EVENT_HDR_LEN + OT_EVENT_DATA_MAX];
	size_t total;
	size_t sent = 0;

	if (fd < 0) {
		return -1;
	}
	if (datalen > OT_EVENT_DATA_MAX) {
		return -1;
	}

	put_le64(buf + 0, delay);       /* buf[0:8] */
	buf[8] = type;                      /* buf[8] */
	put_le64(buf + 9, last_msg_id); /* buf[9:17] */
	put_le16(buf + 17, datalen);    /* buf[17:19] */
	if (datalen > 0) {
		memcpy(buf + EVENT_HDR_LEN, data, datalen);
	}

	total = EVENT_HDR_LEN + datalen;
	while (sent < total) {
		ssize_t w = write(fd, buf + sent, total - sent);

		if (w > 0) {
			sent += (size_t)w;
			continue;
		}
		if (w < 0 && errno == EINTR) {
			continue;
		}
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Connection / handshake                                                    */
/* ------------------------------------------------------------------------- */

static void disconnect(void)
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
	connected = false;
	disabled = true;
	pacer_time = NSI_NEVER;
	nsi_print_warning("ieee802154_otns: disconnected from OTNS\n");
}

static int try_connect(void)
{
	struct sockaddr_un addr;
	size_t path_len;

	if (cmd_socket == NULL || cmd_socket[0] == '\0') {
		return -1;
	}

	path_len = strlen(cmd_socket);
	if (path_len >= sizeof(addr.sun_path)) {
		nsi_print_warning("ieee802154_otns: socket path too long\n");
		return -1;
	}

	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		nsi_print_warning("ieee802154_otns: socket() failed\n");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	memcpy(addr.sun_path, cmd_socket, path_len);

	if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		nsi_print_warning("ieee802154_otns: cannot connect to OTNS at %s\n",
				  cmd_socket);
		close(fd);
		fd = -1;
		return -1;
	}

	connected = true;

	/* Handshake: identify ourselves to OTNS. */
	{
		uint8_t node[4];

		node[0] = cmd_node_id & 0xff;
		node[1] = (cmd_node_id >> 8) & 0xff;
		node[2] = (cmd_node_id >> 16) & 0xff;
		node[3] = (cmd_node_id >> 24) & 0xff;
		(void)write_event(OTNS_EVENT_NODE_INFO, 0, node, sizeof(node));
	}

	/*
	 * OpenThread sets the extended address during early boot, before this
	 * connection was established, so replay the cached value now. Without it
	 * OTNS cannot route unicast frames addressed by extended address to this
	 * node (e.g. the MLE Parent Response), and the link fails with NoAck.
	 */
	if (ext_addr_valid) {
		(void)write_event(OTNS_EVENT_EXT_ADDR, 0, ext_addr,
				       EXTENDED_ADDR_LEN);
	}

	nsi_print_trace("ieee802154_otns: connected to OTNS (node %u) at %s\n",
			cmd_node_id, cmd_socket);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Pacer (virtual-time lock-step)                                            */
/* ------------------------------------------------------------------------- */

/* Compute the next scheduled event time of all HW models except our pacer. */
static uint64_t next_other_event(void)
{
	uint64_t saved = pacer_time;
	uint64_t next;

	pacer_time = NSI_NEVER;
	nsi_hws_find_next_event();
	next = nsi_hws_get_next_event_time();
	pacer_time = saved;

	return next;
}

/* Store a radio event for the embedded ISR and run it synchronously. */
static void deliver_to_embedded(const struct otns_radio_event *ev)
{
	deliver_ev = *ev;
	deliver_ready = true;

	/* Run the embedded ISR now (HW thread -> CPU, returns when CPU idles). */
	hw_irq_ctrl_raise_im(IEEE802154_OTNS_IRQ);
}

/* Parse a RadioCommEventData(+frame) payload into a radio event. */
static void parse_radio_event(struct otns_radio_event *out,
				   const struct raw_event *raw)
{
	out->type = raw->event;
	out->channel = raw->data[0];          /* RadioCommEventData[0] */
	out->power = (int8_t)raw->data[1];    /* RadioCommEventData[1] */
	out->error = raw->data[2];            /* RadioCommEventData[2] */
	out->psdu_len = 0;

	if (raw->event == OTNS_EVENT_RADIO_RX_DONE ||
	    raw->event == OTNS_EVENT_RADIO_COMM_START) {
		/* Payload: RadioCommEventData + channel byte + PSDU. */
		if (raw->datalen > RADIO_COMM_EVENT_DATA_LEN + RADIO_MSG_HDR) {
			uint16_t len = raw->datalen - RADIO_COMM_EVENT_DATA_LEN - RADIO_MSG_HDR;

			if (len > sizeof(out->psdu)) {
				len = sizeof(out->psdu);
			}
			memcpy(out->psdu, &raw->data[RADIO_COMM_EVENT_DATA_LEN + RADIO_MSG_HDR], len);
			out->psdu_len = len;
		}
	}
}

static int event_is_for_embedded(uint8_t type)
{
	return type == OTNS_EVENT_RADIO_RX_DONE ||
	       type == OTNS_EVENT_RADIO_TX_DONE ||
	       type == OTNS_EVENT_RADIO_CHAN_SAMPLE;
}

/*
 * Deliver CLI command bytes (OT_SIM_EVENT_UART_WRITE) to the OTNS CLI bridge.
 * Weak no-op default so the driver links even when the CLI bridge
 * (CONFIG_IEEE802154_OTNS_CLI) is not compiled in; the real implementation is
 * provided by cli/otns_cli_bottom.c.
 */
__attribute__((weak)) void nsi_otns_cli_feed_input(const uint8_t *buf, int len)
{
	(void)buf;
	(void)len;
}

/*
 * Report the current radio state to OTNS, but only if it changed since the last
 * report. Sent right before each sleep event, matching the reference node's
 * platformRadioReportStateToSimulator(false).
 */
static void report_state(void)
{
	uint8_t data[RADIO_STATE_DATA_LEN];

	if (radio_state == reported_state &&
	    radio_channel == reported_channel) {
		return;
	}

	/*
	 * Never advertise the DISABLED energy state to OTNS. DISABLED is only the
	 * driver's initial value, held until OpenThread first starts the radio.
	 * Reporting it emits a RADIO_STATE transition at the same virtual time
	 * (RadioTime=0) as the very first RECEIVE transition. OTNS cannot order
	 * two state transitions carrying an identical timestamp deterministically,
	 * so on some nodes the DISABLED state is applied last and the radio is
	 * left "off" in OTNS's medium model. Such a node is never delivered any
	 * frame, so it never hears its neighbours' MLE advertisements: it stays a
	 * lone leader and endlessly re-runs the router Link Request/Accept
	 * handshake (the repeating "Router table added" churn). Suppressing the
	 * DISABLED report makes the node's first reported state its real RECEIVE
	 * (or SLEEP) state, which OTNS applies unambiguously.
	 */
	if (radio_state == OTNS_RADIO_STATE_DISABLED) {
		return;
	}

	data[0] = radio_channel;            /* mChannel */
	data[1] = 0;                        /* mTxPower */
	data[2] = (uint8_t)RX_SENSITIVITY_DBM; /* mRxSensitivity */
	data[3] = radio_state;              /* mEnergyState */
	data[4] = 0;                        /* mSubState */
	data[5] = radio_state;              /* mState */
	put_le64(&data[6], nsi_hws_get_time()); /* mRadioTime */

	(void)write_event(OTNS_EVENT_RADIO_STATE, 0, data, sizeof(data));

	reported_state = radio_state;
	reported_channel = radio_channel;
}

static void pacer(void)
{
	uint64_t now;

	if (disabled) {
		pacer_time = NSI_NEVER;
		return;
	}

	if (!connected) {
		if (try_connect() < 0) {
			disabled = true;
			pacer_time = NSI_NEVER;
			return;
		}
	}

	now = nsi_hws_get_time();

	/* Deliver a previously scheduled radio event at its due time. */
	if (have_pending) {
		have_pending = false;
		deliver_to_embedded(&pending_ev);
	}

	/* Lock-step exchange with OTNS. */
	for (;;) {
		struct raw_event raw;
		uint64_t next;
		uint64_t delay;

		next = next_other_event();
#ifdef OTNS_TRACE
		fprintf(stderr, "[otns] pacer now=%llu next=%llu\n",
			(unsigned long long)now, (unsigned long long)next);
#endif

		/*
		 * If another HW event (e.g. the kernel tick) is due at or before
		 * the current instant, the node is still busy: let it run before
		 * involving OTNS. Virtual time does not advance until the node is
		 * idle, matching the reference OT node behaviour (only report a
		 * sleep event to OTNS when there is nothing left to do "now").
		 */
		if (next != NSI_NEVER && next <= now) {
			pacer_time = now;
			return;
		}

		delay = (next == NSI_NEVER) ? MAX_SLEEP_US
					    : (next - now);

		/* Report radio state (if changed) before going to sleep. */
		report_state();

		if (write_event(OTNS_EVENT_ALARM_FIRED, delay, NULL, 0) < 0) {
			fprintf(stderr, "[otns] write ALARM_FIRED failed\n");
			disconnect();
			return;
		}

		if (read_event(&raw) < 0) {
			fprintf(stderr, "[otns] read event failed\n");
			disconnect();
			return;
		}
		last_msg_id = raw.msg_id;
#ifdef OTNS_TRACE
		fprintf(stderr, "[otns] recv ev=%u delay=%llu\n", raw.event,
			(unsigned long long)raw.delay);
#endif

		if (!event_is_for_embedded(raw.event)) {
			/*
			 * ALARM_FIRED, COMM_START (rx start), UART_WRITE, etc.
			 * UART_WRITE carries an OpenThread CLI command from OTNS;
			 * hand it to the CLI bridge. All of these only advance time
			 * on the radio side.
			 */
			if (raw.event == OTNS_EVENT_UART_WRITE) {
				nsi_otns_cli_feed_input(raw.data, raw.datalen);
			}
			pacer_time = now + raw.delay;
			return;
		}

		/* Radio event to hand to the embedded driver. */
		if (raw.delay == 0) {
			struct otns_radio_event ev;

			parse_radio_event(&ev, &raw);
			deliver_to_embedded(&ev);
			/* Re-evaluate: node state may have changed. */
			continue;
		}

		parse_radio_event(&pending_ev, &raw);
		have_pending = true;
		pacer_time = now + raw.delay;
		return;
	}
}

/*
 * The pacer must run *after* all other HW models at any given timestamp, so
 * that when it executes the CPU has already been serviced (kernel tick and IRQ
 * controller, prio 0 and 900) and is idle. It therefore uses a high priority
 * number (runs last).
 */
NSI_HW_EVENT(pacer_time, pacer, 950);

/* ------------------------------------------------------------------------- */
/* Command line options                                                      */
/* ------------------------------------------------------------------------- */

static void register_cmdline_opts(void)
{
	static struct args_struct_t cmdline_options[] = {
		{
			.option = "otns-node-id",
			.name = "id",
			.type = 'u',
			.dest = (void *)&cmd_node_id,
			.descript = "OTNS node id assigned to this node (>= 1)",
		},
		{
			.option = "otns-socket",
			.name = "path",
			.type = 's',
			.dest = (void *)&cmd_socket,
			.descript = "Path of the OTNS dispatcher Unix domain socket",
		},
		{
			.option = "otns-seed",
			.name = "seed",
			.type = 'i',
			.dest = (void *)&cmd_seed,
			.descript = "Optional OTNS random seed",
		},
		ARG_TABLE_ENDMARKER,
	};

	nsi_add_command_line_opts(cmdline_options);
}

NSI_TASK(register_cmdline_opts, PRE_BOOT_1, 200);

/* Arm the pacer to run at time 0 so the connection is attempted early. */
static void boot(void)
{
	if (cmd_socket != NULL && cmd_socket[0] != '\0') {
		pacer_time = 0;
	} else {
		disabled = true;
		pacer_time = NSI_NEVER;
	}
}

NSI_TASK(boot, HW_INIT, 500);

/* Close the socket on exit. */
static void cleanup(void)
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
}

NSI_TASK(cleanup, ON_EXIT_PRE, 100);

/* ------------------------------------------------------------------------- */
/* Boundary functions called from the embedded side                         */
/* ------------------------------------------------------------------------- */

int nsi_otns_bottom_get_node_id(void)
{
	return (int)cmd_node_id;
}

bool nsi_otns_bottom_is_connected(void)
{
	return connected;
}

bool nsi_otns_bottom_is_configured(void)
{
	return cmd_socket != NULL && cmd_socket[0] != '\0';
}

/* Wake the pacer at the current time so it forwards freshly-queued events. */
static void wake_pacer_now(void)
{
	if (disabled || !connected) {
		return;
	}
	if (have_pending) {
		/* A radio event is already scheduled; keep its timing. */
		return;
	}
	pacer_time = nsi_hws_get_time();
	nsi_hws_find_next_event();
}

int nsi_otns_bottom_tx(uint8_t channel, int8_t power,
		       const uint8_t *psdu, uint16_t len)
{
	uint8_t data[11 + RADIO_MSG_HDR + OTNS_PSDU_MAX];
	uint64_t duration;

	if (!connected) {
		return -1;
	}
	if (len > OTNS_PSDU_MAX) {
		return -1;
	}

	/* Frame air time in microseconds. */
	duration = (uint64_t)(SHR_PHR_OCTETS + len) * 8U * 1000000U /
		   PHY_BITRATE;

	/* RadioCommEventData. */
	data[0] = channel;              /* channel */
	data[1] = (uint8_t)power;       /* power */
	data[2] = OTNS_ERROR_NONE;      /* error */
	put_le64(&data[3], duration);   /* duration */
	/* RadioMessage: channel byte + PSDU. */
	data[RADIO_COMM_EVENT_DATA_LEN] = channel;
	memcpy(&data[RADIO_COMM_EVENT_DATA_LEN + 1], psdu, len);

	if (write_event(OTNS_EVENT_RADIO_COMM_START, 0, data,
			     (uint16_t)(RADIO_COMM_EVENT_DATA_LEN + 1 + len)) < 0) {
		return -1;
	}

	wake_pacer_now();
	return 0;
}

/*
 * HW-event callback: emit the deferred ACK once virtual time reaches ack_time.
 * Runs before the pacer (lower priority number) at that timestamp, so the
 * COMM_START is queued to OTNS with the node's current (turnaround-delayed)
 * virtual time, exactly like a normal transmission scheduled for that instant.
 */
static void ack_timer_fired(void)
{
	ack_time = NSI_NEVER;
	(void)nsi_otns_bottom_tx(ack_channel, ack_power, ack_psdu, ack_psdu_len);
}

NSI_HW_EVENT(ack_time, ack_timer_fired, 940);

int nsi_otns_bottom_tx_after(uint8_t channel, int8_t power,
			     const uint8_t *psdu, uint16_t len,
			     uint32_t delay_us)
{
	if (!connected) {
		return -1;
	}
	if (len > OTNS_PSDU_MAX) {
		return -1;
	}

	ack_channel = channel;
	ack_power = power;
	memcpy(ack_psdu, psdu, len);
	ack_psdu_len = len;
	ack_time = nsi_hws_get_time() + delay_us;

	/* Recompute the HW scheduler's next-event time to include ack_time. */
	nsi_hws_find_next_event();
	return 0;
}

int nsi_otns_bottom_cca(uint8_t channel)
{
	uint8_t data[11];

	if (!connected) {
		return -1;
	}

	data[0] = channel;
	data[1] = 0;
	data[2] = OTNS_ERROR_NONE;
	put_le64(&data[3], CCA_DURATION_US);

	if (write_event(OTNS_EVENT_RADIO_CHAN_SAMPLE, 0, data,
			     sizeof(data)) < 0) {
		return -1;
	}

	wake_pacer_now();
	return 0;
}

void nsi_otns_bottom_set_state(uint8_t state, uint8_t channel)
{
	/*
	 * Only cache the new radio state here; it is reported to OTNS (with
	 * change detection) by report_state() right before the next sleep
	 * event, so that the RADIO_STATE always precedes the ALARM_FIRED in the
	 * same node "turn" — matching the reference otSysProcessDrivers()
	 * sequencing.
	 */
	radio_state = state;
	radio_channel = channel;
	wake_pacer_now();
}

void nsi_otns_bottom_send_uart(const uint8_t *buf, uint16_t len)
{
	if (!connected || buf == NULL || len == 0) {
		return;
	}

	(void)write_event(OTNS_EVENT_UART_WRITE, 0, buf, len);
	wake_pacer_now();
}

void nsi_otns_bottom_send_ext_addr(const uint8_t *ext_addr_be)
{
	if (ext_addr_be == NULL) {
		return;
	}

	/* Cache so it can be (re)sent once the OTNS connection is up. */
	memcpy(ext_addr, ext_addr_be, EXTENDED_ADDR_LEN);
	ext_addr_valid = true;

	if (!connected) {
		return;
	}

	(void)write_event(OTNS_EVENT_EXT_ADDR, 0, ext_addr,
			       EXTENDED_ADDR_LEN);
	wake_pacer_now();
}

void nsi_otns_bottom_send_status(const char *status, uint16_t len)
{
	if (!connected || status == NULL || len == 0) {
		return;
	}

	(void)write_event(OTNS_EVENT_STATUS_PUSH, 0,
			       (const uint8_t *)status, len);
	wake_pacer_now();
}

bool nsi_otns_bottom_get_event(struct otns_radio_event *ev)
{
	if (!deliver_ready) {
		return false;
	}

	*ev = deliver_ev;
	deliver_ready = false;
	return true;
}
