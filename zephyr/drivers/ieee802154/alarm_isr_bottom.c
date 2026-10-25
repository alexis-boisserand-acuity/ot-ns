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
 * Runner ("bottom") side of the ISR-driven OpenThread alarm variant.
 *
 * Schedules otPlatAlarmMilli/otPlatAlarmMicro deadlines directly on the
 * native simulator's hardware-event scheduler (microsecond virtual time),
 * instead of relying on Zephyr's k_timer (whose resolution on native_sim is
 * limited by CONFIG_SYS_CLOCK_TICKS_PER_SEC). Each deadline raises its own
 * dedicated interrupt, pulled by alarm_isr.c on the embedded side - the same
 * pattern used by the radio/CLI IRQs in ieee802154_bottom.c/cli_bottom.c.
 */

#include "irq_ctrl.h"
#include "nsi_hw_scheduler.h"
#include "nsi_hws_models_if.h"

#include "ieee802154_priv.h"

static uint64_t alarm_time_milli_us = NSI_NEVER;
static uint64_t alarm_time_micro_us = NSI_NEVER;

static uint64_t *const alarm_time_us[OTNS_ALARM_COUNT] = {
    [OTNS_ALARM_MILLI] = &alarm_time_milli_us,
    [OTNS_ALARM_MICRO] = &alarm_time_micro_us,
};

static const unsigned int alarm_irq[OTNS_ALARM_COUNT] = {
    [OTNS_ALARM_MILLI] = IEEE802154_OTNS_ALARM_MILLI_IRQ,
    [OTNS_ALARM_MICRO] = IEEE802154_OTNS_ALARM_MICRO_IRQ,
};

static void alarm_timer_fired(enum otns_alarm alarm)
{
    *alarm_time_us[alarm] = NSI_NEVER;
    hw_irq_ctrl_raise_im(alarm_irq[alarm]);
}

static void alarm_milli_timer_fired(void) { alarm_timer_fired(OTNS_ALARM_MILLI); }

static void alarm_micro_timer_fired(void) { alarm_timer_fired(OTNS_ALARM_MICRO); }

NSI_HW_EVENT(alarm_time_milli_us, alarm_milli_timer_fired, 942);
NSI_HW_EVENT(alarm_time_micro_us, alarm_micro_timer_fired, 944);

uint64_t nsi_otns_bottom_get_time_us(void) { return nsi_hws_get_time(); }

void nsi_otns_bottom_alarm_set_at(enum otns_alarm alarm, uint64_t at_us)
{
    *alarm_time_us[alarm] = at_us;
    nsi_hws_find_next_event();
}

void nsi_otns_bottom_alarm_cancel(enum otns_alarm alarm)
{
    *alarm_time_us[alarm] = NSI_NEVER;
    hw_irq_ctrl_clear_irq(alarm_irq[alarm]);
    nsi_hws_find_next_event();
}
