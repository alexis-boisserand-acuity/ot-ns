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
 * Embedded ("top") side of the ISR-driven OpenThread alarm variant.
 *
 * Overrides Zephyr's stock k_timer-based alarm_milli.c/alarm_micro.c (via
 * -Wl,--wrap, see CMakeLists.txt) with deadlines scheduled on the
 * native simulator's hardware-event scheduler via alarm_isr_bottom.c, fired
 * through two dedicated interrupts instead of a k_timer expiry callback. This
 * decouples alarm precision from Zephyr's system tick
 * (CONFIG_SYS_CLOCK_TICKS_PER_SEC) since the native simulator's virtual clock
 * (nsi_hws_get_time(), microsecond resolution) is used directly for both
 * "now" and deadline scheduling.
 */

#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <openthread-system.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/radio.h>

#include "ieee802154_priv.h"

struct alarm
{
    volatile bool fired;
    uint32_t      us_per_tick;
};

static struct alarm alarms[OTNS_ALARM_COUNT] = {
    [OTNS_ALARM_MILLI] = {.us_per_tick = 1000},
    [OTNS_ALARM_MICRO] = {.us_per_tick = 1},
};
static int32_t time_offset_ms;
static int32_t time_offset_us;

static void alarm_isr(const void *arg)
{
    struct alarm *alarm = (struct alarm *)arg;

    alarm->fired = true;
    otSysEventSignalPending();
}

static uint32_t alarm_milli_get_now(void)
{
    return (uint32_t)(nsi_otns_bottom_get_time_us() / 1000) - time_offset_ms - nsi_otns_bottom_get_drift_offset_ms();
}

static uint32_t alarm_micro_get_now(void) { return (uint32_t)nsi_otns_bottom_get_time_us() - time_offset_us; }

static uint32_t alarm_get_now(enum otns_alarm which)
{
    return which == OTNS_ALARM_MILLI ? alarm_milli_get_now() : alarm_micro_get_now();
}

static void alarm_start_at(enum otns_alarm which, uint32_t t0, uint32_t dt)
{
    struct alarm *alarm = &alarms[which];
    unsigned int  key   = irq_lock();

    nsi_otns_bottom_alarm_cancel(which);
    alarm->fired  = false;
    int32_t delta = (int32_t)(t0 + dt - alarm_get_now(which));

    if (delta > 0)
    {
        nsi_otns_bottom_alarm_set_at(which, nsi_otns_bottom_get_time_us() + (uint64_t)delta * alarm->us_per_tick);
    }
    else
    {
        alarm_isr(alarm);
    }
    irq_unlock(key);
}

static void alarm_stop(enum otns_alarm which)
{
    unsigned int key = irq_lock();

    nsi_otns_bottom_alarm_cancel(which);
    alarms[which].fired = false;
    irq_unlock(key);
}

static bool alarm_take_fired(enum otns_alarm which)
{
    unsigned int key   = irq_lock();
    bool         fired = alarms[which].fired;

    alarms[which].fired = false;
    irq_unlock(key);
    return fired;
}

uint32_t __wrap_otPlatAlarmMilliGetNow(void);
uint32_t __wrap_otPlatAlarmMicroGetNow(void);
void     __wrap_platformAlarmMilliInit(void);
void     __wrap_platformAlarmMicroInit(void);
void     __wrap_platformAlarmMilliProcess(otInstance *aInstance);
void     __wrap_platformAlarmMicroProcess(otInstance *aInstance);
void     __wrap_otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt);
void     __wrap_otPlatAlarmMilliStop(otInstance *aInstance);
void     __wrap_otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt);
void     __wrap_otPlatAlarmMicroStop(otInstance *aInstance);
void     __wrap_alarm_milli_set_time_offset_ms(int32_t offset_ms);

void __wrap_alarm_milli_set_time_offset_ms(int32_t offset_ms) { time_offset_ms = offset_ms; }

uint32_t __wrap_otPlatAlarmMilliGetNow(void) { return alarm_milli_get_now(); }

uint32_t __wrap_otPlatAlarmMicroGetNow(void) { return alarm_micro_get_now(); }

void __wrap_platformAlarmMilliInit(void) {}

void __wrap_platformAlarmMicroInit(void)
{
    static bool irqs_installed;

    if (!irqs_installed)
    {
        irqs_installed = true;
        IRQ_CONNECT(IEEE802154_OTNS_ALARM_MILLI_IRQ, 0, alarm_isr, &alarms[OTNS_ALARM_MILLI], 0);
        IRQ_CONNECT(IEEE802154_OTNS_ALARM_MICRO_IRQ, 0, alarm_isr, &alarms[OTNS_ALARM_MICRO], 0);
        irq_enable(IEEE802154_OTNS_ALARM_MILLI_IRQ);
        irq_enable(IEEE802154_OTNS_ALARM_MICRO_IRQ);
    }

#if defined(CONFIG_OPENTHREAD_PLATFORM_PKT_TXTIME)
    time_offset_us = (int32_t)((int64_t)alarm_micro_get_now() - (uint32_t)otPlatRadioGetNow(NULL));
    __wrap_alarm_milli_set_time_offset_ms(time_offset_us / 1000);
#endif
}

void __wrap_platformAlarmMilliProcess(otInstance *aInstance)
{
    otns_host_msg_process(aInstance);

    if (alarm_take_fired(OTNS_ALARM_MILLI))
    {
#if defined(CONFIG_OPENTHREAD_DIAG)
        if (otPlatDiagModeGet())
        {
            otPlatDiagAlarmFired(aInstance);
        }
        else
        {
#endif
            otPlatAlarmMilliFired(aInstance);
#if defined(CONFIG_OPENTHREAD_DIAG)
        }
#endif
    }
}

void __wrap_platformAlarmMicroProcess(otInstance *aInstance)
{
#if defined(CONFIG_OPENTHREAD_PLATFORM_USEC_TIMER)
    if (alarm_take_fired(OTNS_ALARM_MICRO))
    {
        otPlatAlarmMicroFired(aInstance);
    }
#else
    ARG_UNUSED(aInstance);
#endif
}

void __wrap_otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    ARG_UNUSED(aInstance);

    alarm_start_at(OTNS_ALARM_MILLI, aT0, aDt);
}

void __wrap_otPlatAlarmMilliStop(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    alarm_stop(OTNS_ALARM_MILLI);
}

void __wrap_otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    ARG_UNUSED(aInstance);

    alarm_start_at(OTNS_ALARM_MICRO, aT0, aDt);
}

void __wrap_otPlatAlarmMicroStop(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    alarm_stop(OTNS_ALARM_MICRO);
}
