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

static volatile bool milli_fired;
static volatile bool micro_fired;
static int32_t       time_offset_ms;
static int32_t       time_offset_us;

static void alarm_milli_isr(const void *arg)
{
    ARG_UNUSED(arg);

    milli_fired = true;
    otSysEventSignalPending();
}

static void alarm_micro_isr(const void *arg)
{
    ARG_UNUSED(arg);

    micro_fired = true;
    otSysEventSignalPending();
}

static uint32_t alarm_milli_get_now(void) { return (uint32_t)(nsi_otns_bottom_get_time_us() / 1000) - time_offset_ms; }

static uint32_t alarm_micro_get_now(void) { return (uint32_t)nsi_otns_bottom_get_time_us() - time_offset_us; }

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

void otns_apply_pending_drift_offset(void)
{
    static int32_t last_offset_ms;
    int32_t        offset_ms = nsi_otns_bottom_get_drift_offset_ms();

    if (offset_ms != last_offset_ms)
    {
        last_offset_ms = offset_ms;
        __wrap_alarm_milli_set_time_offset_ms(offset_ms);
    }
}

uint32_t __wrap_otPlatAlarmMilliGetNow(void) { return alarm_milli_get_now(); }

uint32_t __wrap_otPlatAlarmMicroGetNow(void) { return alarm_micro_get_now(); }

void __wrap_platformAlarmMilliInit(void)
{
    /* IRQs for both alarms are installed once, from __wrap_platformAlarmMicroInit() below. */
}

void __wrap_platformAlarmMicroInit(void)
{
    static bool irqs_installed;

    if (!irqs_installed)
    {
        irqs_installed = true;
        IRQ_CONNECT(IEEE802154_OTNS_ALARM_MILLI_IRQ, 0, alarm_milli_isr, NULL, 0);
        IRQ_CONNECT(IEEE802154_OTNS_ALARM_MICRO_IRQ, 0, alarm_micro_isr, NULL, 0);
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
    if (milli_fired)
    {
        milli_fired = false;
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
    /* OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE isn't visible here (it comes from
     * OPENTHREAD_PROJECT_CORE_CONFIG_FILE, only applied within modules/openthread/); use the
     * underlying Kconfig symbol instead, which is globally available via autoconf.h.
     */
#if defined(CONFIG_OPENTHREAD_PLATFORM_USEC_TIMER)
    if (micro_fired)
    {
        micro_fired = false;
        otPlatAlarmMicroFired(aInstance);
    }
#else
    ARG_UNUSED(aInstance);
#endif
}

void __wrap_otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    ARG_UNUSED(aInstance);

    int32_t delta = (int32_t)(aT0 + aDt - alarm_milli_get_now());

    if (delta > 0)
    {
        nsi_otns_bottom_alarm_set_milli_at(nsi_otns_bottom_get_time_us() + (uint64_t)delta * 1000U);
    }
    else
    {
        nsi_otns_bottom_alarm_cancel_milli();
        alarm_milli_isr(NULL);
    }
}

void __wrap_otPlatAlarmMilliStop(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    nsi_otns_bottom_alarm_cancel_milli();
}

void __wrap_otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    ARG_UNUSED(aInstance);

    int32_t delta = (int32_t)(aT0 + aDt - alarm_micro_get_now());

    if (delta > 0)
    {
        nsi_otns_bottom_alarm_set_micro_at(nsi_otns_bottom_get_time_us() + (uint64_t)delta);
    }
    else
    {
        nsi_otns_bottom_alarm_cancel_micro();
        alarm_micro_isr(NULL);
    }
}

void __wrap_otPlatAlarmMicroStop(otInstance *aInstance)
{
    ARG_UNUSED(aInstance);

    nsi_otns_bottom_alarm_cancel_micro();
}
