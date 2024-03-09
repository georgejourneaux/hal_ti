/*
 * Copyright (c) 2015-2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== ClockP_zephyr.c ========
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <zephyr/kernel.h>

#include <ti/drivers/dpl/ClockP.h>

#define US_PER_S (1000000UL)
#define CLOCKP_TICKPERIOD_US (US_PER_S / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

typedef struct _ClockP_Obj
{
    struct k_timer timer;
	ClockP_Fxn clock_fxn;
	uintptr_t arg;
	uint32_t timeout; /* in sys clock uptime ticks */
	uint32_t period; /* in sys clock uptime ticks */
	bool active;
} ClockP_Obj;


static void expiryCallbackFunction(struct k_timer *timer_id);

/*
 *  ======== ClockP_construct ========
 */
ClockP_Handle ClockP_construct(ClockP_Struct *clockP, ClockP_Fxn clockFxn, uint32_t timeout, ClockP_Params *params)
{
	ClockP_Obj* clockPObj = (ClockP_Obj*)clockP;
    ClockP_Params ClockP_defaultParams;

	if (clockPObj == NULL) {
		return NULL;
	}

	if (params == NULL) {
		params = &ClockP_defaultParams;
        ClockP_Params_init(params);
	}

	clockPObj->clock_fxn = clockFxn;
	clockPObj->arg = params->arg;
	clockPObj->period = params->period;
	obclockPObjj->timeout = timeout;
	clockPObj->active = false;
	
	k_timer_init(&clockPObj->timer, expiryCallbackFunction, NULL);
	k_timer_user_data_set(&clockPObj->timer, clockPObj);
	
	if (params->startFlag) {
		ClockP_start(clockPObj);
	}
	
	return ((ClockP_Handle)clockPObj);
}

/*
 *  ======== ClockP_destruct ========
 */
void ClockP_destruct(ClockP_Struct *clockP)
{
    ClockP_Obj* clockPObj = (ClockP_Obj*)clockP->data;
    if(clockPObj == NULL) {
        return;
    }

	clockPObj->clock_fxn = NULL;
	clockPObj->arg = 0;
	clockPObj->period = 0;
	clockPObj->timeout = 0;
	clockPObj->active = false;

	k_timer_stop(&clockPObj->timer);
}

/*
 *  ======== ClockP_create ========
 */
ClockP_Handle ClockP_create(ClockP_Fxn clockFxn, uint32_t timeout, ClockP_Params *params)
{
    ClockP_Struct* clockP = k_malloc(ClockP_STRUCT_SIZE);
    if(ClockP_construct(clockP, clockFxn, timeout, params) == NULL) {
        k_free(clockP);
    }

	return ((ClockP_Handle)clockP);
}

/*
 *  ======== ClockP_delete ========
 */
void ClockP_delete(ClockP_Handle handle)
{
    ClockP_Struct* clockP = (ClockP_Struct*)handle;
    if(clockP == NULL) {
        return;
    }

    ClockP_destruct(clockP);
    k_free(handle);
}

/*
 *  ======== ClockP_getCpuFreq ========
 */
void ClockP_getCpuFreq(ClockP_FreqHz *freq)
{
    freq->lo = (uint32_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    freq->hi = 0;
}

/*
 *  ======== ClockP_getSystemTickPeriod ========
 */
uint32_t ClockP_getSystemTickPeriod(void)
{
    return (CLOCKP_TICK_PERIOD);
}

/*
 *  ======== ClockP_getSystemTicks ========
 */
uint32_t ClockP_getSystemTicks(void)
{
    return k_uptime_ticks();
}

/*
 *  ======== ClockP_getTimeout ========
 */
uint32_t ClockP_getTimeout(ClockP_Handle handle)
{
    ClockP_Obj* clockPObj = (ClockP_Obj*)handle;
    if(clockPObj == NULL) {
        return 0;
    }

    return clockPObj->active ? k_timer_remaining_ticks(&clockPObj->timer) : clockPObj->timeout;
}

/*
 *  ======== ClockP_isActive ========
 */
bool ClockP_isActive(ClockP_Handle handle)
{
    ClockP_Obj* clockPObj = (ClockP_Obj*)handle;
    if(clockPObj == NULL) {
        return false;
    }

    return clockPObj->active;
}

/*
 *  ======== ClockP_Params_init ========
 */
void ClockP_Params_init(ClockP_Params *params)
{
    params->startFlag = false;
    params->period    = 0;
    params->arg       = (uintptr_t)0;
}

/*
 *  ======== ClockP_setTimeout ========
 */
void ClockP_setTimeout(ClockP_Handle handle, uint32_t timeout)
{
    ClockP_Obj* clockPObj = (ClockP_Obj*)handle;
    if(clockPObj == NULL) {
        return;
    }

	clockPObj->timeout = timeout;
}

/*
 *  ======== ClockP_setPeriod ========
 */
void ClockP_setPeriod(ClockP_Handle handle, uint32_t period)
{
    ClockP_Obj* clockPObj = (ClockP_Obj*)handle;
    if(clockPObj == NULL) {
        return;
    }

	clockPObj->period = period;
}

/*
 *  ======== ClockP_start ========
 */
void ClockP_start(ClockP_Handle handle)
{
    ClockP_Obj* clockPObj = (ClockP_Obj*)handle;
    if(clockPObj == NULL) {
        return;
    }

	k_timer_start(&clockPObj->timer, K_TICKS(clockPObj->timeout), K_TICKS(clockPObj->period));
	clockPObj->active = true;
}

/*
 *  ======== ClockP_stop ========
 */
void ClockP_stop(ClockP_Handle handle)
{
    ClockP_Obj* clockPObj = (ClockP_Obj*)handle;
    if(clockPObj == NULL) {
        return;
    }

	k_timer_stop(&clockPObj->timer);
	clockPObj->active = false;
}

/*
 *  ======== ClockP_usleep ========
 */
void ClockP_usleep(uint32_t usec)
{
	k_sleep(K_USEC(usec));
}

/*
 *  ======== ClockP_sleep ========
 */
void ClockP_sleep(uint32_t sec)
{
    k_sleep(K_SECONDS(sec))
}

/*
 *  ======== expiryCallbackFunction ========
 */
static void expiryCallbackFunction(struct k_timer *timer_id)
{
	ClockP_Obj* clockPObj = (ClockP_Obj*)k_timer_user_data_get(timer_id);
    if(clockPObj == NULL) {
        return;
    }
	
	clockPObj->clock_fxn(clockPObj->arg);
}
