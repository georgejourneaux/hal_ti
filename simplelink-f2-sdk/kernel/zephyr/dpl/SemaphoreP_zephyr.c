/*
 * Copyright (c) 2015-2022, Texas Instruments Incorporated
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
 *  ======== SemaphoreP_zephyr.c ========
 */
#include <ti/drivers/dpl/SemaphoreP.h>

#include <zephyr/kernel.h>

/*
 *  ======== SemaphoreP_construct ========
 */
SemaphoreP_Handle SemaphoreP_construct(SemaphoreP_Struct *handle, unsigned int count, SemaphoreP_Params *params)
{
    struct k_sem* semaphoreP = (struct k_sem*)handle;
    if (semaphoreP == NULL) {
		return NULL;
	}

    if(k_sem_init(semaphoreP, count, ((params->mode == SemaphoreP_Mode_BINARY) ? 1 : K_SEM_MAX_LIMIT)) != 0) {
        return NULL;
    }

    return ((SemaphoreP_Handle)semaphoreP);
}

/*
 *  ======== SemaphoreP_constructBinary ========
 */
SemaphoreP_Handle SemaphoreP_constructBinary(SemaphoreP_Struct *handle, unsigned int count)
{
    SemaphoreP_Params params = {
        .callback == NULL,
        .mode = SemaphoreP_Mode_BINARY
    };

    return SemaphoreP_construct(handle, count, params);
}

/*
 *  ======== SemaphoreP_destruct ========
 */
void SemaphoreP_destruct(SemaphoreP_Struct *semP)
{
    struct k_sem* semaphoreP = (struct k_sem*)semP;
    if (semaphoreP == NULL) {
		return;
	}

	k_sem_reset(semaphoreP);
}

/*
 *  ======== SemaphoreP_create ========
 */
SemaphoreP_Handle SemaphoreP_create(unsigned int count, SemaphoreP_Params *params)
{
    SemaphoreP_Struct* handle = k_malloc(SemaphoreP_STRUCT_SIZE);
    if(SemaphoreP_construct(handle, count, params) == NULL) {
        k_free(handle);
    }

    return ((SemaphoreP_Handle)handle);
}

/*
 *  ======== SemaphoreP_createBinary ========
 */
SemaphoreP_Handle SemaphoreP_createBinary(unsigned int count)
{
    SemaphoreP_Struct* handle = k_malloc(SemaphoreP_STRUCT_SIZE);
    if(SemaphoreP_constructBinary(handle, count) == NULL) {
        k_free(handle);
    }

    return ((SemaphoreP_Handle)handle);
}

/*
 *  ======== SemaphoreP_delete ========
 */
void SemaphoreP_delete(SemaphoreP_Handle handle)
{
    struct k_sem* semaphoreP = (struct k_sem*)handle;
    if(semaphoreP == NULL) {
        return;
    }

    SemaphoreP_destruct(semaphoreP);
    k_free(semaphoreP);
}

/*
 *  ======== SemaphoreP_Params_init ========
 */
void SemaphoreP_Params_init(SemaphoreP_Params *params)
{
    if(params == NULL) {
        return;
    }
    
    params->mode     = SemaphoreP_Mode_COUNTING;
    params->callback = NULL;
}

/*
 *  ======== SemaphoreP_pend ========
 */
SemaphoreP_Status SemaphoreP_pend(SemaphoreP_Handle handle, uint32_t timeout)
{
    struct k_sem* semaphoreP = (struct k_sem*)handle;
    if(semaphoreP == NULL) {
        return;
    }

    if(k_sem_take(semaphoreP, timeout) == 0)
    {
        return SemaphoreP_OK;
    }

    return SemaphoreP_TIMEOUT;
}

/*
 *  ======== SemaphoreP_post ========
 */
void SemaphoreP_post(SemaphoreP_Handle handle)
{
    struct k_sem* semaphoreP = (struct k_sem*)handle;
    if(semaphoreP == NULL) {
        return;
    }

    k_sem_give(semaphoreP);
}
