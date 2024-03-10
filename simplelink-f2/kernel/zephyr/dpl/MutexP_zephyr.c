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
 *  ======== MutexP_zephyr.c ========
 */

#include <ti/drivers/dpl/MutexP.h>

#include <zephyr/kernel.h>


/*
 *  ======== MutexP_construct ========
 */
MutexP_Handle MutexP_construct(MutexP_Struct *handle, MutexP_Params *params)
{
	struct k_mutex* mutexP = (struct k_mutex*)handle;
    if (mutexP == NULL) {
		return NULL;
	}

    if(k_mutex_init(mutexP) != 0) {
        return NULL;
    }

    return ((MutexP_Handle)mutexP);
}

/*
 *  ======== MutexP_destruct ========
 */
void MutexP_destruct(MutexP_Struct *mutexP)
{
    if(mutexP == NULL) {
        return;
    }

    k_mutex_init(mutexP);
}

/*
 *  ======== MutexP_create ========
 */
MutexP_Handle MutexP_create(MutexP_Params *params)
{
    MutexP_Struct* handle = k_malloc(MutexP_STRUCT_SIZE);
    if(MutexP_construct(handle, params) == NULL) {
        k_free(handle);
    }

    return ((MutexP_Handle)handle);
}

/*
 *  ======== MutexP_delete ========
 */
void MutexP_delete(MutexP_Handle handle)
{
    struct k_mutex* mutexP = (struct k_mutex*)handle;
    if(mutexP == NULL) {
        return;
    }

    MutexP_destruct(mutexP);
    k_free(mutexP);
}

/*
 *  ======== MutexP_Params_init ========
 */
void MutexP_Params_init(MutexP_Params *params)
{
    if(params == NULL) {
        return;
    }
    params->callback = NULL;
}

/*
 *  ======== MutexP_lock ========
 */
uintptr_t MutexP_lock(MutexP_Handle handle)
{
    struct k_mutex* mutexP = (struct k_mutex*)handle;
    if(mutexP == NULL) {
        return 0;
    }

	k_mutex_lock(mutexP, K_FOREVER);

    return 0;
}

/*
 *  ======== MutexP_unlock ========
 */
void MutexP_unlock(MutexP_Handle handle, uintptr_t key)
{
    struct k_mutex* mutexP = (struct k_mutex*)handle;
    if(mutexP == NULL) {
        return;
    }

	k_mutex_unlock(mutexP);

    return;
}
