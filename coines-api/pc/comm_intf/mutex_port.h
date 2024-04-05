/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    mutex_port.h
 * @brief This file contains communication interface supporting data structures
 *
 */
#ifndef COMM_INTF_MUTEX_PORT_H_
#define COMM_INTF_MUTEX_PORT_H_

#ifdef PLATFORM_WINDOWS
#include <windows.h>

#ifdef __cplusplus
extern "C"
{
#endif
//lint -e64
typedef CRITICAL_SECTION mutex_t;

/*!
 * @brief API to lock mutex
 *
 * @param    : Pointer to the mutex
 *
 * @return void
 */
static void mutex_lock(mutex_t *mutex)
{
    EnterCriticalSection(mutex);
}

/*!
 * @brief API to unlock mutex
 *
 * @param[out] : Pointer to the mutex
 *
 * @return void
 */
static void mutex_unlock(mutex_t *mutex)
{
    LeaveCriticalSection(mutex);
}

/*!
 * @brief API to initiate mutex
 *
 * @param   : Pointer to the mutex
 *
 * @return void
 */
static void mutex_init(mutex_t *mutex)
{
    InitializeCriticalSection(mutex);
}

/*!
 * @brief API to destroy mutex
 *
 * @param   : Pointer to the mutex
 *
 * @return void
 */
static void mutex_destroy(mutex_t *mutex)
{
    DeleteCriticalSection(mutex);
}

#ifdef __cplusplus
}
#endif

#endif

#ifdef PLATFORM_LINUX
#include <pthread.h>
#include <unistd.h>

typedef pthread_mutex_t mutex_t;

/*!
 * @brief API to lock mutex
 *
 * @param   : Pointer to the mutex
 *
 * @return void
 */
static void mutex_lock(mutex_t *mutex)
{
pthread_mutex_lock(mutex);
}

/*!
 * @brief API to unlock mutex
 *
 * @param   : Pointer to the mutex
 *
 * @return void
 */static void mutex_unlock(mutex_t *mutex)
{
pthread_mutex_unlock(mutex);
}

/*!
 * @brief API to initiate mutex
 *
 * @param   : Pointer to the mutex
 *
 * @return void
 */
static void mutex_init(mutex_t *mutex)
{
pthread_mutex_init(mutex, 0);
}

/*!
 * @brief API to delete mutex
 *
 * @param   : Pointer to the mutex
 *
 * @return void
 */
static void mutex_destroy(mutex_t *mutex)
{
pthread_mutex_destroy(mutex);
}
#endif /* COMM_INTF_MUTEX_PORT_H_ */

#endif

/** @}*/
