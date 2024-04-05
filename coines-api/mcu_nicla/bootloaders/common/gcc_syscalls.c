/*!
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    compiler_warning_handler.c
 * @date    June 28, 2023
 * @brief   This file provides the implementation of weak symbol functions that are used to handle gcc toolchain warnings.
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <sys/types.h>
#include <sys/stat.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "gcc_syscalls.h"

/*********************************************************************/
/* Weak Attribute Functions */
/*********************************************************************/

int __attribute__((weak)) _fstat(int fd, struct stat *buf)
{
    return -1;
}

int __attribute__((weak)) _isatty(int fd)
{
    return -1;
}

int __attribute__((weak)) _lseek(int fd, int pos, int whence)
{
    return -1;
}

void __attribute__((weak)) _kill(int pid, int sig)
{

}

int __attribute__((weak)) _getpid(void)
{
    return -1;
}

int __attribute__((weak)) _close(int fd)
{
    return -1;
}

int __attribute__((weak)) _write(int fd, const void *buffer, int len)
{
    return -1;
}

int __attribute__((weak)) _read(int fd, void *buffer, int len)
{
    return -1;
}

/** @}*/