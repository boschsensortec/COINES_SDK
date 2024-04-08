/*!
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 * BSD-3-Clause
 * Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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