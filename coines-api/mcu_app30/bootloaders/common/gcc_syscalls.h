/*!
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    compiler_warning_handler.h
 * @date    June 28, 2023
 * @brief   This file provides the implementation of weak symbol functions that are used to handle gcc toolchain warnings.
 *
 */

#ifndef COMPILER_WARNING_HANDLER_H_
#define COMPILER_WARNING_HANDLER_H_

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <sys/types.h>
#include <sys/stat.h>

/**********************************************************************************/
/* function prototype declarations */
/**********************************************************************************/

int _fstat(int fd, struct stat *buf);
int _isatty(int fd);
int _lseek(int fd, int pos, int whence);
void _kill(int pid, int sig);
int _getpid(void);

#endif /* COMPILER_WARNING_HANDLER_H_ */
/** @}*/