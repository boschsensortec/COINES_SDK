/*
 * Copyright (c) 2021 Martin Storsjo
 *
 * This file is part of llvm-mingw.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "throwcatch-lib.h"
#include <stdio.h>

void libFunc(int param) {
    switch (param) {
    case 1:
        fprintf(stderr, "throwing FirstException\n");
        fflush(stderr);
        throw FirstException("first");
    case 2:
        fprintf(stderr, "throwing SecondException\n");
        fflush(stderr);
        throw SecondException("second");
    case 3:
        fprintf(stderr, "throwing std::exception\n");
        fflush(stderr);
        throw std::exception();
    default:
        fprintf(stderr, "not throwing\n");
        break;
    }
}
