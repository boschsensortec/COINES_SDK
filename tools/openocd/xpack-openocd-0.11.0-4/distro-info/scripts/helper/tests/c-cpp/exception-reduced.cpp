/*
 * Copyright (c) 2019 Marc Aldorasi
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

#include <new>

typedef void (*func_ptr)(int);
struct h {
    func_ptr z;
};
struct c {
    h* e;
};
struct as {
    as() { at = static_cast<int *>(operator new(sizeof(int))); }
    ~as() { operator delete(at); }
    int *at;
};
void am(int) {
    static as au;
    as av;
    throw 0;
}
int main(int argc, char* argv[]) {
    h hh{am};
    c *u = new c{&hh};
    try {
        u->e->z(0);
    } catch (...) {
    }
    return 0;
}
