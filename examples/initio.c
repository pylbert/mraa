/*
 * Author: Houman Brinjcargorabi <houman.brinjcargorabi@intel.com>
 * Contributions: Noel Eck <noel.eck@intel.com>
 * Copyright (c) 2016 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "initio.h"


int main()
{
    char error_str[1024] = {0};

    const char* initstr_0 = "a:1d,i:1:0x20:fast,g:13:in,a:0x10";
    printf("MRAA IO init string: %s\n", initstr_0);

    mraa_io_descriptor* desc = mraa_io_init(initstr_0, error_str);
    if (error_str[0]) printf("%s\n", error_str);
    print_descriptor(desc);

    if (mraa_io_close(desc) != MRAA_SUCCESS)
        fprintf(stderr, "Failed to close MRAA io\n");

    return 0;
}
