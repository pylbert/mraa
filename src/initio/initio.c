/*
 * Author: Noel Eck <noel.eck@intel.com>
 * Copyright (c) 2014-2016 Intel Corporation.
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

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <limits.h>


#include "initio.h"
#include "mraa_internal.h"

static mraa_result_t mraa_atoi_x(const char* intStr, char* str_end, int* value, int base)
{
    long val = strtol(intStr, &str_end, base);
    if (*str_end != '\0' || errno == ERANGE || str_end == intStr || val > INT_MAX || val < INT_MIN) {
        *value = 0;
        return MRAA_ERROR_UNSPECIFIED;
    }
    *value = (int) val;
    return MRAA_SUCCESS;
}


/* AIO_KEY:aio pin[:num_bits] */
static mraa_aio_context parse_aio(const char* proto, char* error_string)
{
    printf("Parsing AIO string: %s\n", proto);
    /* proto + 1 should be ':' */
    /* proto + 2 should be the start of the char* representing the pin */
    if (proto[1] != ':') {
        if (error_string) sprintf(error_string,
                "Failed parsing AIO string '%s' (missing ':'). AIO syntax: AIO_KEY:aio pin[:num_bits]", proto);
        return NULL;
    }

    int pin = 0;
    char str_end[20];
    if(mraa_atoi_x(&proto[2], str_end, &pin, 0) != MRAA_SUCCESS) {
        if (error_string) sprintf(error_string,
                "Failed parsing AIO string '%s' (unable to convert %s to integer). AIO syntax: AIO_KEY:aio pin[:num_bits]", proto, str_end);
        return NULL;

    }

    printf("XXX pin = %d\n", pin);

    return mraa_aio_init(0);
}

mraa_io_descriptor* mraa_io_init(const char* strdesc, char* error_string)
{
    /* Allocate space for a local strdesc, copy over and null-terminate */
    char local_desc[1025];
    char *local_desc_p = local_desc;
    strncpy(local_desc, strdesc, 1024);
    local_desc[1024] = '\0';

    /* If space is provided for an error string, us it */
    char *local_errstr = NULL;
    if (error_string) local_errstr = error_string;

    /* Allocate space for the descriptor */
    mraa_io_descriptor* desc = calloc(1, sizeof(mraa_io_descriptor));
    if (desc == NULL) {
        syslog(LOG_ERR, "mraa_io_init: Failed to allocate memory for context");
        return NULL;
    }

    const char delimiters[] = ",";
    char* proto = strsep(&local_desc_p, delimiters);
    while (proto != NULL)
    {
        if (strncmp(proto, AIO_KEY, strlen(AIO_KEY) ) == 0) {
            /* Parse the AIO protocol */
            mraa_aio_context dev = parse_aio(proto, local_errstr);
            if (!dev) return NULL;

            /* Allocate space for the new AIO context */
            desc->aios = realloc(desc->aios, sizeof(mraa_aio_context) * (desc->n_aio + 1));
            if (!desc->aios) {
                if (local_errstr) strcpy(local_errstr, "mraa_io_init: Failed to allocate memory for context");
                return NULL;
            }
            desc->aios[desc->n_aio++] = dev;
        }
        else if (strncmp(proto, GPIO_KEY, strlen(GPIO_KEY) ) == 0) {
        }
        else if (strncmp(proto, I2C_KEY, strlen(I2C_KEY) ) == 0) {
        }
        else if (strncmp(proto, IIO_KEY, strlen(IIO_KEY) ) == 0) {
        }
        else if (strncmp(proto, PWM_KEY, strlen(PWM_KEY) ) == 0) {
        }
        else if (strncmp(proto, SPI_KEY, strlen(SPI_KEY) ) == 0) {
        }
        else if (strncmp(proto, UART_KEY, strlen(UART_KEY) ) == 0) {
        }
        else if (strncmp(proto, UART_OW_KEY, strlen(UART_OW_KEY) ) == 0) {
        }
        else {}

        proto = strsep(&local_desc_p, ",");
    }

    return desc;
}

mraa_result_t mraa_io_close(mraa_io_descriptor* desc)
{
    return MRAA_SUCCESS;
}

void print_descriptor(const mraa_io_descriptor* desc)
{
    printf("mraa_io_descriptor: %p\n", desc);
    if (desc == NULL)
        return;

    printf("AIO count: %d\n", desc->n_aio);
    for (int i = 0; i < desc->n_aio; i++)
        printf("  %d mraa_aio_context: %p\n", i, desc->aios[i]);

    printf("GPIO count: %d\n", desc->n_gpio);
    for (int i = 0; i < desc->n_gpio; i++)
        printf("  %d mraa_gpio_context: %p\n", i, desc->gpios[i]);

    printf("I2C count: %d\n", desc->n_i2c);
    for (int i = 0; i < desc->n_i2c; i++)
        printf("  %d mraa_i2c_context: %p\n", i, desc->i2cs[i]);

    printf("IIO count: %d\n", desc->n_iio);
    for (int i = 0; i < desc->n_iio; i++)
        printf("  %d mraa_iio_context: %p\n", i, desc->iios[i]);

    printf("PWM count: %d\n", desc->n_pwm);
    for (int i = 0; i < desc->n_pwm; i++)
        printf("  %d mraa_pwm_context: %p\n", i, desc->pwms[i]);

    printf("SPI count: %d\n", desc->n_spi);
    for (int i = 0; i < desc->n_spi; i++)
        printf("  %d mraa_spi_context: %p\n", i, desc->spis[i]);

    printf("UART count: %d\n", desc->n_uart);
    for (int i = 0; i < desc->n_uart; i++)
        printf("mraa_uart_context: %p\n", desc->uarts[i]);

    printf("UART_OW count: %d\n", desc->n_uart_ow);
    for (int i = 0; i < desc->n_uart_ow; i++)
        printf("mraa_uart_ow_context: %p\n", desc->uart_ows[i]);

}
