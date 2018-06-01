/*
 * Author: Henry Bruce <henry.bruce@intel.com>
 * Copyright (c) 2015 Intel Corporation.
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

#include "ftdi_ft4222.hpp"
#include "common.h"
#include "ftd2xx.h"
#include "libft4222.h"
#include "linux/i2c-dev.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <vector>
#include <sys/time.h>
#include <time.h>

#define I2CM_ERROR(status) (((status)&0x02) != 0)
#define PCA9672_ADDR 0x20
#define PCA9555_ADDR 0x27
#define PCA9555_INPUT_REG 0
#define PCA9555_OUTPUT_REG 2
#define PCA9555_POLARITY_REG 4
#define PCA9555_DIRECTION_REG 6
#define PCA9545_ADDR 0x70
#define PCA9672_PINS 8
#define PCA9555_PINS 16
#define PCA9545_BUSSES 4
#define GPIO_PORT_IO_RESET GPIO_PORT2
#define GPIO_PORT_IO_INT GPIO_PORT3
#define MAX_IO_EXPANDER_PINS PCA9555_PINS

typedef enum
{ IO_EXP_NONE, IO_EXP_PCA9672, IO_EXP_PCA9555 } ft4222_io_exp_type;

typedef enum
{
    GPIO_TYPE_BUILTIN,
    GPIO_TYPE_PCA9672,
    GPIO_TYPE_PCA9555,
    GPIO_TYPE_UNKNOWN = 99
} ft4222_gpio_type;

static pthread_mutex_t ft4222_lock;
static FT_HANDLE ftHandleGpio = (FT_HANDLE)NULL; // GPIO Handle
static FT_HANDLE ftHandleI2c = (FT_HANDLE)NULL;  // I2C/SPI Handle
static FT_HANDLE ftHandleSpi = (FT_HANDLE)NULL;  // I2C/SPI Handle

static GPIO_Dir pinDirection[] =
{GPIO_OUTPUT, GPIO_OUTPUT, GPIO_OUTPUT,
    GPIO_OUTPUT};
static uint8_t pca9672DirectionMask = 0;
static uint16_t pca9555OutputValue = 0;
static uint16_t pca9555DirectionValue = 0;
static int bus_speed = 400;
const int gpioPinsPerFt4222 = 4;
static int currentI2cBus = 0;
static ft4222_io_exp_type gpio_expander_chip;

static void mraa_ftdi_ft4222_sleep_ms(unsigned long mseconds)
{
    struct timespec sleepTime;
    // Number of seconds
    sleepTime.tv_sec = mseconds / 1000;
    // Convert fractional seconds to nanoseconds
    sleepTime.tv_nsec = (mseconds % 1000) * 1000000;
    // Iterate nanosleep in a loop until the total sleep time is the original
    // value of the seconds parameter
    while ((nanosleep(&sleepTime, &sleepTime) != 0) && (errno == EINTR));
}

mraa_result_t mraa_ftdi_ft4222_init()
{
    DWORD numDevs = 0;

    if (FT_CreateDeviceInfoList(&numDevs) != FT_OK)
    {
        syslog(LOG_ERR, "FT_CreateDeviceInfoList failed\n");
        return MRAA_ERROR_UNSPECIFIED;
    }
    syslog(LOG_NOTICE, "FT_GetDeviceInfoList returned %d devices\n", numDevs);

    FT_DEVICE_LIST_INFO_NODE devInfo[numDevs];
    if (FT_GetDeviceInfoList(devInfo, &numDevs) != FT_OK)
    {
        syslog(LOG_ERR, "FT_GetDeviceInfoList failed\n");
        return MRAA_ERROR_UNSPECIFIED;
    }

    int first_4222_ndx = -1;
    int ftdi_mode = -1;
    /* Look for FT_DEVICE_4222H_0 devices. Print out all devices found. */
    for (DWORD i = 0; i < numDevs; i++)
    {
        /* Log info for debugging */
        syslog(LOG_NOTICE, "  FTDI ndx: %02d id: 0x%08x %s description: '%s'\n", i,
                devInfo[i].ID,
                (devInfo[i].Flags & 0x2) ? "High-speed USB" : "Full-speed USB",
                devInfo[i].Description);

        /* FTDI_4222 mode 3 provides 2 devices */
        if ((first_4222_ndx == -1) && (devInfo[i].Type == FT_DEVICE_4222H_0) &&
                ((i + 1 < numDevs) && (devInfo[i].ID == devInfo[i + 1].ID)))
        {
            first_4222_ndx = i;
            ftdi_mode = 3;
        }
        /* FTDI_4222 mode 0 provides 1 device */
        else if ((first_4222_ndx == -1) && (devInfo[i].Type == FT_DEVICE_4222H_0))
        {
            first_4222_ndx = i;
            ftdi_mode = 0;
        }
    }

    /* Was a usable 4222 found? */
    if (first_4222_ndx == -1)
    {
        syslog(LOG_ERR, "No FT4222 (mode 0 or 3) devices found.\n");
        return MRAA_ERROR_UNSPECIFIED;
    }

    syslog(LOG_NOTICE, "FTDI 4222 device found at ndx: %02d, mode %d\n",
            first_4222_ndx, ftdi_mode);

    /* Both modes provide a SPI/I2C at the first ndx */
    syslog(LOG_NOTICE, "FTDI ndx: %02d initializing as SPI/I2C\n",
            first_4222_ndx);

    /* Setup I2c */
    DWORD locationIdI2c = devInfo[first_4222_ndx].LocId;
    if (locationIdI2c == 0)
    {
        syslog(LOG_ERR, "No I2C controller for FTDI_4222 device\n");
        return MRAA_ERROR_UNSPECIFIED;
    }

    if (FT_OpenEx((PVOID)(uintptr_t)locationIdI2c, FT_OPEN_BY_LOCATION,
                &ftHandleI2c) != FT_OK)
    {
        syslog(LOG_NOTICE,
                "FTDI ndx: %02d initializing as SPI/I2C device - FAILED to open\n",
                first_4222_ndx + 1);
        return MRAA_ERROR_UNSPECIFIED;
    }

    // Tell the FT4222 to be an I2C Master by default on init.
    FT4222_STATUS ft4222Status = FT4222_I2CMaster_Init(ftHandleI2c, bus_speed);
    if (FT4222_OK != ft4222Status)
    {
        syslog(LOG_NOTICE,
                "FTDI ndx: %02d initializing as SPI/I2C device - FAILED to "
                "initialize\n",
                first_4222_ndx + 1);
        return MRAA_ERROR_UNSPECIFIED;
    }

    if (FT4222_I2CMaster_Reset(ftHandleI2c) != ft4222Status)
    {
        syslog(LOG_NOTICE,
                "FTDI ndx: %02d initializing as SPI/I2C device - FAILED to reset\n",
                first_4222_ndx + 1);
        return MRAA_ERROR_UNSPECIFIED;
    }

    /* Mode 3 adds 1 GPIO device, setup GPIO */
    if (ftdi_mode == 3)
    {
        syslog(LOG_NOTICE, "FTDI ndx: %02d initializing as GPIO device\n",
                first_4222_ndx + 1);

        DWORD locationIdGpio = devInfo[first_4222_ndx + 1].LocId;
        if (locationIdGpio == 0)
        {
            syslog(LOG_ERR, "No GPIO controller for FTDI_4222 device\n");
            return MRAA_ERROR_UNSPECIFIED;
        }

        if (FT_OpenEx((PVOID)(uintptr_t)locationIdGpio, FT_OPEN_BY_LOCATION,
                    &ftHandleGpio) != FT_OK)
        {
            syslog(LOG_NOTICE,
                    "FTDI ndx: %02d initializing as GPIO device - FAILED to open\n",
                    first_4222_ndx + 1);
            return MRAA_ERROR_UNSPECIFIED;
        }

        FT4222_SetSuspendOut(ftHandleGpio, 0);
        FT4222_SetWakeUpInterrupt(ftHandleGpio, 0);
        if (FT4222_GPIO_Init(ftHandleGpio, pinDirection) != FT4222_OK)
        {
            syslog(
                    LOG_NOTICE,
                    "FTDI ndx: %02d initializing as GPIO device - FAILED to initialize\n",
                    first_4222_ndx + 1);
            return MRAA_ERROR_UNSPECIFIED;
        }
    }

    syslog(LOG_NOTICE, "mraa_ftdi_ft4222_init completed successfully\n");
    return MRAA_SUCCESS;
}

mraa_result_t mraa_ftdi_ft4222_get_version(unsigned int *versionChip,
        unsigned int *versionLib)
{
    if (ftHandleI2c != NULL)
    {
        FT4222_Version ft4222Version;
        FT4222_STATUS ft4222Status = FT4222_GetVersion(ftHandleI2c, &ft4222Version);
        if (FT4222_OK == ft4222Status)
        {
            *versionChip = (unsigned int)ft4222Version.chipVersion;
            *versionLib = (unsigned int)ft4222Version.dllVersion;
            syslog(LOG_NOTICE, "FT4222_GetVersion %08X %08X\n", *versionChip,
                    *versionLib);
            return MRAA_SUCCESS;
        } else
        {
            syslog(LOG_ERR, "FT4222_GetVersion failed (error %d)\n",
                    (int)ft4222Status);
            return MRAA_ERROR_NO_RESOURCES;
        }
    } else
    {
        syslog(LOG_ERR, "Bad FT4222 handle\n");
        return MRAA_ERROR_INVALID_HANDLE;
    }
}

static int mraa_ftdi_ft4222_i2c_read_internal(FT_HANDLE handle, uint8_t addr,
        uint8_t *data, int length)
{
    uint16 bytesRead = 0;
    uint8 controllerStatus;

    /* If a read fails, check the I2C controller status, reset the controller,
     * return 0? */
    if (FT4222_I2CMaster_Read(handle, addr, data, length, &bytesRead) !=
            FT4222_OK)
    {
        FT4222_I2CMaster_GetStatus(ftHandleI2c, &controllerStatus);

        syslog(LOG_ERR, "FT4222_I2CMaster_Read failed for address %#02x. Code %d",
                addr, controllerStatus);
        FT4222_I2CMaster_Reset(handle);
        bytesRead = 0;
    }

    return bytesRead;
}

static int mraa_ftdi_ft4222_i2c_write_internal(FT_HANDLE handle, uint8_t addr,
        const uint8_t *data,
        int bytesToWrite)
{
    uint16 bytesWritten = 0;
    uint8 controllerStatus;

    /* If a write fails, check the I2C controller status, reset the controller,
     * return 0? */
    if (FT4222_I2CMaster_Write(handle, addr, (uint8_t *)data, bytesToWrite,
                &bytesWritten) != FT4222_OK)
    {
        FT4222_I2CMaster_GetStatus(ftHandleI2c, &controllerStatus);

        syslog(LOG_ERR, "FT4222_I2CMaster_Write failed address %#02x. Code %d\n",
                addr, controllerStatus);
        FT4222_I2CMaster_Reset(handle);
        bytesWritten = 0;
    }

    if (bytesWritten != bytesToWrite)
        syslog(LOG_ERR, "FT4222_I2CMaster_Write wrote %u of %u bytes.\n",
                bytesWritten, bytesToWrite);

    return bytesWritten;
}

// Function detects known I2C I/O expanders and returns the number of GPIO pins
// on expander
static int mraa_ftdi_ft4222_detect_io_expander()
{
    uint8_t data;
    if (mraa_ftdi_ft4222_i2c_read_internal(ftHandleI2c, PCA9672_ADDR, &data, 1) ==
            1)
    {
        gpio_expander_chip = IO_EXP_PCA9672;
        return PCA9672_PINS;
    } else
    {
        uint8_t reg = PCA9555_INPUT_REG;
        mraa_ftdi_ft4222_i2c_write_internal(ftHandleI2c, PCA9555_ADDR, &reg, 1);
        if (mraa_ftdi_ft4222_i2c_read_internal(ftHandleI2c, PCA9555_ADDR, &data,
                    1) == 1)
        {
            gpio_expander_chip = IO_EXP_PCA9555;
            reg = PCA9555_OUTPUT_REG;
            mraa_ftdi_ft4222_i2c_write_internal(ftHandleI2c, PCA9555_ADDR, &reg, 1);
            mraa_ftdi_ft4222_i2c_read_internal(ftHandleI2c, PCA9555_ADDR,
                    (uint8_t *)&pca9555OutputValue, 2);
            reg = PCA9555_DIRECTION_REG;
            mraa_ftdi_ft4222_i2c_write_internal(ftHandleI2c, PCA9555_ADDR, &reg, 1);
            mraa_ftdi_ft4222_i2c_read_internal(ftHandleI2c, PCA9555_ADDR,
                    (uint8_t *)&pca9555DirectionValue, 2);
            return PCA9555_PINS;
        }
    }
    gpio_expander_chip = IO_EXP_NONE;
    return 0;
}

static ft4222_gpio_type mraa_ftdi_ft4222_get_gpio_type(int pin)
{
    if (pin < gpioPinsPerFt4222)
    {
        return GPIO_TYPE_BUILTIN;
    } else
        switch (gpio_expander_chip)
        {
            case IO_EXP_PCA9672:
                return GPIO_TYPE_PCA9672;
            case GPIO_TYPE_PCA9555:
                return GPIO_TYPE_PCA9555;
            default:
                return GPIO_TYPE_UNKNOWN;
        }
}

static mraa_result_t ftdi_ft4222_set_internal_gpio_dir(int pin,
        GPIO_Dir direction)
{
    pinDirection[pin] = direction;
    if (FT4222_GPIO_Init(ftHandleGpio, pinDirection) != FT4222_OK)
        return MRAA_ERROR_UNSPECIFIED;
    else
        return MRAA_SUCCESS;
}

static mraa_result_t mraa_ftdi_ft4222_gpio_set_pca9672_dir(int pin, mraa_gpio_dir_t dir)
{
    uint8_t mask = 1 << pin;
    switch (dir)
    {
        case MRAA_GPIO_IN:
            {
                pca9672DirectionMask |= mask;
                int bytes_written = mraa_ftdi_ft4222_i2c_write_internal(
                        ftHandleI2c, PCA9672_ADDR, &pca9672DirectionMask, 1);
                return bytes_written == 1 ? MRAA_SUCCESS : MRAA_ERROR_UNSPECIFIED;
            }
        case MRAA_GPIO_OUT:
            {
                pca9672DirectionMask &= (~mask);
                return MRAA_SUCCESS;
            }
        default:
            return MRAA_ERROR_UNSPECIFIED;
    }
}

static mraa_result_t mraa_ftdi_ft4222_gpio_set_pca9555_dir(int pin, mraa_gpio_dir_t dir)
{
    uint16_t mask = 1 << pin;
    switch (dir)
    {
        case MRAA_GPIO_IN:
            pca9555DirectionValue |= mask;
            break;
        case MRAA_GPIO_OUT:
            pca9555DirectionValue &= (~mask);
            break;
        default:
            return MRAA_ERROR_UNSPECIFIED;
    }
    uint8_t buf[3];
    buf[0] = PCA9555_DIRECTION_REG;
    buf[1] = (uint8_t)(pca9555DirectionValue & 0xFF);
    buf[2] = (uint8_t)(pca9555DirectionValue >> 8);
    pthread_mutex_lock(&ft4222_lock);
    int bytes_written = mraa_ftdi_ft4222_i2c_write_internal(
            ftHandleI2c, PCA9555_ADDR, buf, sizeof(buf));
    pthread_mutex_unlock(&ft4222_lock);
    return bytes_written == sizeof(buf) ? MRAA_SUCCESS : MRAA_ERROR_UNSPECIFIED;
}

static mraa_result_t ftdi_ft4222_set_internal_gpio_trigger(int pin, GPIO_Trigger trigger)
{
    FT4222_STATUS ft4222Status = FT4222_GPIO_SetInputTrigger(
            ftHandleGpio, static_cast<GPIO_Port>(pin), trigger);
    if (ft4222Status == FT4222_OK)
        return MRAA_SUCCESS;
    else
        return MRAA_ERROR_UNSPECIFIED;
}

// Function detects known I2C switches and returns the number of busses.
// On startup switch is disabled so default bus will be integrated i2c bus.
static int mraa_ftdi_ft4222_detect_i2c_switch()
{
    uint8_t data;
    if (mraa_ftdi_ft4222_i2c_read_internal(ftHandleI2c, PCA9545_ADDR, &data, 1) ==
            1)
    {
        data = 0;
        return mraa_ftdi_ft4222_i2c_write_internal(ftHandleI2c, PCA9545_ADDR, &data,
                1) == 1
            ? PCA9545_BUSSES
            : 0;
    }
    return 0;
}

static mraa_result_t mraa_ftdi_ft4222_i2c_select_bus(int bus)
{
    if (bus > 0 && bus != currentI2cBus)
    {
        syslog(LOG_NOTICE, "mraa_ftdi_ft4222_i2c_select_bus switching to bus %d",
                bus);
        uint8_t data;
        if (bus == 0)
            data = 0;
        else
            data = 1 << (bus - 1);
        if (mraa_ftdi_ft4222_i2c_write_internal(ftHandleI2c, PCA9545_ADDR, &data,
                    1) == 1)
            currentI2cBus = bus;
        else
            return MRAA_ERROR_UNSPECIFIED;
    }
    return MRAA_SUCCESS;
}

static int mraa_ftdi_ft4222_i2c_context_read(mraa_i2c_context dev,
        uint8_t *data, int length)
{
    int bytes_read = 0;
    if (mraa_ftdi_ft4222_i2c_select_bus(dev->busnum) == MRAA_SUCCESS)
        bytes_read = mraa_ftdi_ft4222_i2c_read_internal(dev->handle, dev->addr,
                data, length);
    return bytes_read;
}

static int mraa_ftdi_ft4222_i2c_context_write(mraa_i2c_context dev,
        uint8_t *data, int length)
{
    int bytes_written = 0;
    if (mraa_ftdi_ft4222_i2c_select_bus(dev->busnum) == MRAA_SUCCESS)
        bytes_written = mraa_ftdi_ft4222_i2c_write_internal(dev->handle, dev->addr,
                data, length);
    return bytes_written;
}

/******************* I2C functions *******************/

static mraa_result_t mraa_ftdi_ft4222_i2c_init_bus_replace(mraa_i2c_context dev)
{
    // Tell the FT4222 to be an I2C Master.
    FT4222_STATUS ft4222Status = FT4222_I2CMaster_Init(ftHandleI2c, bus_speed);
    if (FT4222_OK != ft4222Status)
    {
        syslog(LOG_ERR, "FT4222_I2CMaster_Init failed (error %d)!\n", ft4222Status);
        return MRAA_ERROR_NO_RESOURCES;
    }

    // Reset the I2CM registers to a known state.
    ft4222Status = FT4222_I2CMaster_Reset(ftHandleI2c);
    if (FT4222_OK != ft4222Status)
    {
        syslog(LOG_ERR, "FT4222_I2CMaster_Reset failed (error %d)!\n",
                ft4222Status);
        return MRAA_ERROR_NO_RESOURCES;
    }

    syslog(LOG_NOTICE,
            "I2C interface enabled GPIO0 and GPIO1 will be unavailable.\n");
    dev->handle = ftHandleI2c;
    // We don't use file descriptors
    dev->fh = -1;

    // Advertise minimal i2c support as per
    // https://www.kernel.org/doc/Documentation/i2c/functionality
    dev->funcs = I2C_FUNC_I2C;
    return MRAA_SUCCESS;
}

static mraa_result_t mraa_ftdi_ft4222_i2c_frequency(mraa_i2c_context dev,
        mraa_i2c_mode_t mode)
{
    switch (mode)
    {
        case MRAA_I2C_STD: /**< up to 100Khz */
            bus_speed = 100;
            break;
        case MRAA_I2C_FAST: /**< up to 400Khz */
            bus_speed = 400;
            break;
        case MRAA_I2C_HIGH: /**< up to 3.4Mhz */
            bus_speed = 3400;
            break;
    }
    return FT4222_I2CMaster_Init(ftHandleI2c, bus_speed) == FT4222_OK
        ? MRAA_SUCCESS
        : MRAA_ERROR_NO_RESOURCES;
}

static mraa_result_t mraa_ftdi_ft4222_i2c_address(mraa_i2c_context dev,
        uint8_t addr)
{
    dev->addr = (int)addr;
    return MRAA_SUCCESS;
}

static int mraa_ftdi_ft4222_i2c_read(mraa_i2c_context dev, uint8_t *data,
        int length)
{
    pthread_mutex_lock(&ft4222_lock);
    int bytes_read =
        mraa_ftdi_ft4222_i2c_read_internal(dev->handle, dev->addr, data, length);
    pthread_mutex_unlock(&ft4222_lock);
    return bytes_read;
}

static int mraa_ftdi_ft4222_i2c_read_byte(mraa_i2c_context dev)
{
    uint8_t data;
    pthread_mutex_lock(&ft4222_lock);
    int bytes_read = mraa_ftdi_ft4222_i2c_context_read(dev, &data, 1);
    pthread_mutex_unlock(&ft4222_lock);
    return bytes_read == 1 ? data : -1;
}

static int mraa_ftdi_ft4222_i2c_read_byte_data(mraa_i2c_context dev,
        uint8_t command)
{
    uint8_t data;
    int bytes_read = 0;
    pthread_mutex_lock(&ft4222_lock);
    uint16 bytesWritten = mraa_ftdi_ft4222_i2c_context_write(dev, &command, 1);
    if (bytesWritten == 1)
        bytes_read = mraa_ftdi_ft4222_i2c_context_read(dev, &data, 1);
    pthread_mutex_unlock(&ft4222_lock);
    if (bytes_read == 1)
    {
        return (int)data;
    }
    return -1;
}

static int mraa_ftdi_ft4222_i2c_read_word_data(mraa_i2c_context dev,
        uint8_t command)
{
    union
    {
        uint8_t bytes[2];
        uint16_t word;
    } buf;
    int bytes_read = 0;

    pthread_mutex_lock(&ft4222_lock);
    int bytes_written = mraa_ftdi_ft4222_i2c_context_write(dev, &command, 1);
    if (bytes_written == 1)
        bytes_read = mraa_ftdi_ft4222_i2c_context_read(dev, buf.bytes, 2);
    pthread_mutex_unlock(&ft4222_lock);

    if (bytes_read == 2)
    {
        return (int)buf.word;
    }

    return -1;
}

static int mraa_ftdi_ft4222_i2c_read_bytes_data(mraa_i2c_context dev,
        uint8_t command, uint8_t *data,
        int length)
{
    int bytes_read = 0;
    pthread_mutex_lock(&ft4222_lock);
    int bytes_written = mraa_ftdi_ft4222_i2c_context_write(dev, &command, 1);
    if (bytes_written == 1)
        bytes_read = mraa_ftdi_ft4222_i2c_context_read(dev, data, length);
    pthread_mutex_unlock(&ft4222_lock);
    return bytes_read;
}

static mraa_result_t mraa_ftdi_ft4222_i2c_write(mraa_i2c_context dev,
        const uint8_t *data,
        int bytesToWrite)
{
    pthread_mutex_lock(&ft4222_lock);
    uint16 bytesWritten =
        mraa_ftdi_ft4222_i2c_context_write(dev, (uint8_t *)data, bytesToWrite);
    pthread_mutex_unlock(&ft4222_lock);
    return bytesToWrite == bytesWritten ? MRAA_SUCCESS
        : MRAA_ERROR_INVALID_HANDLE;
}

static mraa_result_t mraa_ftdi_ft4222_i2c_write_byte(mraa_i2c_context dev,
        uint8_t data)
{
    mraa_result_t status = mraa_ftdi_ft4222_i2c_write(dev, &data, 1);
    return status;
}

static mraa_result_t
mraa_ftdi_ft4222_i2c_write_byte_data(mraa_i2c_context dev, const uint8_t data,
        const uint8_t command)
{
    uint8_t buf[2];
    buf[0] = command;
    buf[1] = data;
    mraa_result_t status = mraa_ftdi_ft4222_i2c_write(dev, buf, 2);
    return status;
}

static mraa_result_t
mraa_ftdi_ft4222_i2c_write_word_data(mraa_i2c_context dev, const uint16_t data,
        const uint8_t command)
{
    uint8_t buf[3];
    buf[0] = command;
    buf[1] = (uint8_t)data;
    buf[2] = (uint8_t)(data >> 8);
    mraa_result_t status = mraa_ftdi_ft4222_i2c_write(dev, buf, 3);
    return status;
}

static mraa_result_t mraa_ftdi_ft4222_i2c_stop(mraa_i2c_context dev)
{
    return MRAA_SUCCESS;
}

/******************* GPIO functions *******************/

static mraa_result_t
mraa_ftdi_ft4222_gpio_init_internal_replace(mraa_gpio_context dev, int pin)
{
    dev->phy_pin = (pin < gpioPinsPerFt4222) ? pin : pin - gpioPinsPerFt4222;
    if (pin < 2)
    {
        syslog(LOG_NOTICE, "Closing I2C interface to enable GPIO%d\n", pin);

        /* Replace with call to SPI init when SPI is fully implemented */
        FT4222_STATUS ft4222Status =
            FT4222_SPIMaster_Init(ftHandleSpi, SPI_IO_SINGLE, CLK_DIV_4,
                    CLK_IDLE_HIGH, CLK_LEADING, 0x01);
        if (FT4222_OK != ft4222Status)
        {
            syslog(LOG_ERR,
                    "Failed to close I2C interface and start SPI (error %d)!\n",
                    ft4222Status);
            return MRAA_ERROR_NO_RESOURCES;
        }
    }
    return MRAA_SUCCESS;
}

static mraa_result_t mraa_ftdi_ft4222_gpio_mode_replace(mraa_gpio_context dev,
        mraa_gpio_mode_t mode)
{
    return MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
}

static mraa_result_t
mraa_ftdi_ft4222_gpio_edge_mode_replace(mraa_gpio_context dev,
        mraa_gpio_edge_t mode)
{
    switch (mraa_ftdi_ft4222_get_gpio_type(dev->pin))
    {
        case GPIO_TYPE_BUILTIN:
            switch (mode)
            {
                case MRAA_GPIO_EDGE_NONE:
                    return MRAA_SUCCESS;
                case MRAA_GPIO_EDGE_BOTH:
                    return ftdi_ft4222_set_internal_gpio_trigger(
                            dev->pin, static_cast<GPIO_Trigger>(GPIO_TRIGGER_RISING |
                                GPIO_TRIGGER_FALLING));
                case MRAA_GPIO_EDGE_RISING:
                    return ftdi_ft4222_set_internal_gpio_trigger(dev->pin,
                            GPIO_TRIGGER_RISING);
                case MRAA_GPIO_EDGE_FALLING:
                    return ftdi_ft4222_set_internal_gpio_trigger(dev->pin,
                            GPIO_TRIGGER_FALLING);
                default:
                    return MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
            }
            break;
        case GPIO_TYPE_PCA9672:
        case GPIO_TYPE_PCA9555:
            return MRAA_SUCCESS;
        default:
            return MRAA_ERROR_INVALID_RESOURCE;
    }
}

static mraa_result_t mraa_ftdi_ft4222_i2c_read_io_expander(uint16_t *value)
{
    int bytes_read = 0;
    uint8_t reg = PCA9555_INPUT_REG;
    pthread_mutex_lock(&ft4222_lock);
    switch (gpio_expander_chip)
    {
        case IO_EXP_PCA9672:
            bytes_read = mraa_ftdi_ft4222_i2c_read_internal(ftHandleI2c, PCA9672_ADDR,
                    (uint8_t *)value, 1);
            break;
        case GPIO_TYPE_PCA9555:
            if (mraa_ftdi_ft4222_i2c_write_internal(ftHandleI2c, PCA9555_ADDR, &reg,
                        1) == 1)
                bytes_read = mraa_ftdi_ft4222_i2c_read_internal(ftHandleI2c, PCA9555_ADDR,
                        (uint8_t *)value, 2);
            break;
        default:;
    }
    pthread_mutex_unlock(&ft4222_lock);
    return bytes_read > 0 ? MRAA_SUCCESS : MRAA_ERROR_UNSPECIFIED;
}

static int mraa_ftdi_ft4222_gpio_read_replace(mraa_gpio_context dev)
{
    switch (mraa_ftdi_ft4222_get_gpio_type(dev->pin))
    {
        case GPIO_TYPE_BUILTIN:
            {
                BOOL value;
                FT4222_STATUS ft4222Status = FT4222_GPIO_Read(
                        ftHandleGpio, static_cast<GPIO_Port>(dev->phy_pin), &value);
                if (FT4222_OK != ft4222Status)
                {
                    syslog(LOG_ERR, "FT4222_GPIO_Read failed (error %d)!\n", ft4222Status);
                    return -1;
                }
                return value;
            }
        case GPIO_TYPE_PCA9672:
        case GPIO_TYPE_PCA9555:
            {
                uint16_t mask = 1 << dev->phy_pin;
                uint16_t value;
                mraa_result_t res = mraa_ftdi_ft4222_i2c_read_io_expander(&value);
                return res == MRAA_SUCCESS ? (value & mask) == mask : -1;
            }
        default:
            return -1;
    }
}

static mraa_result_t mraa_ftdi_ft4222_gpio_write_replace(mraa_gpio_context dev,
        int write_value)
{
    switch (mraa_ftdi_ft4222_get_gpio_type(dev->pin))
    {
        case GPIO_TYPE_BUILTIN:
            {
                FT4222_STATUS ft4222Status = FT4222_GPIO_Write(
                        ftHandleGpio, static_cast<GPIO_Port>(dev->phy_pin), write_value);
                if (FT4222_OK != ft4222Status)
                {
                    syslog(LOG_ERR, "FT4222_GPIO_Write failed (error %d)!\n", ft4222Status);
                    return MRAA_ERROR_UNSPECIFIED;
                }
                return MRAA_SUCCESS;
            }
        case GPIO_TYPE_PCA9672:
            {
                uint8_t mask = 1 << dev->phy_pin;
                uint8_t value;
                int bytes_written = 0;
                pthread_mutex_lock(&ft4222_lock);
                int bytes_read = mraa_ftdi_ft4222_i2c_read_internal(
                        ftHandleI2c, PCA9672_ADDR, &value, 1);
                if (bytes_read == 1)
                {
                    if (write_value == 1)
                        value = value | mask | pca9672DirectionMask;
                    else
                        value &= (~mask);
                    bytes_written = mraa_ftdi_ft4222_i2c_write_internal(
                            ftHandleI2c, PCA9672_ADDR, &value, 1);
                }
                pthread_mutex_unlock(&ft4222_lock);
                return bytes_written == 1 ? MRAA_SUCCESS : MRAA_ERROR_UNSPECIFIED;
            }
        case GPIO_TYPE_PCA9555:
            {
                uint16_t mask = 1 << dev->phy_pin;
                if (write_value)
                    pca9555OutputValue |= mask;
                else
                    pca9555OutputValue &= (~mask);
                uint8_t buf[3];
                buf[0] = PCA9555_OUTPUT_REG;
                buf[1] = (uint8_t)(pca9555OutputValue & 0xFF);
                buf[2] = (uint8_t)(pca9555OutputValue >> 8);
                pthread_mutex_lock(&ft4222_lock);
                int bytes_written = mraa_ftdi_ft4222_i2c_write_internal(
                        ftHandleI2c, PCA9555_ADDR, buf, sizeof(buf));
                pthread_mutex_unlock(&ft4222_lock);
                return bytes_written == sizeof(buf) ? MRAA_SUCCESS : MRAA_ERROR_UNSPECIFIED;
            }
        default:
            return MRAA_ERROR_INVALID_RESOURCE;
    }
}

static mraa_result_t mraa_ftdi_ft4222_gpio_dir_replace(mraa_gpio_context dev,
        mraa_gpio_dir_t dir)
{
    switch (mraa_ftdi_ft4222_get_gpio_type(dev->pin))
    {
        case GPIO_TYPE_BUILTIN:
            switch (dir)
            {
                case MRAA_GPIO_IN:
                    return ftdi_ft4222_set_internal_gpio_dir(dev->phy_pin, GPIO_INPUT);
                case MRAA_GPIO_OUT:
                    return ftdi_ft4222_set_internal_gpio_dir(dev->phy_pin, GPIO_OUTPUT);
                case MRAA_GPIO_OUT_HIGH:
                    if (ftdi_ft4222_set_internal_gpio_dir(dev->phy_pin, GPIO_OUTPUT) !=
                            MRAA_SUCCESS)
                        return MRAA_ERROR_UNSPECIFIED;
                    return mraa_ftdi_ft4222_gpio_write_replace(dev, 1);
                case MRAA_GPIO_OUT_LOW:
                    if (ftdi_ft4222_set_internal_gpio_dir(dev->phy_pin, GPIO_OUTPUT) !=
                            MRAA_SUCCESS)
                        return MRAA_ERROR_UNSPECIFIED;
                    return mraa_ftdi_ft4222_gpio_write_replace(dev, 0);
                default:
                    return MRAA_ERROR_INVALID_PARAMETER;
            }
        case GPIO_TYPE_PCA9672:
            switch (dir)
            {
                case MRAA_GPIO_IN:
                case MRAA_GPIO_OUT:
                    return mraa_ftdi_ft4222_gpio_set_pca9672_dir(dev->phy_pin, dir);
                case MRAA_GPIO_OUT_HIGH:
                    if (mraa_ftdi_ft4222_gpio_set_pca9672_dir(dev->phy_pin, dir) !=
                            MRAA_SUCCESS)
                        return MRAA_ERROR_UNSPECIFIED;
                    return mraa_ftdi_ft4222_gpio_write_replace(dev, 1);
                case MRAA_GPIO_OUT_LOW:
                    if (mraa_ftdi_ft4222_gpio_set_pca9672_dir(dev->phy_pin, dir) !=
                            MRAA_SUCCESS)
                        return MRAA_ERROR_UNSPECIFIED;
                    return mraa_ftdi_ft4222_gpio_write_replace(dev, 0);
                default:
                    return MRAA_ERROR_INVALID_PARAMETER;
            }
        case GPIO_TYPE_PCA9555:
            switch (dir)
            {
                case MRAA_GPIO_IN:
                case MRAA_GPIO_OUT:
                    return mraa_ftdi_ft4222_gpio_set_pca9555_dir(dev->phy_pin, dir);
                case MRAA_GPIO_OUT_HIGH:
                    if (mraa_ftdi_ft4222_gpio_set_pca9555_dir(dev->phy_pin, dir) !=
                            MRAA_SUCCESS)
                        return MRAA_ERROR_UNSPECIFIED;
                    return mraa_ftdi_ft4222_gpio_write_replace(dev, 1);
                case MRAA_GPIO_OUT_LOW:
                    if (mraa_ftdi_ft4222_gpio_set_pca9555_dir(dev->phy_pin, dir) !=
                            MRAA_SUCCESS)
                        return MRAA_ERROR_UNSPECIFIED;
                    return mraa_ftdi_ft4222_gpio_write_replace(dev, 0);
                default:
                    return MRAA_ERROR_INVALID_PARAMETER;
            }
        default:
            return MRAA_ERROR_INVALID_RESOURCE;
    }
}

static mraa_boolean_t mraa_ftdi_ft4222_has_internal_gpio_triggered(int pin)
{
    uint16 num_events = 0;
    FT4222_GPIO_GetTriggerStatus(ftHandleGpio, static_cast<GPIO_Port>(pin),
            &num_events);
    if (num_events > 0)
    {
        int i;
        uint16 num_events_read;
        GPIO_Trigger event;
        for (i = 0; i < num_events; ++i)
            FT4222_GPIO_ReadTriggerQueue(ftHandleGpio, static_cast<GPIO_Port>(pin),
                    &event, 1, &num_events_read);
        return TRUE;
    } else
        return FALSE;
}

static struct
{
    pthread_t thread;
    pthread_mutex_t mutex;
    mraa_boolean_t should_stop;
    mraa_boolean_t is_interrupt_detected[MAX_IO_EXPANDER_PINS];
    int num_active_pins;
} gpio_monitor =
{0};

// INT pin of i2c PCA9672 GPIO expander is connected to FT4222 GPIO #3
// We use INT to detect any expander GPIO level change
static void *mraa_ftdi_ft4222_gpio_monitor(void *arg)
{
    uint16_t prev_value = 0;
    mraa_ftdi_ft4222_i2c_read_io_expander(&prev_value);
    while (!gpio_monitor.should_stop)
    {
        mraa_boolean_t gpio_activity_detected =
            mraa_ftdi_ft4222_has_internal_gpio_triggered(GPIO_PORT_IO_INT);
        if (gpio_activity_detected)
        {
            uint16_t value;
            if (mraa_ftdi_ft4222_i2c_read_io_expander(&value) == MRAA_SUCCESS)
            {
                uint16_t change_value = prev_value ^ value;
                int i;
                pthread_mutex_lock(&gpio_monitor.mutex);
                for (i = 0; i < MAX_IO_EXPANDER_PINS; ++i)
                {
                    uint16_t mask = 1 << i;
                    gpio_monitor.is_interrupt_detected[i] = (change_value & mask) ? 1 : 0;
                }
                pthread_mutex_unlock(&gpio_monitor.mutex);
                prev_value = value;
            }
        }
        mraa_ftdi_ft4222_sleep_ms(20);
    }
    return NULL;
}

static void mraa_ftdi_ft4222_gpio_monitor_add_pin(int pin)
{
    if (gpio_monitor.num_active_pins == 0)
    {
        pthread_mutex_init(&gpio_monitor.mutex, NULL);
        pthread_create(&gpio_monitor.thread, NULL, mraa_ftdi_ft4222_gpio_monitor,
                NULL);
    }
    pthread_mutex_lock(&gpio_monitor.mutex);
    gpio_monitor.num_active_pins++;
    pthread_mutex_unlock(&gpio_monitor.mutex);
}

static void mraa_ftdi_ft4222_gpio_monitor_remove_pin(int pin)
{
    pthread_mutex_lock(&gpio_monitor.mutex);
    gpio_monitor.num_active_pins--;
    if (gpio_monitor.num_active_pins == 0)
    {
        pthread_mutex_unlock(&gpio_monitor.mutex);
        gpio_monitor.should_stop = TRUE;
        pthread_join(gpio_monitor.thread, NULL);
        pthread_mutex_destroy(&gpio_monitor.mutex);
    } else
        pthread_mutex_unlock(&gpio_monitor.mutex);
}

static mraa_boolean_t
mraa_ftdi_ft4222_gpio_monitor_is_interrupt_detected(int pin)
{
    mraa_boolean_t is_interrupt_detected = FALSE;
    pthread_mutex_lock(&gpio_monitor.mutex);
    if (gpio_monitor.is_interrupt_detected[pin])
    {
        gpio_monitor.is_interrupt_detected[pin] = FALSE;
        is_interrupt_detected = TRUE;
    }
    pthread_mutex_unlock(&gpio_monitor.mutex);
    return is_interrupt_detected;
}

static mraa_result_t
mraa_ftdi_ft4222_gpio_interrupt_handler_init_replace(mraa_gpio_context dev)
{
    switch (mraa_ftdi_ft4222_get_gpio_type(dev->pin))
    {
        case GPIO_TYPE_BUILTIN:
            mraa_ftdi_ft4222_has_internal_gpio_triggered(dev->phy_pin);
            break;
        case GPIO_TYPE_PCA9672:
        case GPIO_TYPE_PCA9555:
            ftdi_ft4222_set_internal_gpio_dir(GPIO_PORT_IO_INT, GPIO_INPUT);
            ftdi_ft4222_set_internal_gpio_trigger(GPIO_PORT_IO_INT,
                    GPIO_TRIGGER_FALLING);
            mraa_ftdi_ft4222_has_internal_gpio_triggered(GPIO_PORT_IO_INT);
            mraa_ftdi_ft4222_gpio_monitor_add_pin(dev->phy_pin);
            break;
        default:
            return MRAA_ERROR_INVALID_RESOURCE;
    }
    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_ftdi_ft4222_gpio_wait_interrupt_replace(mraa_gpio_context dev)
{
    mraa_ftdi_ft4222_gpio_read_replace(dev);
    ft4222_gpio_type gpio_type = mraa_ftdi_ft4222_get_gpio_type(dev->pin);
    mraa_boolean_t interrupt_detected = FALSE;

    while (!dev->isr_thread_terminating && !interrupt_detected)
    {
        switch (gpio_type)
        {
            case GPIO_TYPE_BUILTIN:
                interrupt_detected =
                    mraa_ftdi_ft4222_has_internal_gpio_triggered(dev->phy_pin);
                break;
            case GPIO_TYPE_PCA9672:
            case GPIO_TYPE_PCA9555:
                interrupt_detected =
                    mraa_ftdi_ft4222_gpio_monitor_is_interrupt_detected(dev->phy_pin);
                break;
            default:;
        }
        if (!interrupt_detected)
            mraa_ftdi_ft4222_sleep_ms(20);
    }
    if (dev->isr_thread_terminating)
        mraa_ftdi_ft4222_gpio_monitor_remove_pin(dev->phy_pin);
    return MRAA_SUCCESS;
}

static void
mraa_ftdi_ft4222_populate_i2c_func_table(mraa_adv_func_t *func_table)
{
    func_table->i2c_init_bus_replace = &mraa_ftdi_ft4222_i2c_init_bus_replace;
    func_table->i2c_set_frequency_replace = &mraa_ftdi_ft4222_i2c_frequency;
    func_table->i2c_address_replace = &mraa_ftdi_ft4222_i2c_address;
    func_table->i2c_read_replace = &mraa_ftdi_ft4222_i2c_read;
    func_table->i2c_read_byte_replace = &mraa_ftdi_ft4222_i2c_read_byte;
    func_table->i2c_read_byte_data_replace = &mraa_ftdi_ft4222_i2c_read_byte_data;
    func_table->i2c_read_word_data_replace = &mraa_ftdi_ft4222_i2c_read_word_data;
    func_table->i2c_read_bytes_data_replace =
        &mraa_ftdi_ft4222_i2c_read_bytes_data;
    func_table->i2c_write_replace = &mraa_ftdi_ft4222_i2c_write;
    func_table->i2c_write_byte_replace = &mraa_ftdi_ft4222_i2c_write_byte;
    func_table->i2c_write_byte_data_replace =
        &mraa_ftdi_ft4222_i2c_write_byte_data;
    func_table->i2c_write_word_data_replace =
        &mraa_ftdi_ft4222_i2c_write_word_data;
    func_table->i2c_stop_replace = &mraa_ftdi_ft4222_i2c_stop;
}

static void
mraa_ftdi_ft4222_populate_gpio_func_table(mraa_adv_func_t *func_table)
{
    func_table->gpio_init_internal_replace =
        &mraa_ftdi_ft4222_gpio_init_internal_replace;
    func_table->gpio_mode_replace = &mraa_ftdi_ft4222_gpio_mode_replace;
    func_table->gpio_edge_mode_replace = &mraa_ftdi_ft4222_gpio_edge_mode_replace;
    func_table->gpio_dir_replace = &mraa_ftdi_ft4222_gpio_dir_replace;
    func_table->gpio_read_replace = &mraa_ftdi_ft4222_gpio_read_replace;
    func_table->gpio_write_replace = &mraa_ftdi_ft4222_gpio_write_replace;
    func_table->gpio_interrupt_handler_init_replace =
        &mraa_ftdi_ft4222_gpio_interrupt_handler_init_replace;
    func_table->gpio_wait_interrupt_replace =
        &mraa_ftdi_ft4222_gpio_wait_interrupt_replace;
}

/* Store mraa_board_t subplatform allocated by this library */
std::vector<mraa_board_t> sub_plats_4222;
char FTDI_4222_PLATFORM_NAME[] = "FTDI FT4222";
mraa_board_t *mraa_ftdi_ft4222()
{
    if (pthread_mutex_init(&ft4222_lock, NULL) != 0)
    {
        syslog(LOG_ERR, "Could not create mutex for FT4222 access");
    }

    /* Always allocate a new mraa_board_t */
    sub_plats_4222.push_back(mraa_board_t());
    /* Use a reference for readability  */
    mraa_board_t &plat = sub_plats_4222.back();

    int numI2cGpioExpanderPins = mraa_ftdi_ft4222_detect_io_expander();
    int pinIndex = 0;
    int numUsbGpio = gpioPinsPerFt4222 + numI2cGpioExpanderPins;
    int numI2cBusses = 1 + mraa_ftdi_ft4222_detect_i2c_switch();
    int numUsbPins =
        numUsbGpio +
        2 * (numI2cBusses - 1); // Add SDA and SCL for each i2c switch bus
    mraa_pincapabilities_t pinCapsI2c =
        (mraa_pincapabilities_t){1, 0, 0, 0, 0, 1, 0, 0};
    mraa_pincapabilities_t pinCapsI2cGpio =
        (mraa_pincapabilities_t){1, 1, 0, 0, 0, 1, 0, 0};
    mraa_pincapabilities_t pinCapsGpio =
        (mraa_pincapabilities_t){1, 1, 0, 0, 0, 0, 0, 0};

    plat.platform_name = FTDI_4222_PLATFORM_NAME;
    plat.platform_type = MRAA_FTDI_FT4222;
    plat.phy_pin_count = numUsbPins;
    plat.gpio_count = numUsbGpio;
    plat.pins =
        static_cast<mraa_pininfo_t *>(calloc(numUsbPins, sizeof(mraa_pininfo_t)));

    if (plat.pins == NULL)
    {
        sub_plats_4222.pop_back();
        return NULL;
    }

    int bus = 0;
    plat.i2c_bus_count = numI2cBusses;
    plat.def_i2c_bus = bus;
    plat.i2c_bus[bus].bus_id = bus;

    // I2c pins (these are virtual, entries are required to configure i2c layer)
    // We currently assume that GPIO 0/1 are reserved for i2c operation
    strncpy(plat.pins[pinIndex].name, "IGPIO0/SCL0", MRAA_PIN_NAME_SIZE);
    plat.pins[pinIndex].capabilities = pinCapsI2cGpio;
    plat.pins[pinIndex].gpio.pinmap = pinIndex;
    plat.pins[pinIndex].gpio.mux_total = 0;
    plat.pins[pinIndex].i2c.mux_total = 0;
    plat.i2c_bus[bus].scl = pinIndex;
    pinIndex++;
    strncpy(plat.pins[pinIndex].name, "IGPIO1/SDA0", MRAA_PIN_NAME_SIZE);
    plat.pins[pinIndex].capabilities = pinCapsI2cGpio;
    plat.pins[pinIndex].gpio.pinmap = pinIndex;
    plat.pins[pinIndex].gpio.mux_total = 0;
    plat.pins[pinIndex].i2c.mux_total = 0;
    plat.i2c_bus[bus].sda = pinIndex;
    pinIndex++;

    // FTDI4222 gpio
    strncpy(plat.pins[pinIndex].name, "INT-GPIO2", MRAA_PIN_NAME_SIZE);
    plat.pins[pinIndex].capabilities = pinCapsGpio;
    plat.pins[pinIndex].gpio.pinmap = pinIndex;
    plat.pins[pinIndex].gpio.mux_total = 0;
    pinIndex++;
    strncpy(plat.pins[pinIndex].name, "INT-GPIO3", MRAA_PIN_NAME_SIZE);
    plat.pins[pinIndex].capabilities = pinCapsGpio;
    plat.pins[pinIndex].gpio.pinmap = pinIndex;
    plat.pins[pinIndex].gpio.mux_total = 0;
    pinIndex++;

    // Virtual gpio pins on i2c I/O expander.
    for (int i = 0; i < numI2cGpioExpanderPins; ++i)
    {
        snprintf(plat.pins[pinIndex].name, MRAA_PIN_NAME_SIZE, "EXP-GPIO%d",
                i);
        plat.pins[pinIndex].capabilities = pinCapsGpio;
        plat.pins[pinIndex].gpio.pinmap = pinIndex;
        plat.pins[pinIndex].gpio.mux_total = 0;
        pinIndex++;
    }

    // Now add any extra i2c buses behind i2c switch
    for (bus = 1; bus < numI2cBusses; ++bus)
    {
        plat.i2c_bus[bus].bus_id = bus;
        plat.pins[pinIndex].i2c.mux_total = 0;
        snprintf(plat.pins[pinIndex].name, MRAA_PIN_NAME_SIZE, "SDA%d", bus);
        plat.pins[pinIndex].capabilities = pinCapsI2c;
        plat.i2c_bus[bus].sda = pinIndex;
        pinIndex++;
        snprintf(plat.pins[pinIndex].name, MRAA_PIN_NAME_SIZE, "SCL%d", bus);
        plat.pins[pinIndex].capabilities = pinCapsI2c;
        plat.pins[pinIndex].i2c.mux_total = 0;
        plat.i2c_bus[bus].scl = pinIndex;
        pinIndex++;
    }

    // Set override functions
    plat.adv_func = static_cast<mraa_adv_func_t *>(calloc(1, sizeof(mraa_adv_func_t)));
    if (plat.adv_func == NULL)
    {
        free(plat.pins);
        sub_plats_4222.pop_back();
        return NULL;
    }

    mraa_ftdi_ft4222_populate_i2c_func_table(plat.adv_func);
    mraa_ftdi_ft4222_populate_gpio_func_table(plat.adv_func);

    /* Success, return the sub platform */
    return &plat;
}

mraa_platform_t mraa_usb_platform_extender(mraa_board_t *board)
{
    /* If no board provided or ft4222 initialization fails, return unknown */
    if ((board == NULL) || (mraa_ftdi_ft4222_init() != MRAA_SUCCESS))
        return MRAA_UNKNOWN_PLATFORM;

    /* Basic chip version/library version check.  For now, make sure they
     * are non-zero */
    unsigned int v_chip, v_lib;
    if ((mraa_ftdi_ft4222_get_version(&v_chip, &v_lib) == MRAA_SUCCESS) &&
            (v_chip > 0) && (v_lib > 0))
    {
        /* If we have a valid subplatform, then this pointer is valid,
         * if we don't have a valid subplatform, then this pointer will be null */
        board->sub_platform = mraa_ftdi_ft4222();
    }

    return (board->sub_platform == NULL) ? MRAA_UNKNOWN_PLATFORM
        : MRAA_FTDI_FT4222;
}
