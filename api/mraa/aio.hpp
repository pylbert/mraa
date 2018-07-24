/*
 * Author: Brendan Le Foll <brendan.le.foll@intel.com>
 * Copyright (c) 2014 Intel Corporation.
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

#pragma once

#include "aio.h"
#include "types.hpp"
#include <map>
#include <stdexcept>

namespace mraa
{

/**
 * @brief API to Analog IO
 *
 * This file defines the aio interface for libmraa
 *
 * @snippet aio.cpp Interesting
 */
class Aio
{
  public:
    /**
     * Aio Constructor, takes a pin number which will map directly to the
     * board number
     *
     * @param pin channel number to read ADC inputs
     */
    Aio(int pin)
    {
        m_aio = mraa_aio_init(pin);
        if (m_aio == NULL) {
            throw std::invalid_argument("Invalid AIO pin specified - do you have an ADC?");
        }
        m_map()[m_aio]++;
    }
    /**
     * Aio Constructor, takes a pointer to the AIO context and initialises
     * the AIO class. Aio pins are always 0 indexed reguardless of their
     * position. Check your board mapping for details. An arduino style layout
     * will have A0 as pin14 but AIO0.
     *
     * @param aio_context void * to an AIO context
     */
    Aio(void* aio_context)
    {
        m_aio = (mraa_aio_context) aio_context;
        if (m_aio == NULL) {
            throw std::invalid_argument("Invalid AIO context");
        }
        m_map()[static_cast<mraa_aio_context>(aio_context)]++;
    }

    /**
     * Handle Aio copy
     * Create a new Aio copy from src
     * @param src Target Aio instance to copy
     */
    Aio(const Aio& src)
    {
        m_aio = src.m_aio;
        m_map()[m_aio]++;
    }

    /**
     * Handle Aio assignment.
     * Multiple Aio class instances can reference the same aio context.
     * @param src Target Aio instance to copy
     * @return New Aio instance which references the same resources from src
     */
    Aio&
    operator=(const Aio& src)
    {
        this->m_aio = src.m_aio;
        m_map()[m_aio]++;
        return *this;
    }

    /**
     * Aio destructor
     */
    virtual ~Aio()
    {
        /* If this is the last Aio object which references the
         * mraa_aio_context, then remove the context from the map and
         * free up the aio resources
         */
        if (m_map()[m_aio]-- == 1) {
            m_map().erase(m_aio);
            mraa_aio_close(m_aio);
        }
    }

    /**
     * Read a value from the AIO pin. By default mraa will shift
     * the raw value up or down to a 10 bit value.
     *
     * @throws std::invalid_argument in case of error
     * @returns The current input voltage. By default, a 10bit value
     */
    unsigned int
    read()
    {
        int x = mraa_aio_read(m_aio);
        if (x == -1) {
            throw std::invalid_argument("Unknown error in Aio::read()");
        }
        return (unsigned int) x;
    }
    /**
     * Read a value from the AIO pin and return it as a normalized float.
     *
     * @throws std::invalid_argument in case of error
     * @returns The current input voltage as a normalized float (0.0f-1.0f)
     */
    float
    readFloat()
    {
        float x = mraa_aio_read_float(m_aio);
        if (x == -1.0f) {
            throw std::invalid_argument("Unknown error in Aio::readFloat()");
        }
        return x;
    }
    /**
     * Set the bit value which mraa will shift the raw reading
     * from the ADC to. I.e. 10bits
     * @param bits the bits the return from read should be i.e 10
     * @return mraa::Result type
     */
    Result
    setBit(int bits)
    {
        return (Result) mraa_aio_set_bit(m_aio, bits);
    }
    /**
     * Gets the bit value mraa is shifting the analog read to.
     *
     * @return bit value mraa is set return from the read function
     */
    int
    getBit()
    {
        return mraa_aio_get_bit(m_aio);
    }

  private:
    mraa_aio_context m_aio;

    /* Keep track of instances per mraa_aio_context */
    static std::map<mraa_aio_context, int>&
    m_map()
    {
        static std::map<mraa_aio_context, int> _map;
        return _map;
    }
};
}
