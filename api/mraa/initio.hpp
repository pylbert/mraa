#pragma once

#include "initio.h"
#include <string>
#include <stdexcept>
#include <vector>

#include "aio.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "iio.hpp"
#include "pwm.hpp"
#include "spi.hpp"
#include "uart.hpp"
#include "uart_ow.hpp"

namespace mraa
{
class MraaIo
{
  private:
      mraa_io_descriptor *descs;

  public:
    MraaIo(std::string initStr)
    {
      if (mraa_io_init(initStr.c_str(), &descs) != MRAA_SUCCESS) {
        throw std::runtime_error("mraa_io_init error");
      }

      for (int i = 0; i < descs->n_aio; ++i) {
        aios.emplace_back(descs->aios[i]);
      }

      for (int i = 0; i < descs->n_gpio; ++i) {
        gpios.emplace_back(descs->gpios[i]);
      }

      for (int i = 0; i < descs->n_i2c; ++i) {
        i2cs.emplace_back(descs->i2cs[i]);
      }

      for (int i = 0; i < descs->n_iio; ++i) {
        iios.emplace_back(descs->iios[i]);
      }

      for (int i = 0; i < descs->n_pwm; ++i) {
        pwms.emplace_back(descs->pwms[i]);
      }

      for (int i = 0; i < descs->n_spi; ++i) {
        spis.emplace_back(descs->spis[i]);
      }

      for (int i = 0; i < descs->n_uart; ++i) {
        uarts.emplace_back(descs->uarts[i]);
      }

      for (int i = 0; i < descs->n_uart_ow; ++i) {
        uart_ows.emplace_back(descs->uart_ows[i]);
      }

      if (descs->leftover_str) {
        leftoverStr = std::string(descs->leftover_str);
      } else {
        leftoverStr = std::string("");
      }
    }

    ~MraaIo()
    {
      if (descs->leftover_str)
        free(descs->leftover_str);

      if (descs->n_aio)
        free(descs->aios);
      if (descs->n_gpio)
        free(descs->gpios);
      if (descs->n_i2c)
        free(descs->i2cs);
      if (descs->n_iio)
        free(descs->iios);
      if (descs->n_pwm)
        free(descs->pwms);
      if (descs->n_spi)
        free(descs->spis);
      if (descs->n_uart)
        free(descs->uarts);
      if (descs->n_uart_ow)
        free(descs->uart_ows);

      /* Finally free the mraa_io_descriptor structure. */
      free(descs);
    }

  public:
    std::vector<Aio> aios;
    std::vector<Gpio> gpios;
    std::vector<I2c> i2cs;
    std::vector<Iio> iios;
    std::vector<Pwm> pwms;
    std::vector<Spi> spis;
    std::vector<Uart> uarts;
    std::vector<UartOW> uart_ows;

  protected:
    /* Used exclusively by the UPM library. */
    std::string leftoverStr;
};
}