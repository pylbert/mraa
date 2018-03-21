#pragma once

#include "initio.h"
#include <string>
#include <stdexcept>
#include <vector>

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

      aios.insert(aios.end(), &descs->aios[0], &descs->aios[descs->n_aio]);
      gpios.insert(gpios.end(), &descs->gpios[0], &descs->gpios[descs->n_gpio]);
      i2cs.insert(i2cs.end(), &descs->i2cs[0], &descs->i2cs[descs->n_i2c]);
      iios.insert(iios.end(), &descs->iios[0], &descs->iios[descs->n_iio]);
      pwms.insert(pwms.end(), &descs->pwms[0], &descs->pwms[descs->n_pwm]);
      spis.insert(spis.end(), &descs->spis[0], &descs->spis[descs->n_spi]);
      uarts.insert(uarts.end(), &descs->uarts[0], &descs->uarts[descs->n_uart]);
      uart_ows.insert(uart_ows.end(), &descs->uart_ows[0], &descs->uart_ows[descs->n_uart_ow]);

      if (descs->leftover_str) {
        leftoverStr = std::string(descs->leftover_str);
      } else {
        leftoverStr = std::string("");
      }
    }

    ~MraaIo()
    {
      mraa_io_close(descs);
    }

  public:
    std::vector<mraa_aio_context> aios;
    std::vector<mraa_gpio_context> gpios;
    std::vector<mraa_i2c_context> i2cs;
    std::vector<mraa_iio_context> iios;
    std::vector<mraa_pwm_context> pwms;
    std::vector<mraa_spi_context> spis;
    std::vector<mraa_uart_context> uarts;
    std::vector<mraa_uart_ow_context> uart_ows;

  protected:
    /* Used exclusively by the UPM library. */
    std::string leftoverStr;
};
}