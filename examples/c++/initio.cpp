#include <iostream>

#include <mraa/initio.hpp>


using namespace mraa;

int main() {
  MraaIo mraaIo("g:5:1-some_upm_specific_stuff");

  /* For now, blend mraa C API calls with the existing C++ API. */
  std::cout << mraa_gpio_read(mraaIo.gpios[0]) << std::endl;
}