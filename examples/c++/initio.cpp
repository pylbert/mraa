#include <iostream>

#include <mraa/initio.hpp>


using namespace mraa;

int main() {
  MraaIo mraaIo("g:5:0,p:32-some_upm_specific_stuff");

  std::cout << mraaIo.gpios.size() << " " << mraaIo.gpios[0].read() << std::endl;
}