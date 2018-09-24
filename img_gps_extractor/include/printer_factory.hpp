#ifndef PRINTER_FACTORY_HPP
#define PRINTER_FACTORY_HPP

#include <memory>
#include <printer.hpp>

namespace gpmf_io
{
  class printer_factory
  {
  public:
    static std::unique_ptr<printer> create(const std::string& filename);

  private:

    printer_factory();
  };
}

#endif // PRINTER_FACTORY_HPP
