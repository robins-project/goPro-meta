#ifndef PRINTER_HPP
#define PRINTER_HPP

#include <string>

namespace gpmf_io
{
  class printer
  {
  public:

    virtual ~printer()
    {
    }

    virtual void BeginGroup() = 0;

    virtual void Header(const std::string& name) = 0;
    virtual void Comment(const std::string& str) = 0;

    virtual void Begin() = 0;
    virtual void Record(const std::string& name, const uint64_t value) = 0;
    virtual void Record(const std::string& name, const float  value) = 0;
    virtual void Record(const std::string& name, const double value) = 0;
    virtual void End() = 0;

    virtual void EndGroup() = 0;

  protected:

    printer()
    {
    }
  };
}

#endif // PRINTER_HPP
