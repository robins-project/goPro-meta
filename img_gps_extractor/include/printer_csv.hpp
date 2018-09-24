#ifndef PRINTER_CSV_HPP
#define PRINTER_CSV_HPP

#include "printer.hpp"
#include <fstream>

namespace gpmf_io
{
  class printer_csv : public printer
  {
  public:
    printer_csv(const std::string& filename)
      : _filename(filename)
    {
    }

    virtual ~printer_csv()
    {
      std::fstream fs;
      fs.open(_filename, std::fstream::out);
      fs << _out.str().c_str();
      fs.close();
    }

    virtual void BeginGroup()
    {
    }

    virtual void Header(const std::string& name)
    {
    }

    virtual void Comment(const std::string& str)
    {
    }

    virtual void Begin()
    {
    }

    virtual void Record(const std::string& name, const float  value)
    {
      _out << value << '\t';
    }

    virtual void Record(const std::string& name, const double value)
    {
      _out << value << '\t';
    }

    virtual void End()
    {
    }

    virtual void EndGroup()
    {
      _out << '\n';
    }

  private:

    std::stringstream _out;
    std::string       _filename;
  };
}

#endif // PRINTER_CSV_HPP
