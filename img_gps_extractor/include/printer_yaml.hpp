#ifndef PRINTER_YAML_HPP
#define PRINTER_YAML_HPP

#include "printer.hpp"
#include <fstream>

// libyaml stuff
#include "yaml-cpp/yaml.h"

namespace gpmf_io
{
  class printer_yaml : public printer
  {
  public:
    printer_yaml(const std::string& filename)
      : _filename(filename)
    {
    }

    virtual ~printer_yaml()
    {
      std::fstream fs;
      fs.open(_filename, std::fstream::out);
      fs << _out.c_str();
      fs.close();
    }

    virtual void Header(const std::string& name)
    {
      _out << YAML::Key << name;
      _out << YAML::Value;
    }

    virtual void Comment(const std::string& str)
    {
      _out << YAML::Comment(str);
    }

    virtual void Begin()
    {
      _out << YAML::BeginMap;
    }

    virtual void Record(const std::string& name, const float  value)
    {
      _out << YAML::Key << name << YAML::Value << value;
    }

    virtual void Record(const std::string& name, const double value)
    {
      _out << YAML::Key << name << YAML::Value << value;
    }

    virtual void End()
    {
      _out << YAML::EndMap;
    }

  private:

    YAML::Emitter _out;
    std::string   _filename;
  };
}

#endif // PRINTER_YAML_HPP
