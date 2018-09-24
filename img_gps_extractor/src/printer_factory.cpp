#include "printer_factory.hpp"

#include "printer_yaml.hpp"
#include "printer_csv.hpp"

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace gpmf_io
{
  std::unique_ptr<printer> printer_factory::create (const std::string& filename)
  {
    if (filename.empty ())
      return nullptr;

    std::string ext = fs::extension (filename);
    if (ext.empty ())
      return nullptr;

    std::transform(ext.begin (), ext.end (), ext.begin (), ::tolower);
    if (ext == ".yaml")
      return std::unique_ptr<printer>(new printer_yaml (filename));
    else if (ext == ".csv")
      return std::unique_ptr<printer>(new printer_csv (filename));
    else
      return nullptr;
  }
}
