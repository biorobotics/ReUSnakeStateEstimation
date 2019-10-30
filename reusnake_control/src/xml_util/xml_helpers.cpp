#include "xml_helpers.hpp"

namespace hebi {
namespace xml {

bool trySetIntParameter(pugi::xml_attribute&& attr, int& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_int();
    return true;
  }
  return false;
}

bool trySetFloatParameter(pugi::xml_attribute&& attr, float& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_float();
    return true;
  }
  return false;
}

bool trySetDoubleParameter(pugi::xml_attribute&& attr, double& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_double();
    return true;
  }
  return false;
}

bool trySetBoolParameter(pugi::xml_attribute&& attr, bool& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_bool();
    return true;
  }
  return false;
}

bool trySetStringParameter(pugi::xml_attribute&& attr, std::string& param)
{
  if (attr)
  {
    // TODO: handle bad parse?
    param = attr.as_string();
    return true;
  }
  return false;
}

} // namespace xml
} // namespace hebi
