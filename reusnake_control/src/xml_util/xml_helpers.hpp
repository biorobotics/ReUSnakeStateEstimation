#pragma once

#include "pugixml.hpp"

namespace hebi {
namespace xml {

bool trySetIntParameter(pugi::xml_attribute&& attr, int& param);
bool trySetFloatParameter(pugi::xml_attribute&& attr, float& param);
bool trySetDoubleParameter(pugi::xml_attribute&& attr, double& param);
bool trySetBoolParameter(pugi::xml_attribute&& attr, bool& param);
bool trySetStringParameter(pugi::xml_attribute&& attr, std::string& param);

} // namespace xml
} // namespace hebi
