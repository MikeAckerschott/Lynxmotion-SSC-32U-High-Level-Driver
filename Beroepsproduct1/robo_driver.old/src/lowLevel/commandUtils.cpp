#include "commandUtils.hpp"
#include <iostream>
#include <string>

char CommandUtils::cr = 13;
std::string CommandUtils::cancel = "STOP ";
std::string CommandUtils::park = "# 0 P1500 # 1 P1888 # 2 P1966 # 3 P722 # 4 P1944 # 5 P1500 T3000";
std::string CommandUtils::ready = "# 0 P1500 # 1 P1666 # 2 P1500 # 3 P1222 # 4 P944 # 5 P1500 T3000";
std::string CommandUtils::straightUp = "# 0 P1500 # 1 P1450 # 2 P700 # 3 P1450 # 4 P944 # 5 P1450 T3000";