#include "commandUtils.hpp"
#include <iostream>
#include <string>

char CommandUtils::cr = 13;
std::string CommandUtils::cancel = "STOP ";
std::string CommandUtils::park = "# 0 P1500 # 1 P2055 # 2 P1922 # 3 P722 # 4 P1944 # 5 P1500 T3000";
std::string CommandUtils::ready = "# 0 P1500 # 1 P2111 # 2 P1600 # 3 P1055 # 4 P944 # 5 P1500 T3000";
std::string CommandUtils::straightUp = "# 0 P1500 # 1 P1500 # 2 P655 # 3 P1450 # 4 P944 # 5 P1450 T3000";