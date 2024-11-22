#include "commandUtils.hpp"
#include <iostream>
#include <string>

char CommandUtils::cr = 13;
std::string CommandUtils::cancel = "STOP ";
std::string CommandUtils::park = "# 0 P1500 # 1 P2055 # 2 P1922 # 3 P722 # 4 P1944 # 5 P1500 T3000";
std::string CommandUtils::ready = "# 0 P1500 # 1 P1833 # 2 P1600 # 3 P1055 # 4 P1500 # 5 P1500 T3000";
std::string CommandUtils::straightUp = "# 0 P1555 # 1 P1500 # 2 P655 # 3 P1450 # 4 P1500 # 5 P1450 T3000";

std::string CommandUtils::wristLeft = "# 4 P600 T500";
std::string CommandUtils::wristRight = "# 4 P2400 T500";
std::string CommandUtils::wristMiddle = "# 4 P1500 T500";

std::string CommandUtils::elbowShakingUp = "# 2 P1300 T500";
std::string CommandUtils::elbowShakingDown = "# 2 P1600 T500";

std::string CommandUtils::parkLeft = "# 0 P700 # 1 P2055 # 2 P1922 # 3 P722 # 4 P1944 # 5 P1500 T1500";
std::string CommandUtils::parkRight = "# 0 P2450 # 1 P2055 # 2 P1922 # 3 P722 # 4 P1944 # 5 P1500 T1500";