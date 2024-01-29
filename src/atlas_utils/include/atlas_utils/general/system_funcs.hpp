/*
 *  Copyright 2021 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#ifndef CMD_EXEC_UTILS_HPP
#define CMD_EXEC_UTILS_HPP

#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace atlas_utils{
namespace utils {

std::string executeCommand(std::string command);

std::string getPathOfFilename(std::string filename);

std::vector<std::string> getLocateOfFilename(std::string filename);

} // namespace
}

#endif // CMD_EXEC_UTILS_HPP

