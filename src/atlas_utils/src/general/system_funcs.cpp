/*
 *  Copyright 2023 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#include "atlas_utils/general/system_funcs.hpp"

namespace atlas_utils{
namespace utils {

// Main function of this file, executes and returns the result of the command
std::string executeCommand(std::string command) {
    char buffer[128];
    std::string result = "";
    // Open pipe to file
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return "popen failed!";
    }
    // read till end of process:
    while (!feof(pipe)) {
        // use buffer to read and add to result
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

std::string getPathOfFilename(std::string filename) {
    std::string directory = executeCommand(std::string("echo $(dirname ")+filename+std::string(")"));
    boost::replace_all(directory,"\n","");
    std::string exec_directory = executeCommand(std::string("cd ")+directory+std::string(" && echo $(pwd)"));
    boost::replace_all(exec_directory,"\n","");
    return exec_directory;
}

std::vector<std::string> getLocateOfFilename(std::string filename) {
    std::vector<std::string> tmp;
    std::string list_of_files = executeCommand(std::string("locate ")+filename);
    boost::split(tmp, list_of_files, boost::algorithm::is_any_of("\n"));
    return tmp;
}

}
}