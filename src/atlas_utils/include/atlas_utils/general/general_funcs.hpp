/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Albert Arlà Romero, Martí Bolet, Iñaki Lorente
   Contact: support.idi@ageve.net
*/

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/if_link.h>
#include <string>
#include <iostream>

namespace atlas_utils {
namespace general{

    std::string getIp();

} // namespace general
} // namespace atlas_utils