/*
 *  Copyright 2023 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#include "atlas_utils/general/general_funcs.hpp"

namespace atlas_utils {
namespace general{

std::string getIp()
{
    struct ifaddrs *ifaddr;
    int family, s;
    char host[NI_MAXHOST];
    std::string h;

    if (getifaddrs(&ifaddr) == -1) 
    {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    /* Walk through linked list, maintaining head pointer so we
        can free list later. */
    for (struct ifaddrs *ifa = ifaddr; ifa != NULL;
            ifa = ifa->ifa_next) 
            {
                if (ifa->ifa_addr == NULL)
                    continue;

                family = ifa->ifa_addr->sa_family;

                /* Display interface name and family (including symbolic
                    form of the latter for the common families). */
                if (family != AF_INET) continue;
                std::string ifa_name(ifa->ifa_name);
                if (ifa_name.substr(0, 2) != std::string("wl")) continue;

                /* For an AF_INET* interface address, display the address. */
                if (family == AF_INET || family == AF_INET6) 
                {
                    s = getnameinfo(ifa->ifa_addr,
                            (family == AF_INET) ? sizeof(struct sockaddr_in) :
                                                    sizeof(struct sockaddr_in6),
                            host, NI_MAXHOST,
                            NULL, 0, NI_NUMERICHOST);
                    if (s != 0) 
                    {
                        std::cout << "Error getting name info of AF_INET: "
                            << ifa->ifa_addr << std::endl;
                        return "";
                    }

                    h = host;
                    break;
                }
            }

    freeifaddrs(ifaddr);
    return h;
} 

} // namespace general
} // namespace atlas_utils