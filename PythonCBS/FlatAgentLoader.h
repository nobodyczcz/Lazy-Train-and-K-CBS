//
// Created by Zhe Chen on 1/7/20.
//

#ifndef MAPD_FLATAGENTLOADER_H
#define MAPD_FLATAGENTLOADER_H

#include <boost/python.hpp>
#include "agents_loader.h"

class FlatAgentLoader : (AgentsLoader){
public:
        FlatAgentLoader(boost::python::object agents);

};

#endif //MAPD_FLATAGENTLOADER_H
