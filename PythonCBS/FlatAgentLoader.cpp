//
// Created by Zhe Chen on 1/7/20.
//
#include "FlatAgentLoader.h"

namespace p = boost::python;


FlatAgentLoader::FlatAgentLoader(p::object agents) {
    int agentsNum = p::len(agents);
    this->num_of_agents = agentsNum;
    for (int i = 0; i < num_of_agents; i++) {
        pair<int, int> initial;
        pair<int, int> goal;
        p::tuple iniTuple(agents[i].attr("position"));
        initial.first = p::extract<int>(p::long_(iniTuple[0]));
        initial.second = p::extract<int>(p::long_(iniTuple[1]));
        p::tuple goalTuple(agents[i].attr("target"));
        goal.first = p::extract<int>(p::long_(goalTuple[0]));
        goal.second = p::extract<int>(p::long_(goalTuple[1]));
        int heading = p::extract<int>(p::long_(agents[i].attr("direction")));
        this->initial_locations.push_back(initial);
        this->goal_locations.push_back(goal);
        this->headings.push_back(heading);
        this->min_end_time.push_back(0);
        this->done.push_back(false);


    }


}
