//
// Created by Zhe Chen on 5/8/21.
//
#pragma once

#ifndef CBS_K_PATH_H
#define CBS_K_PATH_H

#include "Conflict.h"

struct PathEntry
        {

    int location;
    list<int> occupations;
    vector<bool> singles;
    int heading;
    bool single;
    int actionToHere;
    int timeStep;
    int self_conflict;
    PathEntry(int loc = -1) { location = loc; single = false; }
    std::list<int> locations; // all possible locations at the same time step
    std::list<Conflict> conflist;
};

typedef std::vector<PathEntry> Path;


#endif //CBS_K_PATH_H
