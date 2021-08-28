//
// Created by Zhe Chen on 24/3/20.
//
#pragma once
#ifndef CBS_K_MDDNODE_H
#define CBS_K_MDDNODE_H
#endif //CBS_K_MDDNODE_H
#include <iostream>
#include <list>
#include <vector>



class MDDNode
{
public:
    MDDNode(std::list<int> currlocs, MDDNode* parent, bool train);

    std::list<int> locs;
    int row;
    int col;
    int level;
    int heading;
    bool train;
    bool shrinking = false;

    bool operator == (const MDDNode & node) const
    {
        bool same_locs = true;
        if (!train){
            same_locs = locs.front() == node.locs.front();
        }
        else if (this->locs.size() != node.locs.size())
            same_locs = false;
        else{
            auto it1 = this->locs.begin();
            auto it2 = node.locs.begin();
            while (it1 != this->locs.end() && it2 != node.locs.end()){
                if (*it1 != *it2){
                    same_locs = false;
                    break;
                }
                it1++;
                it2++;
            }
        }

        return (same_locs) && (this->level == node.level) && (this->heading == node.heading);
    }


    std::list<MDDNode*> children;
    std::list<MDDNode*> parents;
    MDDNode* parent;
};


typedef std::vector<std::list<MDDNode*>> MDDLevels;
