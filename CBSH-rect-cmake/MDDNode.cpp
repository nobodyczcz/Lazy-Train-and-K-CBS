//
// Created by Zhe Chen on 24/3/20.
//
#include "MDDNode.h"


MDDNode::MDDNode(std::list<int> currlocs, MDDNode* parent)
{
    locs = currlocs;
    this->parent = parent;
    if(parent == NULL)
        level = 0;
    else
    {
        level = parent->level + 1;
        parents.push_back(parent);
    }
    parent = NULL;
}
