#pragma once
#include <list>
#include <vector>

#include "SingleAgentICBS.h"

class MDDNode
{
public:
	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if(parent == NULL)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
		parent = NULL;
	}
	int location;
	int level;
	int heading;

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level) && (this->heading == node.heading);
	}


	std::list<MDDNode*> children;
	std::list<MDDNode*> parents;
	MDDNode* parent;
};

template<class Map>
class MDD
{
public:
	std::vector<std::list<MDDNode*>> levels;

	bool buildMDD(const std::vector < std::list< std::pair<int, int> > >& constraints, 
		int numOfLevels,  SingleAgentICBS<Map> &solver);
	bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels, SingleAgentICBS<Map> & solver, int start, int start_time, int goal,int start_heading=-1);
	MDDNode* find(int location, int level);
	void deleteNode(MDDNode* node);
	void clear();

	MDD(){};
	MDD(MDD & cpy);
	~MDD();
};

