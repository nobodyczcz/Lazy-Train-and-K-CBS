#pragma once
#include <list>
#include <vector>

#include "SingleAgentICBS.h"
#include "flat_map_loader.h"
#include <iostream>

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
	int row;
	int col;
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

class MDDEmpty {
public:
	std::vector<std::list<MDDNode*>> levels;

	virtual bool buildMDD(const std::vector < std::list< std::pair<int, int> > >& constraints,
		int numOfLevels, SingleAgentICBS<MapLoader> &solver) {};
	virtual bool buildMDD(const std::vector < std::list< std::pair<int, int> > >& constraints,
		int numOfLevels, SingleAgentICBS<FlatlandLoader> &solver) {};
	virtual bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels, SingleAgentICBS<MapLoader> & solver, int start, int start_time, int start_heading = -1) {};
	virtual bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels, SingleAgentICBS<FlatlandLoader> & solver, int start, int start_time, int start_heading = -1) {};

	virtual MDDNode* find(int location, int level) {};
	virtual void deleteNode(MDDNode* node) {};
	virtual void clear() {};
	virtual void print() {};

};

template<class Map>
class MDD: public MDDEmpty
{
public:
	bool buildMDD(const std::vector < std::list< std::pair<int, int> > >& constraints, 
		int numOfLevels,  SingleAgentICBS<Map> &solver);
	bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels, SingleAgentICBS<Map> & solver, int start, int start_time,int start_heading=-1);
	MDDNode* find(int location, int level);
	void deleteNode(MDDNode* node);
	void clear();
	void print() {
		for (int i = 0; i < levels.size(); i++) {
			std::list<MDDNode*>::iterator it;
			std::cout << "level " << i << ": ";
			for (it = levels[i].begin(); it != levels[i].end(); ++it) {
				std::cout << (*it)->row << ","<< (*it)->col<<" ";
			}
			std::cout << std::endl;
		}
	};

	MDD(){};
	MDD(MDD & cpy);
	~MDD();
};

