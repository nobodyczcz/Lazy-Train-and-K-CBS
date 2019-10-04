#pragma once
#include <list>
#include <vector>
#include <set>
#include "SingleAgentICBS.h"
#include "ICBSNode.h"
#include "flat_map_loader.h"
#include "ConstraintTable.h"
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

	virtual bool buildMDD( ConstraintTable& constraint_table,
		int numOfLevels, SingleAgentICBS<MapLoader>& solver) {};
	virtual bool buildMDD( ConstraintTable& constraint_table,
		int numOfLevels, SingleAgentICBS<FlatlandLoader>& solver) {};
	virtual bool buildMDD( ConstraintTable& constraint_table, int numOfLevels,
		SingleAgentICBS<MapLoader>& solver, int start, int start_time, int start_heading = -1) {};
	virtual bool buildMDD( ConstraintTable& constraint_table, int numOfLevels,
		SingleAgentICBS<FlatlandLoader>& solver, int start, int start_time, int start_heading = -1) {};

	virtual MDDNode* find(int location, int level) {};
	virtual void deleteNode(MDDNode* node) {};
	virtual void clear() {};
	virtual void print() {};
	~MDDEmpty() { clear(); };

};

template<class Map>
class MDD: public MDDEmpty
{
public:
	bool buildMDD( ConstraintTable& constraints,
		int numOfLevels,  SingleAgentICBS<Map> &solver);
	bool buildMDD( ConstraintTable& constraints, int numOfLevels, SingleAgentICBS<Map> & solver, int start, int start_time,int start_heading=-1);
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

struct ConstraintsHasher // Hash a CT node by constraints on one agent
{
	int a;
	ICBSNode* n;

	ConstraintsHasher() {};
	ConstraintsHasher(int a, ICBSNode* n) : a(a), n(n) {};

	bool operator==(const ConstraintsHasher& other) const
	{
		std::set<Constraint> cons1, cons2;
		const ICBSNode* curr = n;
		while (curr->parent != NULL)
		{
			if (curr->agent_id == a)
				for (auto con : curr->constraints)
					cons1.insert(con);
			curr = curr->parent;
		}
		curr = other.n;
		while (curr->parent != NULL)
		{
			if (curr->agent_id == a)
				for (auto con : curr->constraints)
					cons2.insert(con);
			curr = curr->parent;
		}
		if (cons1.size() != cons2.size())
			return false;

		if (!equal(cons1.begin(), cons1.end(), cons2.begin()))
			return false;
		else
			return true;
	}
};

template <>
struct std::hash<ConstraintsHasher>
{
	std::size_t operator()(const ConstraintsHasher& entry) const
	{
		const ICBSNode* curr = entry.n;
		size_t cons_hash = 0;
		while (curr->parent != NULL)
		{
			if (curr->agent_id == entry.a)
			{
				for (auto con : curr->constraints)
				{
					cons_hash += 3 * std::hash<int>()(std::get<0>(con)) + 5 * std::hash<int>()(std::get<1>(con)) + 7 * std::hash<int>()(std::get<2>(con));
				}
			}
			curr = curr->parent;
		}
		return (cons_hash << 1);
	}
};

