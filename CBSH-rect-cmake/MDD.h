#pragma once
#include <list>
#include <vector>
#include <set>
#include "SingleAgentICBS.h"
#include "ICBSNode.h"
#include "flat_map_loader.h"
#include "ConstraintTable.h"
#include <iostream>
#include "MDDNode.h"

class MDDEmpty {
public:
	MDDLevels levels;
	int getMahattanDistance(int loc1, int loc2, int map_cols)
	{
		int loc1_x = loc1 / map_cols;
		int loc1_y = loc1 % map_cols;
		int loc2_x = loc2 / map_cols;
		int loc2_y = loc2 % map_cols;
		return std::abs(loc1_x - loc2_x) + std::abs(loc1_y - loc2_y);
	}
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

};

template<class Map>
class MDD: public MDDEmpty
{
public:
	bool buildMDD( ConstraintTable& constraints,
		int numOfLevels,  SingleAgentICBS<Map> &solver);
	bool buildMDD( ConstraintTable& constraints, int numOfLevels, SingleAgentICBS<Map> & solver, int start, int start_time,int start_heading=-1);
	bool buildMDD(ConstraintTable& constraints, int numOfLevels, SingleAgentICBS<Map> & solver, int start, int start_time,int goal, int start_heading = -1);

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
	int k = 0;

	ConstraintsHasher() {};
    ConstraintsHasher(int a, ICBSNode* n, int k = 0) : a(a), n(n),k(k) {};


	bool operator==(const ConstraintsHasher& other) const
	{
	    if (a != other.a)
	        return false;
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

		if (k != other.k)
		    return false;
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

