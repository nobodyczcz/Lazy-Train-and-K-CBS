#pragma once
#include <list>
#include <vector>
#include <set>
#include "SingleAgentICBS.h"
#include "ICBSNode.h"

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

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}


	std::list<MDDNode*> children;
	std::list<MDDNode*> parents;
	MDDNode* parent;
};

class MDD
{
public:
	std::vector<std::list<MDDNode*>> levels;

	bool buildMDD(ConstraintTable& constraint_table, 
		int numOfLevels, const SingleAgentICBS & solver);

	MDDNode* find(int location, int level);
	void deleteNode(MDDNode* node);
	void clear();

	MDD(){};
	MDD(MDD & cpy);
	~MDD();
};

struct ConstraintsHasher // Hash a CT node by constraints on one agent
{
	int a;
	const ICBSNode* n;

	ConstraintsHasher() {};
	ConstraintsHasher(int a, const ICBSNode* n) : a(a), n(n) {};

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

typedef std::unordered_map<ConstraintsHasher, MDD*> MDDTable;