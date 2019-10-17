#pragma once
//#include "MDD.h"
#include "LLNode.h"
#include "Conflict.h"
#include "common.h"
#include <vector>




class ICBSNode
{
public:
	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const 
		{
			return n1->f_val >= n2->f_val;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

	// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node 
	{
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const
		{
			if (n1->num_of_collisions == n2->num_of_collisions)
			{
				if (n1->g_val == n2->g_val) {
					if (n1->h_val == n2->h_val) {
						if (rand() % 2 == 0)
							return true;
						else
							return false;
					}
					return n1->h_val <= n2->h_val;
				}
				return n1->g_val <= n2->g_val;

			}
			return n1->num_of_collisions >= n2->num_of_collisions;
		}
	};  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	// The following is used by googledensehash for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specilized template inside std namespace
	struct ICBSNodeHasher 
	{
		std::size_t operator()(const ICBSNode* n) const {
			size_t agent_id_hash = std::hash<int>()(n->agent_id);
			size_t time_generated_hash = std::hash<int>()(n->time_generated);
			return (agent_id_hash ^ (time_generated_hash << 1));
		}
	};

	// conflicts in the current paths
	std::list<std::shared_ptr<Conflict>> conflicts;
	std::list<std::shared_ptr<Conflict>> unknownConf;
	std::unordered_set<string> resolvedConflicts;

	// The chosen conflict
	//std::shared_ptr<RConflict> rConflict;
	std::shared_ptr<Conflict> conflict;

	ICBSNode* parent=NULL;
	std::vector<ICBSNode*> children;

	int agent_id;
	list<pair<int, vector<PathEntry> > > paths; // path of agent_id
	std::list<Constraint> constraints; // constraints imposed to agent_id
	

	int g_val;
	int h_val;
	int f_val;
	size_t depth; // depath of this CT node
	size_t makespan; // makespan over all paths
	int num_of_collisions; // number of conflicts in the current paths

	uint64_t time_expanded;
	uint64_t time_generated;

	

	void clear();

	ICBSNode(){}
	ICBSNode(int id): agent_id(id) {}
};

