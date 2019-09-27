#pragma once
#include "MDD.h"


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

	typedef boost::heap::fibonacci_heap< ICBSNode*, compare<ICBSNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, compare<ICBSNode::secondary_compare_node> >::handle_type focal_handle_t;
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
	// rec_conflict: <agend_id1, agent_id2, rg (<0), s1_t, s2_t>
	// vert_conflict: <agend_id1, agent_id2, vert_loc, -1, t>
	// edge_conflict: <agend_id1, agent_id2, edge_end_loc, edge_start_loc , t>
	// **kDelay_conflict: <agend_id1, agent_id2, edge_end_loc/vert_loc, edge_start_loc/-1, a1_t, a2_t>
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>> rectCardinalConf;
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>> rectSemiConf;
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>> rectNonConf;
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>> cardinalConf;
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>> semiConf;
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>> nonConf;
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>> unknownConf;	
	//std::list<std::shared_ptr<std::tuple<int, int, int, int, int, int>>> kDelayConf;

	
	// The chosen conflict
	std::shared_ptr<std::tuple<int, int, int, int, int>> conflict;

	ICBSNode* parent;

	int agent_id;
	std::vector<PathEntry> path; // path of agent_id
	std::list<std::tuple<int, int, int>> constraints; // constraints imposed to agent_id
	

	int g_val;
	int h_val;
	int f_val;
	size_t depth; // depath of this CT node
	size_t makespan; // makespan over all paths
	int num_of_collisions; // number of conflicts in the current paths
	bool in_focalList=false;
	uint64_t time_expanded;
	uint64_t time_generated;


	void clear();
};

