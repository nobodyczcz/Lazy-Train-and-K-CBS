#pragma once

#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>

#include "LLNode.h"
#include "map_loader.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <google/dense_hash_map>


class SingleAgentICBS
{
public:
	// define typedefs and handles for heap and hash_map
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t;
	typedef google::dense_hash_map<LLNode*, LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;
	LLNode* empty_node;
	LLNode* deleted_node;

	int start_location;
	int goal_location;

	const bool* my_map;
	int map_size;
	int num_col;
	const int* moves_offset;
	std::vector<int> my_heuristic;  // this is the precomputed heuristic for this agent

	uint64_t num_expanded;
	uint64_t num_generated;

	double lower_bound;  // FOCAL's lower bound ( = e_weight * min_f_val)
	double min_f_val;  // min f-val seen so far
	int num_of_conf; // number of conflicts between this agent to all the other agents

	//returns the minimal plan length for the agent (that is, extract the latest timestep which
	// has a constraint invloving this agent's goal location).
	int extractLastGoalTimestep(int goal_location, const std::vector< std::list<std::pair<int, int> > >* cons);

	//Checks if a vaild path found (wrt my_map and constraints)
	//Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
	//bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons) const;
	inline bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)  const
	{
		if (cons == NULL)
			return false;
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		if (next_timestep < static_cast<int>(cons->size()))
		{
			for (std::list< std::pair<int, int> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it)
			{
				if ((std::get<0>(*it) == next_id && std::get<1>(*it) < 0)//vertex constraint
					|| (std::get<0>(*it) == curr_id && std::get<1>(*it) == next_id)) // edge constraint
					return true;
			}
		}
		return false;
	}

	// Updates the path datamember
	void updatePath(const LLNode* goal, std::vector<PathEntry> &path); 

	// Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
	// Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
	int numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const bool* res_table, int max_plan_len);

	// find path by time-space A* search
	// Returns true if a collision free path found  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is the lowerbound of the length of the path
	// max_plan_len used to compute the size of res_table
	bool findPath(std::vector<PathEntry> &path, double f_weight, const std::vector < std::list< std::pair<int, int> > >* constraints, const bool* res_table, size_t max_plan_len, double lowerbound);

	bool validMove(int curr, int next) const; // whetehr curr->next is a valid move

	inline void releaseClosedListNodes(hashtable_t* allNodes_table);

	SingleAgentICBS(int start_location, int goal_location, const bool* my_map, int map_size, const int* moves_offset, int num_col);
	~SingleAgentICBS();

};

