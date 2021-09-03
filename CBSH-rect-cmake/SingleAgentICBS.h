#pragma once

#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>
#include <ctime>
#include "common.h"
#include "Path.h"

#include "LLNode.h"
#include "map_loader.h"
#include "flat_map_loader.h"
#include "ReservationTable.h"
#include "ConstraintTable.h"
#include "compute_heuristic.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>


template<class Map>
class SingleAgentICBS
{
public:
	// define typedefs and handles for heap and hash_map
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t;
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;
	list<LLNode*> goal_nodes;
	LLNode* empty_node = NULL;
	LLNode* deleted_node = NULL;

	int agent_id;
	int start_location;
	int goal_location;
	int start_heading;
	int min_end_time = 0;
	int departure_time = 0;


	Map* ml;
	int map_size;
	int num_col;
	std::vector<hvals> my_heuristic;  // this is the precomputed heuristic for this agent

	int kRobust;

	uint64_t num_expanded;
	uint64_t num_generated;

	double lower_bound;  // FOCAL's lower bound ( = e_weight * min_f_val)
	double min_f_val;  // min f-val seen so far
	int num_of_conf; // number of conflicts between this agent to all the other agents

	options option;



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
	void updatePath(LLNode* goal, std::vector<PathEntry> &path,ReservationTable* res_table);


	// find path by time-space A* search
	// Returns true if a collision free path found  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is the lowerbound of the length of the path
	// max_plan_len used to compute the size of res_table
	bool findPath(std::vector<PathEntry> &path, double f_weight,
		ConstraintTable& constraints, ReservationTable* res_table,
		size_t max_plan_len, double lowerbound,
		std::clock_t start = 0, int time_limit = 0, bool train = false);
    bool getOccupations(list<int>& next_locs,int next_id, LLNode* curr);

	inline void releaseClosedListNodes(hashtable_t* allNodes_table);

	SingleAgentICBS(int start_location, int goal_location, Map* ml, int agent_id, options cbs_option, int start_heading = -1, int kRobust = 0, int min_end = 0);
	~SingleAgentICBS();

};


