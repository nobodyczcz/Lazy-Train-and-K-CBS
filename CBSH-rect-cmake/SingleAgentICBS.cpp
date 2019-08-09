#include "SingleAgentICBS.h"
#include <iostream>
#include <ctime>


void SingleAgentICBS::updatePath(const LLNode* goal, std::vector<PathEntry> &path)
{
	path.resize(goal->timestep + 1);
	const LLNode* curr = goal;
	num_of_conf = goal->num_internal_conf;
	for(int t = goal->timestep; t >= 0; t--)
	{
		path[t].location = curr->loc;
		curr = curr->parent;
	}
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
int SingleAgentICBS::extractLastGoalTimestep(int goal_location, const std::vector< std::list< std::pair<int, int> > >* cons) {
	if (cons != NULL) {
		for (int t = static_cast<int>(cons->size()) - 1; t > 0; t--) 
		{
			for (std::list< std::pair<int, int> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it)
			{
				if (std::get<0>(*it) == goal_location && it->second < 0) 
				{
					return (t);
				}
			}
		}
	}
	return -1;
}


// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
//        cons[timestep] is a list of <loc1,loc2> of (vertex/edge) constraints for that timestep.
//inline bool SingleAgentICBS::isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)  const
//{
//	if (cons == NULL)
//		return false;
//	// check vertex constraints (being in next_id at next_timestep is disallowed)
//	if (next_timestep < static_cast<int>(cons->size()))
//	{
//		for (std::list< std::pair<int, int> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it)
//		{
//			if ((std::get<0>(*it) == next_id && std::get<1>(*it) < 0)//vertex constraint
//				|| (std::get<0>(*it) == curr_id && std::get<1>(*it) == next_id)) // edge constraint
//				return true;
//		}
//	}
//	return false;
//}



int SingleAgentICBS::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const bool* res_table, int max_plan_len) {
	int retVal = 0;
	if (next_timestep >= max_plan_len) {
		// check vertex constraints (being at an agent's goal when he stays there because he is done planning)
		if (res_table[next_id + (max_plan_len - 1)*map_size] == true)
			retVal++;
		// Note -- there cannot be edge conflicts when other agents are done moving
	}
	else {
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		if (res_table[next_id + next_timestep*map_size] == true)
			retVal++;
		// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
		// which means that res_table is occupied with another agent for [curr_id,next_timestep] and [next_id,next_timestep-1]
		if (res_table[curr_id + next_timestep*map_size] && res_table[next_id + (next_timestep - 1)*map_size])
			retVal++;
	}
	return retVal;
}

bool SingleAgentICBS::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_size)
		return false;
	int curr_x = curr / num_col;
	int curr_y = curr % num_col;
	int next_x = next / num_col;
	int next_y = next % num_col;
	return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleAgentICBS::findPath(std::vector<PathEntry> &path, double f_weight, const std::vector < std::list< std::pair<int, int> > >* constraints, const bool* res_table, size_t max_plan_len, double lowerbound, std::clock_t start_clock ,int time_limit) 
{
	num_expanded = 0;
	num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()

	 // generate start and add it to the OPEN list
	LLNode* start = new LLNode(start_location, 0, my_heuristic[start_location], NULL, 0, 0, false);
	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table[start] = start;
	min_f_val = start->getFVal();
	lower_bound = std::max(lowerbound, f_weight * min_f_val);
	std:clock_t runtime;
	int lastGoalConsTime = extractLastGoalTimestep(goal_location, constraints); // the last timestep of a constraint at the goal

	while (!focal_list.empty()) 
	{
		if (time_limit != 0) {
			runtime = std::clock() - start_clock;
			if (runtime > time_limit) {
				return false;
			}
		}
		LLNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);

		curr->in_openlist = false;
		num_expanded++;

		// check if the popped node is a goal
		if (curr->loc == goal_location && curr->timestep > lastGoalConsTime) 
		{
			updatePath(curr, path);
			releaseClosedListNodes(&allNodes_table);
			open_list.clear();
			focal_list.clear();
			allNodes_table.clear();
			return true;
		}

		for (int i = 0; i < 5; i++)
		{
			int next_id = curr->loc + moves_offset[i];

			int next_timestep = curr->timestep + 1;
			if (validMove(curr->loc, next_id) && !my_map[next_id] && !isConstrained(curr->loc, next_id, next_timestep, constraints))
			{
				// compute cost to next_id via curr node
				int next_g_val = curr->g_val + 1;
				int next_h_val = my_heuristic[next_id];
				int next_internal_conflicts = 0;
				if (max_plan_len > 0)  // check if the reservation table is not empty (that is tha max_length of any other agent's plan is > 0)
					next_internal_conflicts = curr->num_internal_conf + numOfConflictsForStep(curr->loc, next_id, next_timestep, res_table, max_plan_len);
				
				// generate (maybe temporary) node
				LLNode* next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep, next_internal_conflicts, false);

				// try to retrieve it from the hash table
				it = allNodes_table.find(next);
				if (it == allNodes_table.end()) 
				{
					next->open_handle = open_list.push(next);
					next->in_openlist = true;
					num_generated++;
					if (next->getFVal() <= lower_bound)
						next->focal_handle = focal_list.push(next);
					allNodes_table[next] = next;
				}
				else
				{  // update existing node's if needed (only in the open_list)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it).second;

					if (existing_next->in_openlist == true)
					{  // if its in the open list
						if (existing_next->getFVal() > next_g_val + next_h_val ||
							(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts))
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
							bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
							bool update_open = false;
							if ((next_g_val + next_h_val) <= lower_bound)
							{  // if the new f-val qualify to be in FOCAL
								if (existing_next->getFVal() > lower_bound)
									add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
								else
									update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
							}
							if (existing_next->getFVal() > next_g_val + next_h_val)
								update_open = true;
							// update existing node
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							existing_next->num_internal_conf = next_internal_conflicts;
	
							if (update_open) 
								open_list.increase(existing_next->open_handle);  // increase because f-val improved
							if (add_to_focal) 
								existing_next->focal_handle = focal_list.push(existing_next);
							if (update_in_focal) 
								focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
						}				
					}
					else 
					{  // if its in the closed list (reopen)
						if (existing_next->getFVal() > next_g_val + next_h_val ||
							(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) 
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							existing_next->num_internal_conf = next_internal_conflicts;
							existing_next->open_handle = open_list.push(existing_next);
							existing_next->in_openlist = true;
							if (existing_next->getFVal() <= lower_bound)
								existing_next->focal_handle = focal_list.push(existing_next);
						}
					}  // end update a node in closed list
				}  // end update an existing node
			}// end if case forthe move is legal
		}  // end for loop that generates successors
		   
		// update FOCAL if min f-val increased
		if (open_list.size() == 0)  // in case OPEN is empty, no path found
			break;
		LLNode* open_head = open_list.top();
		if (open_head->getFVal() > min_f_val) 
		{
			double new_min_f_val = open_head->getFVal();
			double new_lower_bound = std::max(lowerbound, f_weight * new_min_f_val);
			for (LLNode* n : open_list) 
			{
				if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound)
					n->focal_handle = focal_list.push(n);
			}
			min_f_val = new_min_f_val;
			lower_bound = new_lower_bound;
		}
	}  // end while loop
	  
	  // no path found
	releaseClosedListNodes(&allNodes_table);
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	return false;
}

inline void SingleAgentICBS::releaseClosedListNodes(hashtable_t* allNodes_table)
{
	hashtable_t::iterator it;
	for (it = allNodes_table->begin(); it != allNodes_table->end(); it++) 
		delete ((*it).second);  
}

SingleAgentICBS::SingleAgentICBS(int start_location, int goal_location,
	const bool* my_map, int map_size, const int* moves_offset, int num_col) 
{
	this->my_map = my_map;
	this->moves_offset = moves_offset;

	this->start_location = start_location;
	this->goal_location = goal_location;

	this->map_size = map_size;

	this->num_expanded = 0;
	this->num_generated = 0;

	this->lower_bound = 0;
	this->min_f_val = 0;

	this->num_col = num_col;

	// initialize allNodes_table (hash table)
	empty_node = new LLNode();
	empty_node->loc = -1;
	deleted_node = new LLNode();
	deleted_node->loc = -2;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);

}


SingleAgentICBS::~SingleAgentICBS()
{
	delete[] my_map;
	delete (empty_node);
	delete (deleted_node);
}
