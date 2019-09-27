#include "ICBSSearch.h"
#include "flat_map_loader.h"
#include "ReservationTable.h"

#include <ctime>
#include <iostream>
#include <limits.h>


// takes the and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr)
{
	for(int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr->parent != NULL)
	{
		if (!updated[curr->agent_id])
		{
			paths[curr->agent_id] = &(curr->path);
			updated[curr->agent_id] = true;
		}
		curr = curr->parent;
	}
}


std::vector <std::list< std::pair<int, int> > >* ICBSSearch::collectConstraints(ICBSNode* curr, int agent_id)
{
	std::clock_t t1 = std::clock();
	// extract all constraints on agent_id
	list < tuple<int, int, int> > constraints;
	int max_timestep = -1;
	while (curr != dummy_start) 
	{
		if (curr->agent_id == agent_id) 
		{
			for (auto constraint : curr->constraints)
			{
				constraints.push_back(constraint);
				if (get<2>(constraint) > max_timestep) // calc constraints' max_timestep
					max_timestep = get<2>(constraint);
			}
		}
		curr = curr->parent;
	}


	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > >(max_timestep+kDelay + 1, list< pair<int, int> >());
	
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) 
	{
		if (get<0>(*it) < 0) // barrier constraint
		{
			int x1 = (-get<0>(*it) - 1) / num_col, y1 = (-get<0>(*it) - 1) % num_col;
			int x2 = get<1>(*it) / num_col, y2 = get<1>(*it) % num_col;

			if (x1 == x2) //row barrier
			{
				if (y1 < y2) // down barrier
					for (int i = 0; i <= y2 - y1; i++) {
						if (get<2>(*it) - i >= 0)
							cons_vec->at(get<2>(*it) - i).push_back(make_pair(x1 * num_col + y2 - i, -1));
					}
				else //up barrier
					for (int i = 0; i <= y1 - y2; i++) {
						if (get<2>(*it) - i >= 0)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair(x1 * num_col + y2 + i, -1));
					}
			}
			else // y1== y2 column barrier
			{
				if (x1 < x2)
					for (int i = 0; i <= x2 - x1; i++) {
						if (get<2>(*it) - i >= 0)
							cons_vec->at(get<2>(*it) - i).push_back(make_pair((x2 - i) * num_col + y1, -1));
					}
				else
					for (int i = 0; i <= x1 - x2; i++) {
						if (get<2>(*it) - i >= 0)
							cons_vec->at(get<2>(*it) - i).push_back(make_pair((x2 + i) * num_col + y1, -1));
					}
			}
		}
		else
			cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));
	}
	
	runtime_updatecons += std::clock() - t1;
	return cons_vec;
}

int ICBSSearch::computeHeuristics(const ICBSNode& curr)
{
	// Conflict graph
	vector<vector<bool>> CG(num_of_agents);
	int num_of_CGnodes = 0, num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
		CG[i].resize(num_of_agents, false);
	for (list<std::shared_ptr<tuple<int,int,int,int,int>>>::const_iterator it = curr.cardinalConf.begin(); it != curr.cardinalConf.end(); ++it)
	{
		if(!CG[get<0>(**it)][get<1>(**it)])
		{
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			num_of_CGedges++;
		}
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.rectCardinalConf.begin(); it != curr.rectCardinalConf.end(); ++it)
	{
		if (!CG[get<0>(**it)][get<1>(**it)])
		{
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			num_of_CGedges++;
		}
	}

	if (num_of_CGedges < 2)
		return num_of_CGedges;

	// Compute #CG nodes that have edges
	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = 0; j < num_of_agents; j++)
		{
			if (CG[i][j])
			{
				num_of_CGnodes++;
				break;
			}
		}
	}

	// Minimum Vertex Cover
	if (curr.parent == NULL) // root node of CBS tree
	{
		for (int i = 1; i < num_of_CGnodes; i++)
			if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, i))
				return i;
	}
	if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val - 1))
		return curr.parent->h_val - 1;
	else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val))
		return curr.parent->h_val;
	else
		return curr.parent->h_val + 1;
}



// Whether there exists a k-vertex cover solution
bool ICBSSearch::KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k)
{
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k) 
		return false;

	vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < num_of_agents - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < num_of_agents && flag; j++)
		{
			if (CG[i][j])
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		vector<vector<bool>> CG_copy(num_of_agents);
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < num_of_agents; j++)
		{
			if (CG_copy[node[i]][j])
			{
				CG_copy[node[i]][j] = false;
				CG_copy[j][node[i]] = false;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1))
			return true;
	}
	return false;
}

// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void ICBSSearch::copyConflicts(const std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>>& conflicts, 
	std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>>& copy, int excluded_agent) const
{
	for (std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>>::const_iterator it = conflicts.begin(); it != conflicts.end(); ++it)
	{
		if (get<0>(**it) != excluded_agent && get<1>(**it) != excluded_agent)
		{
			copy.push_back(*it);
		}
	}
}


void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (debug_mode) {
		cout << "start find conflict" << endl;
		printPaths();
	}
	if (curr.parent != NULL)
	{
		// Copy from parent
		copyConflicts(curr.parent->rectCardinalConf, curr.rectCardinalConf, curr.agent_id);
		copyConflicts(curr.parent->rectSemiConf, curr.rectSemiConf, curr.agent_id);
		copyConflicts(curr.parent->rectNonConf, curr.rectNonConf, curr.agent_id);
		copyConflicts(curr.parent->cardinalConf, curr.cardinalConf, curr.agent_id);
		copyConflicts(curr.parent->semiConf, curr.semiConf, curr.agent_id);
		copyConflicts(curr.parent->nonConf, curr.nonConf, curr.agent_id);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, curr.agent_id);

		// detect new conflicts
		int a1 = curr.agent_id;

		//collect conflict from path;
		for (size_t t = 0; t < paths[curr.agent_id]->size(); t++) {
			if (paths[curr.agent_id]->at(t).conflist != NULL) {
				curr.unknownConf.splice(curr.unknownConf.end(), *(paths[curr.agent_id]->at(t).conflist));
			}
		}

		//Detect any agent conflict with goal location
		for (int a2 = 0; a2 < num_of_agents; a2++)
		{
			if(a1 == a2)
				continue;

			if (paths[a1]->size() + 1 < paths[a2]->size())
			{
				int loc1 = paths[a1]->back().location;
				for (size_t timestep = paths[a1]->size(); timestep < paths[a2]->size(); timestep++)
				{
					int loc2 = paths[a2]->at(timestep).location;
					if (loc1 == loc2)
					{
						//for goal, current code won't influence k delay plan
						curr.unknownConf.push_front(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, -1, timestep))); // It's at least a semi conflict			
					}
				}
			}

		}


	}
	else
	{
		for(int a1 = 0; a1 < num_of_agents ; a1++)
		{
			//collect conflicts from path
			for (size_t t = 0; t < paths[a1]->size(); t++) {
				if (paths[a1]->at(t).conflist != NULL) {
					curr.unknownConf.splice(curr.unknownConf.end(), *(paths[a1]->at(t).conflist));
				}
			}
			for (int a2 = a1+1 ; a2 < num_of_agents; a2++)
			{

				size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
				//collect conflict from path;
				
					
				
				if (paths[a1]->size() != paths[a2]->size())
				{
					//short one a1_ longer one a2_
					//current short after goal code should work good for k robust
					int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
					int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
					int loc1 = paths[a1_]->back().location;// short one's goal location
					for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
					{
						//in future longer one can't pass through the goal location of shorter one.
						int loc2 = paths[a2_]->at(timestep).location;
						if (loc1 == loc2)
						{
							curr.unknownConf.push_front(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1_, a2_, loc1, -1, timestep))); // It's at least a semi conflict			
						}
					}
				}
			}
		}
	}
}

std::shared_ptr<tuple<int, int, int, int, int>> ICBSSearch::chooseEarliestConflict(ICBSNode &parent)
{
	if (parent.unknownConf.empty())
		return NULL;  // No conflict
	std::shared_ptr<tuple<int, int, int, int, int>> choose = parent.unknownConf.front();
	for (auto con : parent.unknownConf)
	{
		if (get<4>(*con) < get<4>(*choose))
			choose = con;
	}
	return choose;
}

std::shared_ptr<tuple<int, int, int, int, int>> ICBSSearch::classifyConflicts(ICBSNode &parent)
{
	if (parent.rectCardinalConf.empty() && parent.rectSemiConf.empty() && parent.rectNonConf.empty() &&
		parent.cardinalConf.empty() && parent.semiConf.empty() && parent.nonConf.empty() && parent.unknownConf.empty())
		return NULL; // No conflict

	// Classify all conflicts in unknownConf
	while (!parent.unknownConf.empty())
	{
		std::shared_ptr<tuple<int, int, int, int, int>> con = parent.unknownConf.front();
		parent.unknownConf.pop_front();

		bool cardinal1 = false, cardinal2 = false;
		if (get<4>(*con) >= paths[get<0>(*con)]->size())
			cardinal1 = true;
		else if (!paths[get<0>(*con)]->at(0).single)
		{
			buildMDD(parent, get<0>(*con));
		}
		if (get<4>(*con) >= paths[get<1>(*con)]->size())
			cardinal2 = true;
		else if (!paths[get<1>(*con)]->at(0).single)
		{
			buildMDD(parent, get<1>(*con));
		}

		if (get<3>(*con) >= 0) // Edge conflict
		{
			cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single && paths[get<0>(*con)]->at(get<4>(*con) - 1).single;
			cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single && paths[get<1>(*con)]->at(get<4>(*con) - 1).single;
		}
		else // vertex conflict
		{
			if (!cardinal1)
				cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single;
			if (!cardinal2)
				cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single;
		}
		if (cardinal1 && cardinal2) 
		{
			parent.cardinalConf.push_back(con);
			if (cons_strategy == constraint_strategy::ICBS)
				return con;
			continue;
		}
		else if (cardinal1 || cardinal2)
		{
			parent.semiConf.push_back(con);
		}
		else
		{
			parent.nonConf.push_back(con);
		}

		if (cons_strategy == constraint_strategy::ICBS || cons_strategy == constraint_strategy::CBSH)
			continue;
		else if(get<3>(*con) >= 0) // Edge conflict
			continue;
		else if (paths[get<0>(*con)]->size() <= get<4>(*con) || paths[get<1>(*con)]->size() <= get<4>(*con))//conflict happens after agent reaches its goal
			continue;

		//Rectangle reasoning for semi and non cardinal vertex conflicts
		int a1 = get<0>(*con);
		int a2 = get<1>(*con);

		if (cons_strategy == constraint_strategy::CBSH_CR)//Identify cardinal rectangle by start and goals
		{
			if(isRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2],
				paths[a1]->size() - 1, paths[a2]->size() - 1) &&
				classifyRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2]) == 2)
			{
				std::pair<int, int> Rg = getRg(al.initial_locations[a1], al.goal_locations[a1], al.goal_locations[a2]);
				parent.rectCardinalConf.push_back(std::shared_ptr<tuple<int, int, int, int, int>>
					(new tuple<int, int, int, int, int>(a1, a2, -1 - Rg.first * num_col - Rg.second, 0, 0)));
			}
		}
		else if (cons_strategy == constraint_strategy::CBSH_R)//Identify rectangle by start and goals
		{
			//Identify rectangles by start and goals
			if (isRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2],
				paths[a1]->size() - 1, paths[a2]->size() - 1))
			{
				int type = classifyRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2]);
				std::pair<int, int> Rg = getRg(al.initial_locations[a1], al.goal_locations[a1], al.goal_locations[a2]);
			
				std::shared_ptr<tuple<int, int, int, int, int >> conflict = std::shared_ptr<tuple<int, int, int, int, int>>
					(new tuple<int, int, int, int, int>(a1, a2, -1 - Rg.first * num_col - Rg.second, 0, 0));
				if (type == 2) // cardinal rectangle
				{
					parent.rectCardinalConf.push_back(conflict);
				}
				else if (type == 1 && !findRectangleConflict(parent.parent, *conflict))
				{
					parent.rectSemiConf.push_back(conflict);
				}
				else if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
				{
					parent.rectNonConf.push_back(conflict);
				}
			}
		}
		else if (cons_strategy == constraint_strategy::CBSH_RM)
		{
			int timestep = get<4>(*con);
			std::list<int>	s1s = getStartCandidates(*paths[a1], timestep, num_col);
			std::list<int>	g1s = getGoalCandidates(*paths[a1], timestep, num_col);
			std::list<int>	s2s = getStartCandidates(*paths[a2], timestep, num_col);
			std::list<int>	g2s = getGoalCandidates(*paths[a2], timestep, num_col);

			// Try all possible combinations
			bool found = false;
			std::shared_ptr<tuple<int, int, int, int, int>> conflict;
			int type = -1;
			int area = 0;
			for (int t1_start: s1s)
			{
				for (int t1_end: g1s)
				{
					int s1 = paths[a1]->at(t1_start).location;
					int g1 = paths[a1]->at(t1_end).location;
					if (!isManhattanOptimal(s1, g1, t1_end - t1_start, num_col))
						continue;
					for (int t2_start: s2s)
					{
						for (int t2_end: g2s)
						{
							int s2 = paths[a2]->at(t2_start).location;
							int g2 = paths[a2]->at(t2_end).location;
							if (!isManhattanOptimal(s2, g2, t2_end - t2_start, num_col))
								continue;
							if (!isRectangleConflict(s1, s2, g1, g2, num_col))
								continue;
							std::pair<int, int> Rg = getRg(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(g1 / num_col, g1 % num_col), 
								std::make_pair(g2 / num_col, g2 % num_col));
							std::pair<int, int> Rs = getRs(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(s2 / num_col, s2 % num_col),
								std::make_pair(g1 / num_col, g1 % num_col));
							int new_area = (abs(Rs.first - Rg.first) + 1) * (abs(Rs.second - Rg.second) + 1);
							int new_type = classifyRectangleConflict(s1, s2, g1, g2, Rg, num_col);
							if (new_type > type || (new_type == type && new_area > area))
							{
								/*cout << get<0>(*con)<<" " << get<1>(*con) << " " << get<2>(*con) << " " << get<3>(*con) << " " << get<4>(*con) << " " << endl;
								cout << "s1 " << s1 << " s2 " << s2 << " g1 "<< g1 << " g2 "<< g2 << " rg "<< Rg.first<<" "<<Rg.second << endl;
								*/
								conflict = std::shared_ptr<tuple<int, int, int, int, int>>
									(new tuple<int, int, int, int, int>(get<0>(*con), get<1>(*con), -1 - Rg.first * num_col - Rg.second, t1_start, t2_start));
								type = new_type;
								area = new_area;
							}
						}
					}
				}
			}
			if (type == 2)
			{
				parent.rectCardinalConf.push_back(conflict);
			}
			else if (type == 1 && !findRectangleConflict(parent.parent, *conflict))
			{
				parent.rectSemiConf.push_back(conflict);
			}
			else if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
			{
				parent.rectNonConf.push_back(conflict);
			}
		}
	}

	std::shared_ptr<tuple<int, int, int, int, int>> choose;
	int time = INT_MAX;
	if (!parent.rectCardinalConf.empty()) // Choose the earliest rect cardinal
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.rectCardinalConf.begin(); it != parent.rectCardinalConf.end(); ++it)
		{
			int s1 = paths[get<0>(**it)]->at(get<3>(**it)).location;
			int s2 = paths[get<1>(**it)]->at(get<4>(**it)).location;
			std::pair<int, int>  S1 = std::make_pair(s1 / num_col, s1 % num_col);
			std::pair<int, int> S2 = std::make_pair(s2 / num_col, s2 % num_col);
			std::pair<int, int> Rg = std::make_pair(get<2>(**it) / num_col, get<2>(**it) % num_col);
			std::pair<int, int> Rs = getRs(S1, S2, Rg);
			int new_time = get<3>(**it) - abs(S1.first - Rs.first) - abs(S1.second - Rs.second);
			if (new_time < time)
			{
				choose = (*it);
				time = new_time;
			}
		}
	}
	if (!parent.cardinalConf.empty()) // or choose the earliest cardinal
	{
		if (time == INT_MAX)
			choose = parent.cardinalConf.front();
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.cardinalConf.begin(); it != parent.cardinalConf.end(); ++it)
			if (get<4>(**it) < time)
			{
				choose = (*it);
				time = get<4>(**it);
			}
	}
	if (time < INT_MAX)
		return choose;

	if (!parent.rectSemiConf.empty()) // Choose the earliest semi rect
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.rectSemiConf.begin(); it != parent.rectSemiConf.end(); ++it)
		{
			int s1 = paths[get<0>(**it)]->at(get<3>(**it)).location;
			int s2 = paths[get<1>(**it)]->at(get<4>(**it)).location;
			std::pair<int, int>  S1 = std::make_pair(s1 / num_col, s1 % num_col);
			std::pair<int, int> S2 = std::make_pair(s2 / num_col, s2 % num_col);
			std::pair<int, int> Rg = std::make_pair(get<2>(**it) / num_col, get<2>(**it) % num_col);
			std::pair<int, int> Rs = getRs(S1, S2, Rg);
			int new_time = get<3>(**it) - abs(S1.first - Rs.first) - abs(S1.second - Rs.second);
			if (new_time < time)
			{
				choose = (*it);
				time = new_time;
			}
		}
	}
	if (time < INT_MAX)
		return choose;
	if (!parent.semiConf.empty()) // Choose the earliest semi
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.semiConf.begin(); it != parent.semiConf.end(); ++it)
			if (get<4>(**it) < time)
			{
				choose = (*it);
				time = get<4>(**it);
			}
		return choose;
	}
	if (time < INT_MAX)
		return choose;
	if (!parent.rectNonConf.empty()) // Choose a rect randomly
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.rectNonConf.begin(); it != parent.rectNonConf.end(); ++it)
		{
			int s1 = paths[get<0>(**it)]->at(get<3>(**it)).location;
			int s2 = paths[get<1>(**it)]->at(get<4>(**it)).location;
			std::pair<int, int>  S1 = std::make_pair(s1 / num_col, s1 % num_col);
			std::pair<int, int> S2 = std::make_pair(s2 / num_col, s2 % num_col);
			std::pair<int, int> Rg = std::make_pair(get<2>(**it) / num_col, get<2>(**it) % num_col);
			std::pair<int, int> Rs = getRs(S1, S2, Rg);
			int new_time = get<3>(**it) - abs(S1.first - Rs.first) - abs(S1.second - Rs.second);
			if (new_time < time)
			{
				choose = (*it);
				time = new_time;
			}
		}
	}
	if (time < INT_MAX)
		return choose;
	if (!parent.nonConf.empty())// Choose the earliest non
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.nonConf.begin(); it != parent.nonConf.end(); ++it)
			if (get<4>(**it) < time)
			{
				choose = (*it);
				time = get<4>(**it);
			}
	}
	return choose;
}

template<class Map>
bool MultiMapICBSSearch<Map>::findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound)
{
	// extract all constraints on agent ag
	ICBSNode* curr = node;
	vector < list< pair<int, int> > >* cons_vec = collectConstraints(curr, ag);
	// build reservation table
	size_t max_plan_len = node->makespan + 1;
	ReservationTable* res_table = new ReservationTable(map_size,&paths,curr->agent_id);  // initialized to false

	// find a path w.r.t cons_vec (and prioretize by res_table).
	bool foundSol = search_engines[ag]->findPath(node->path, focal_w, cons_vec, res_table, max_plan_len, lowerbound,start,time_limit);
	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;
	delete (cons_vec);
	delete (res_table);
	if (foundSol)
	{
		node->g_val = node->g_val - paths[ag]->size() + node->path.size();
		paths[ag] = &node->path;
		node->makespan = std::max(node->makespan, node->path.size() - 1);
		return true;
	}
	else
	{
		return false;
	}
}

bool ICBSSearch::generateChild(ICBSNode*  node, ICBSNode* curr)
{
	node->parent = curr;
	node->g_val = curr->g_val;
	node->makespan = curr->makespan;
	node->depth = curr->depth + 1;

	std::clock_t t1;

	t1 = std::clock();


	double lowerbound;
	if(get<4>(*curr->conflict) >= (int)paths[node->agent_id]->size()) //conflict happens after agent reaches its goal
		lowerbound = get<4>(*curr->conflict) + 1;
	else 
		lowerbound = (int)paths[node->agent_id]->size() - 1;
		
	if (!findPathForSingleAgent(node, node->agent_id, lowerbound))
		return false;

	
	runtime_lowlevel += std::clock() - t1;
	
	//Estimate h value
	if (node->parent->g_val == node->g_val)
	{
		node->h_val = node->parent->h_val;
	}
	else if (node->parent->h_val > 1)
	{
		node->h_val = node->parent->h_val - 1;		
	}
	else if (!node->cardinalConf.empty() || !node->rectCardinalConf.empty())
	{
		node->h_val = 1;
	}
	else
		node->h_val = 0;
	node->f_val = node->g_val + node->h_val;

	t1 = std::clock();
	findConflicts(*node);

	runtime_conflictdetection += std::clock() - t1;

	node->num_of_collisions = node->unknownConf.size() + node->cardinalConf.size() + node->semiConf.size() + node->nonConf.size() + node->rectCardinalConf.size();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->f_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);


	return true;
}


void ICBSSearch::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (int t = 0; t < paths[i]->size(); t++)
			std::cout << "(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}
}
boost::python::list ICBSSearch::outputPaths()
{
	boost::python::list result;
	for (int i = 0; i < num_of_agents; i++)
	{
		boost::python::list agentPath;


		for (int t = 0; t < paths[i]->size(); t++) {
			boost::python::tuple location = boost::python::make_tuple(paths[i]->at(t).location / num_col, paths[i]->at(t).location % num_col, paths[i]->at(t).actionToHere);
			agentPath.append(location);
		}
		result.append(agentPath);
	}
	return result;
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ICBSSearch::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) 
{
	for (ICBSNode* n : open_list) {
		if (n->f_val > old_lower_bound &&
			n->f_val <= new_lower_bound)
			n->focal_handle = focal_list.push(n);
	}
}

void ICBSSearch::updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node) {
	for (int ag = 0; ag < num_of_agents; ag++) 
	{
		if (ag != exclude_agent && paths[ag] != NULL) 
		{
			for (size_t timestep = 0; timestep < node.makespan + 1; timestep++) 
			{
				int id;
				if (timestep >= paths[ag]->size())
					id = paths[ag]->at(paths[ag]->size() - 1).location;
				else// otherwise, return its location for that timestep
					id = paths[ag]->at(timestep).location;
				res_table[timestep * map_size + id] = true;
			}
		}
	}
}


void ICBSSearch::printStrategy() const
{
	switch (cons_strategy)
	{
	case constraint_strategy::CBS:
		cout << "      CBS: ";
		break;
	case constraint_strategy::ICBS:
		cout << "     ICBS: ";
		break;
	case constraint_strategy::CBSH:
		cout << "     CBSH:";
		break;
	case constraint_strategy::CBSH_R:
		cout << "   CBSH-R:";
		break;
	case constraint_strategy::CBSH_CR:
		cout << "  CBSH-CR:";
		break;
	case constraint_strategy::CBSH_RM:
		cout << "  CBSH-RM:";
		break;
	default:
		exit(10);
	}
}

bool ICBSSearch::runICBSSearch() 
{
	if (debug_mode)
		cout << "Start search" << endl;
	printStrategy();
	// set timer
	start = std::clock();
	std::clock_t t1;
	runtime_computeh = 0;
	runtime_lowlevel = 0;
	runtime_listoperation = 0;
	runtime_conflictdetection = 0;
	runtime_updatepaths = 0;
	runtime_updatecons = 0;
	// start is already in the open_list

	while (!focal_list.empty() && !solution_found) 
	{
		runtime = (std::clock() - start);
		if (runtime > time_limit)
		{  // timeout
			timeout = true;
			cout << "TIMEOUT  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime/CLOCKS_PER_SEC << " ; " << endl;
			break;
		}
		t1 = std::clock();
		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		runtime_listoperation += std::clock() - t1;
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		t1 = std::clock();
		updatePaths(curr);
		runtime_updatepaths += std::clock() - t1;
		if (cons_strategy == constraint_strategy::CBS)
		{
			t1 = std::clock();
			curr->conflict = chooseEarliestConflict(*curr);
			runtime_conflictdetection += std::clock() - t1;
		}
		else if (cons_strategy == constraint_strategy::ICBS) // No heuristics
		{
			t1 = std::clock();
			curr->conflict = classifyConflicts(*curr);
			runtime_conflictdetection += std::clock() - t1;
		}
		else if(curr->conflict == NULL) //CBSH based, and h value has not been computed yet
		{
			t1 = std::clock();
			curr->conflict = classifyConflicts(*curr);
			runtime_conflictdetection += std::clock() - t1;

			t1 = std::clock();
			curr->h_val = computeHeuristics(*curr);
			runtime_computeh += std::clock() - t1;
			curr->f_val = curr->g_val + curr->h_val;

			if (curr->f_val > focal_list_threshold)
			{	
				t1 = std::clock();
				curr->open_handle = open_list.push(curr);
				ICBSNode* open_head = open_list.top();
				if (open_head->f_val > min_f_val) 
				{
					min_f_val = open_head->f_val;
					double new_focal_list_threshold = min_f_val * focal_w;
					updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
					focal_list_threshold = new_focal_list_threshold;
				}
				runtime_listoperation += std::clock() - t1;
				continue;
			}
		}

		if (curr->conflict == NULL) //Fail to find a conflict => no conflicts
		{  // found a solution (and finish the while look)
			runtime = (std::clock() - start); 
			solution_found = true;
			solution_cost = curr->g_val;
			cout << solution_cost << " ; " << solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; ";
			cout << endl;
			break;
		}
		if (debug_mode)
			cout << "choose conflict: " << get<0>(*curr->conflict)  <<" " << get<1>(*curr->conflict)<<" " << "(" << get<2>(*curr->conflict) / num_col << "," << get<2>(*curr->conflict) % num_col << ") " << get<4>(*curr->conflict) << endl;



		 //Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;

		ICBSNode* n1 = new ICBSNode();
		ICBSNode* n2 = new ICBSNode();
			
		n1->agent_id = get<0>(*curr->conflict);
		n2->agent_id = get<1>(*curr->conflict);


		if (get<2>(*curr->conflict) < 0) // Rectangle conflict
		{
			numOfRectangle += 1;
			int Rg = -1 - get<2>(*curr->conflict);
			int S1_t = get<3>(*curr->conflict);
			int S2_t = get<4>(*curr->conflict);
			if (cons_strategy == constraint_strategy::CBSH_RM) // add modified barrier constraints
			{
				
				if (debug_mode) {
					cout << "add modified barrier constraint," << n1->agent_id << " " << n2->agent_id << endl;
				}

				addModifiedBarrierConstraints(*paths[get<0>(*curr->conflict)], *paths[get<1>(*curr->conflict)],
					S1_t, S2_t, Rg, num_col, n1->constraints, n2->constraints,kDelay);
			}
			else // add barrier constraints
			{
				//get<0>(*curr->conflict) agent1 id, get<3>(*curr->conflict) agent1 conflict time
				int S1 = paths[get<0>(*curr->conflict)]->at(get<3>(*curr->conflict)).location;
				int S2 = paths[get<1>(*curr->conflict)]->at(get<4>(*curr->conflict)).location;
				int G1 = paths[get<0>(*curr->conflict)]->back().location;
				int G2 = paths[get<1>(*curr->conflict)]->back().location;

				if (debug_mode) {
					cout << "add barrier constraint," << n1->agent_id << " "<< n2->agent_id << endl;
				}
				if (kDelay == 0) {
					addBarrierConstraints(S1, S2, S1_t, S2_t, Rg, num_col, n1->constraints, n2->constraints);
				}
				else {
					if (shortBarrier)
						addShortKDelayBarrierConstraints(S1, S2, S1_t, S2_t, Rg, num_col, n1->constraints, n2->constraints, kDelay);
					else
						addKDelayBarrierConstraints(S1, S2, S1_t, S2_t, Rg, G1, G2, num_col, n1->constraints, n2->constraints, kDelay, asymmetry_constraint);
				}
			}

		}
		else if (get<3>(*curr->conflict) < 0) // vertex conflict
		{
			if (asymmetry_constraint) {
				//n1 is the agent that at time t, n2 is the agent between t and t + k, 
				//thus for a asymmetric range constraint, constraint n1 is a single time constraint,
				//constraint n2 is the range constraint from t-kdelay to t+kdelay
				if (debug_mode) {
					cout << "add asy range constraint" << endl;
				}
				n1->constraints.push_back(make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict)));
				for (int i = -kDelay; i <= kDelay; i++) {
					if ((get<4>(*curr->conflict) + i) < 0)
						continue;
					n2->constraints.push_back(make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict) + i));
				}

			}
			else {
				if (debug_mode) {
					cout << "add sym range constraint" << endl;
				}
				for (int i = 0; i <= kDelay; i++) {
					n1->constraints.push_back(make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict) + i));
					n2->constraints.push_back(make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict) + i));
				}
			}

		}
		else // edge conflict
		{
			//if (asymmetry_constraint) {
			//	if (debug_mode)
			//		cout << "add asy range edge constraint" << endl;
			//	n1->constraints.push_back(make_tuple(get<2>(*curr->conflict), get<3>(*curr->conflict), get<4>(*curr->conflict)));
			//	for (int i = -kDelay; i <= kDelay; i++) {
			//		if ((get<4>(*curr->conflict) + i) < 0)
			//			continue;
			//		n2->constraints.push_back(make_tuple(get<3>(*curr->conflict), get<2>(*curr->conflict), get<4>(*curr->conflict) + i));
			//	}
			//}
			//else {
			//	if (debug_mode) 
			//		cout << "add sym range edge constraint" << endl;
			//	for (int i = 0; i <= kDelay; i++) {
			n1->constraints.push_back(make_tuple(get<2>(*curr->conflict), get<3>(*curr->conflict), get<4>(*curr->conflict) ));
			n2->constraints.push_back(make_tuple(get<3>(*curr->conflict), get<2>(*curr->conflict), get<4>(*curr->conflict) ));
			//	}
			//}
			//
		}

		bool Sol1 = false, Sol2 = false;
		vector<vector<PathEntry>*> copy(paths);
		if (debug_mode)
			cout << "generate child 1" << endl;

		//Agent1 is the base agent for a k-delay conflict. Thus if conflict time is 0, means agent 1's initial location
		//conflict with agent 2's 0+1~0+k location. Thus we only diable node 1 when k-delay conflict happen on time 0.
		

		if (get<4>(*curr->conflict) != 0 || get<2>(*curr->conflict) < 0 || ignore_t0)
			Sol1 = generateChild(n1, curr);
		else if (debug_mode)
			cout << "Time 0 conflict, ignore node 1" << endl;


		paths = copy;
		if (debug_mode)
			cout << "generate child 2" << endl;
		Sol2 = generateChild(n2, curr);

		if(!Sol1)
		{
			delete (n1);
			n1 = NULL;
		}
		if(!Sol2)
		{
			delete (n2);
			n2 = NULL;
		}
		curr->clear();
		t1 = std::clock();
		if (open_list.size() == 0) {
			solution_found = false;
			break;
		}
		ICBSNode* open_head = open_list.top();
		if (open_head->f_val > min_f_val) 
		{
			min_f_val = open_head->f_val;
			double new_focal_list_threshold = min_f_val * focal_w;
			updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
			focal_list_threshold = new_focal_list_threshold;
		}
		runtime_listoperation += std::clock() - t1;

	}  // end of while loop


	if (focal_list.empty() && solution_cost < 0)
	{
		solution_cost = -2;
		cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
			HL_num_expanded << " ; " << HL_num_generated << " ; " <<
			LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; " <<
			"|Open|=" << open_list.size() << endl;
		solution_found = false;
	}
	if (debug_mode)
		printPaths();
	return solution_found;
}


template<class Map>
void MultiMapICBSSearch<Map>::initializeDummyStart() {
	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;

	if (debug_mode)
		cout << "Initializing first solutions" << endl;
	// initialize paths_found_initially
	paths.resize(num_of_agents, NULL);
	paths_found_initially.resize(num_of_agents);
	ReservationTable* res_table = new ReservationTable(map_size);  // initialized to false
	for (int i = 0; i < num_of_agents; i++) {

		//cout << "agent " << i << endl;
		if (search_engines[i]->findPath(paths_found_initially[i], focal_w, NULL, res_table, dummy_start->makespan + 1, 0) == false)
			cout << "NO SOLUTION EXISTS";

		paths[i] = &paths_found_initially[i];
		res_table->addPath(i, paths[i]);
		dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);

		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;

	}
	delete (res_table);



	if (debug_mode)
		cout << "Initializing dummy start" << endl;
	// generate dummy start and update data structures	
	dummy_start->g_val = 0;
	for (int i = 0; i < num_of_agents; i++)
		dummy_start->g_val += paths[i]->size() - 1;
	dummy_start->h_val = 0;
	dummy_start->f_val = dummy_start->g_val;

	dummy_start->depth = 0;

	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);

	HL_num_generated++;
	dummy_start->time_generated = HL_num_generated;
	allNodes_table.push_back(dummy_start);
	findConflicts(*dummy_start);

	min_f_val = dummy_start->f_val;
	focal_list_threshold = min_f_val * focal_w;
	if (debug_mode)
		cout << "Initializing done" << endl;
}

inline void ICBSSearch::releaseClosedListNodes() 
{
	for (list<ICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete *it;
}

inline void ICBSSearch::releaseOpenListNodes()
{
	while(!open_list.empty())
	{
		ICBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}

template<class Map>
MultiMapICBSSearch<Map>::~MultiMapICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	releaseClosedListNodes();
}

template<class Map>
MultiMapICBSSearch<Map>::MultiMapICBSSearch(Map* ml, AgentsLoader& al, double f_w, constraint_strategy c, int time_limit, int kDlay, options options1)
{
	this->focal_w = f_w;
	this->time_limit = time_limit;

	if (debug_mode)
		cout << "Initializing CBS" << endl;
	cons_strategy = c;
	HL_num_expanded = 0;
	HL_num_generated = 0;
	LL_num_expanded = 0;
	LL_num_generated = 0;
	this->num_col = ml->cols;
	this->al = al;
	num_of_agents = al.num_of_agents;
	map_size = ml->rows*ml->cols;
	solution_found = false;
	solution_cost = -1;
	kDelay = kDlay;
	asymmetry_constraint = options1.asymmetry_constraint;
	debug_mode = options1.debug;
	ignore_t0 = options1.ignore_t0;
	shortBarrier = options1.shortBarrier;
	search_engines = vector < SingleAgentICBS<Map>* >(num_of_agents);
	if (debug_mode)
		cout << "Initializing search engines" << endl;
	for (int i = 0; i < num_of_agents; i++) {
		int init_loc = ml->linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml->linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic<Map> ch(init_loc, goal_loc, ml, al.headings[i]);
		search_engines[i] = new SingleAgentICBS<Map>(init_loc, goal_loc, ml,i, al.headings[i],kDelay);
		ch.getHVals(search_engines[i]->my_heuristic);
		/*if (debug_mode) {
			std::cout << "Heuristic table for " << i << ": ";
			for (int h = 0; h < search_engines[i]->my_heuristic.size(); h++) {
				for (int heading = 0; heading < 5; heading++)
					if (search_engines[i]->my_heuristic[h].heading[heading]<INT_MAX)
					std::cout << "(" << h << ": "<<heading<<": " << search_engines[i]->my_heuristic[h].heading[heading] << ")";
			}
			std::cout << std::endl;

		}*/

	}

	if (debug_mode) {
		cout << "Initializing search engines done" << endl;
	}
	initializeDummyStart();
}

template<class Map>
void MultiMapICBSSearch<Map>::buildMDD(ICBSNode& curr, int id)
{
	MDD<Map> * mdd = new MDD<Map>();

	vector < list< pair<int, int> > >* cons_vec = collectConstraints(&curr, id);
	mdd->buildMDD(*cons_vec, paths[id]->size(), *search_engines[id]);

	for (int i = 0; i < mdd->levels.size(); i++)
		paths[id]->at(i).single = mdd->levels[i].size() == 1;
	if (cons_strategy == constraint_strategy::CBSH_RM)
	{
		for (int i = 0; i < mdd->levels.size(); i++)
		{
			for (MDDNode* n : mdd->levels[i])
			{
				paths[id]->at(i).locations.push_back(n->location);
			}
		}
	}
	delete mdd;
	delete cons_vec;
}

template class MultiMapICBSSearch<MapLoader>;
template class MultiMapICBSSearch<FlatlandLoader>;



