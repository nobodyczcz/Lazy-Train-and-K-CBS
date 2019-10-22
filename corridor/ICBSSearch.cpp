#include "ICBSSearch.h"
#include <ctime>
#include <iostream>


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr)
{
	for(int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr->parent != NULL)
	{
		for (list<pair<int, vector<PathEntry>>>::iterator it = curr->paths.begin(); it != curr->paths.end() ; ++it)
		{
			if (!updated[it->first])
			{
				paths[it->first] = &(it->second);
				updated[it->first] = true;
			}
		}
		curr = curr->parent;
	}
}




void ICBSSearch::updateConstraintTable(ICBSNode* curr, int agent_id)
{
	constraintTable.clear();
	constraintTable.goal_location = search_engines[agent_id]->goal_location;
	while (curr != dummy_start)
	{
		if (curr->agent_id == agent_id)
		{
			for (auto constraint : curr->constraints)
			{
				int x, y, z;
				constraint_type type;
				tie(x, y, z, type) = constraint;
				if (type == constraint_type::RANGE) // time range constraint
				{
					constraintTable.insert(x, y, z + 1);
				}
				else if (type == constraint_type::BARRIER) // barrier constraint
				{
					int x1 = x / num_col, y1 = x % num_col;
					int x2 = y / num_col, y2 = y % num_col;
					if (x1 == x2)
					{
						if (y1 < y2)
							for (int i = 0; i <= std::min(y2 - y1, z); i++)
							{
								constraintTable.insert(x1 * num_col + y2 - i, z - i, z - i + 1);
							}
						else
							for (int i = 0; i <= std::min(y1 - y2, z); i++)
							{
								constraintTable.insert(x1 * num_col + y2 + i, z - i, z - i + 1);
							}
					}
					else // y1== y2
					{
						if (x1 < x2)
							for (int i = 0; i <= std::min(x2 - x1, z); i++)
							{
								constraintTable.insert((x2 - i) * num_col + y1, z - i, z - i + 1);
							}
						else
							for (int i = 0; i <= std::min(x1 - x2, z); i++)
							{
								constraintTable.insert((x2 + i) * num_col + y1, z - i, z - i + 1);
							}
					}
				}
				else if (type == constraint_type::LENGTH)
				{
					if (x < 0 && y == agent_id)
					{ // <-1, agent_id, t>: path of agent_id should be of length at least t + 1 
						constraintTable.length_min = max(constraintTable.length_min, z + 1);
					}
					else if (x >= 0 && y == agent_id)
					{ // <loc, agent_id, t>: path of agent_id should be of length at most t
						constraintTable.length_max = min(constraintTable.length_max, z);
					}
					else if (x >= 0 && y != agent_id)
					{ // <loc, agent_id, t>: any other agent cannot be at loc at or after timestep t
						constraintTable.insert(x, z, INT_MAX);
					}
				}
				else if (type == constraint_type::VERTEX)
				{
					constraintTable.insert(x, z, z +1);
				}
				else // edge
				{
					constraintTable.insert(x * map_size + y, z, z + 1);
				}
			}
		}
		else if (!curr->constraints.empty())
		{
			int x, y, z;
			constraint_type type;
			tie(x, y, z, type) = curr->constraints.front();
			if (type == constraint_type::LENGTH && x >=0 && y != agent_id)
			{
				constraintTable.insert(x, z, INT_MAX);
			}
		}
		curr = curr->parent;
	}
}

/*std::vector <std::list< std::pair<int, int> > >* ICBSSearch::collectConstraints(ICBSNode* curr, int agent_id)
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
				if(-1 - get<2>(constraint) > max_timestep) // calc constraints' max_timestep
					max_timestep = -1 - get<2>(constraint);
			}
		}
		curr = curr->parent;
	}


	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > >(max_timestep + 1, list< pair<int, int> >());
	
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) 
	{
		if (get<2>(*it) < 0) // time range constraint
		{
			int loc = get<0>(*it);
			int t1 = -1 - get<1>(*it);
			int t2 = -1 - get<2>(*it);			
			for (int i = t1; i <= t2; i++)
				cons_vec->at(i).push_back(make_pair(loc, -1));
		}
		else if (get<0>(*it) < 0) // barrier constraint
		{
			int x1 = (-get<0>(*it) - 1) / num_col, y1 = (-get<0>(*it) - 1) % num_col;
			int x2 = get<1>(*it) / num_col, y2 = get<1>(*it) % num_col;
			if (x1 == x2)
			{
				if (y1 < y2)
					for (int i = 0; i <= std::min(y2 - y1, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair(x1 * num_col + y2 - i, -1));
				else
					for (int i = 0; i <= std::min(y1 - y2, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair(x1 * num_col + y2 + i, -1));
			}
			else // y1== y2
			{
				if (x1 < x2)
					for (int i = 0; i <= std::min(x2 - x1, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair((x2 - i) * num_col + y1, -1));
				else
					for (int i = 0; i <= std::min(x1 - x2, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair((x2 + i) * num_col + y1, -1));
			}
		}
		else
			cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));
	}
	
	runtime_updatecons += std::clock() - t1;
	return cons_vec;
}*/

int ICBSSearch::computeHeuristics(const ICBSNode& curr)
{
	// Conflict graph
	vector<vector<bool>> CG(num_of_agents);
	int num_of_CGnodes = 0, num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
		CG[i].resize(num_of_agents, false);
	for (auto conflict : curr.conflicts)
	{
		if(conflict->p == conflict_priority::CARDINAL && !CG[conflict->a1][conflict->a2])
		{
			CG[conflict->a1][conflict->a2] = true;
			CG[conflict->a2][conflict->a1] = true;
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
void ICBSSearch::copyConflicts(const std::list<std::shared_ptr<Conflict >>& conflicts,
	std::list<std::shared_ptr<Conflict>>& copy, const list<int>& excluded_agents) const
{
	for (auto conflict : conflicts)
	{
		bool found = false;
		for (auto a : excluded_agents)
		{
			if (conflict->a1 == a || conflict->a2 == a)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			copy.push_back(conflict);
		}
	}
}

//void ICBSSearch::copyConflicts(const std::list<std::shared_ptr<CConflict >>& conflicts,
//	std::list<std::shared_ptr<CConflict>>& copy, int excluded_agent) const
//{
//	for (std::list<std::shared_ptr<CConflict>>::const_iterator it = conflicts.begin(); it != conflicts.end(); ++it)
//	{
//		if (get<0>(**it) != excluded_agent && get<1>(**it) != excluded_agent)
//		{
//			copy.push_back(*it);
//		}
//	}
//}

void ICBSSearch::findConflicts(ICBSNode& curr, int a1, int a2)
{
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	for (size_t timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2)
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			if (targetReasoning && paths[a1]->size() == timestep + 1)
			{
				conflict->targetConflict(a1, a2, loc1, timestep);
			}
			else if (targetReasoning && paths[a2]->size() == timestep + 1)
			{
				conflict->targetConflict(a2, a1, loc1, timestep);
			}
			else
			{
				conflict->vertexConflict(a1, a2, loc1, timestep);
			}

			curr.unknownConf.push_back(conflict);
		}
		else if (timestep < min_path_length - 1
			&& loc1 == paths[a2]->at(timestep + 1).location
			&& loc2 == paths[a1]->at(timestep + 1).location)
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->edgeConflict(a1, a2, loc1, loc2, timestep + 1);
			curr.unknownConf.push_back(conflict); // edge conflict
		}
	}
	if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{
				std::shared_ptr<Conflict> conflict(new Conflict());
				if (targetReasoning)
					conflict->targetConflict(a1_, a2_, loc1, timestep);
				else
					conflict->vertexConflict(a1_, a2_, loc1, timestep);
				curr.unknownConf.push_front(conflict); // It's at least a semi conflict			
			}
		}
	}
}


void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (curr.parent != NULL)
	{
		// Copy from parent、
		list<int> new_agents;
		for (auto p : curr.paths)
		{
			new_agents.push_back(p.first);
		}
		copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

		// detect new conflicts
		for (list<int>::iterator it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it; 
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (a1 == a2)
					continue;
				bool skip = false;
				for (list<int>::iterator it2 = new_agents.begin(); it2 != it; ++it2)
				{
					if (*it2 == a2)
					{
						skip = true;
						break;
					}
				}
				findConflicts(curr, a1, a2);
			}
		}
	}
	else
	{
		for(int a1 = 0; a1 < num_of_agents ; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
}

//void ICBSSearch::deleteRectConflict(ICBSNode& curr, const Conflict& conflict)
//{
//	list<shared_ptr<Conflict>>::iterator it;
//	if (!curr.rectCardinalConf.empty())
//	{
//		for (it = curr.rectCardinalConf.begin(); it != curr.rectCardinalConf.end(); ++it)
//		{
//			if ((**it) == conflict)
//			{
//				curr.rectCardinalConf.erase(it);
//				return;
//			}
//		}
//	}
//	if (!curr.rectSemiConf.empty())
//	{
//		for (it = curr.rectSemiConf.begin(); it != curr.rectSemiConf.end(); ++it)
//		{
//			if ((**it) == conflict)
//			{
//				curr.rectSemiConf.erase(it);
//				return;
//			}
//		}
//	}
//	if (!curr.rectNonConf.empty())
//	{
//		for (it = curr.rectNonConf.begin(); it != curr.rectNonConf.end(); ++it)
//		{
//			if ((**it) == conflict)
//			{
//				curr.rectNonConf.erase(it);
//				return;
//			}
//		}
//	}
//}

MDD* ICBSSearch::buildMDD(ICBSNode& node, int id)
{
	MDD * mdd = NULL;
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		MDDTable::const_iterator got = mddTable[c.a].find(c);
		if (got != mddTable[c.a].end())
		{
			mdd = got->second;
		}
	}
	if (mdd == NULL)
	{
		mdd = new MDD();
		// vector < list< pair<int, int> > >* cons_vec = collectConstraints(&node, id);
		updateConstraintTable(&node, id);
		mdd->buildMDD(constraintTable, paths[id]->size(), *search_engines[id]);
	}
	
	for(int i = 0; i < mdd->levels.size(); i++)
			paths[id]->at(i).single = mdd->levels[i].size() == 1;
	if(cons_strategy == constraint_strategy::CBSH_RM || cons_strategy == constraint_strategy::CBSH_GR)
	{
		for (int i = 0; i < mdd->levels.size(); i++)
		{
			for (MDDNode* n : mdd->levels[i])
			{
				paths[id]->at(i).locations.push_back(n->location);
			}
		}
	}
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		mddTable[c.a][c] = mdd;
	}
	return mdd;
}


bool ICBSSearch::isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node)
{
	int a[2] = {con->a1, con->a2};
	int  loc1, loc2, timestep;
	constraint_type type;
	std::tie(loc1, loc2, timestep, type) = con->constraint1.back();
	int curr = -1;
	if (getDegree(loc1, my_map, num_col, map_size) == 2)
	{
		curr = loc1;
		if (loc2 >= 0)
			timestep--;
	}
	else if (getDegree(loc2, my_map, num_col, map_size) == 2)
		curr = loc2;
	if (curr <= 0)
		return false;
	
	int t[2];
	for (int i = 0; i < 2; i++)
		t[i] = getEnteringTime(*paths[a[i]], *paths[a[1-i]], timestep, my_map, num_col, map_size);
	if (t[0] > t[1])
	{
		int temp = t[0]; t[0] = t[1]; t[1] = temp;
		temp = a[0]; a[0] = a[1]; a[1] = temp;
	}
	int u[2];
	for (int i = 0; i < 2; i++)
		u[i] = paths[a[i]]->at(t[i]).location;
	if (u[0] == u[1])
		return false;
	for (int i = 0; i < 2; i++)
	{
		bool found = false;
		for (int time = t[i]; time < paths[a[i]]->size() && !found; time++)
		{
			if (paths[a[i]]->at(time).location == u[1 - i])
				found = true;
		}
		if (!found)
			return false;
	}
	std::pair<int, int> edge; // one edge in the corridor
	int k = getCorridorLength(*paths[a[0]], t[0], u[1], edge);
	
	if (corridor2)
	{
		std::pair<int, int> edge_empty = make_pair(-1, -1);
		updateConstraintTable(node, a[0]);
		int t3 = getBypassLength(paths[a[0]]->front().location, u[1], edge_empty, my_map, num_col, map_size, constraintTable, INT_MAX);
		int t3_ = getBypassLength(paths[a[0]]->front().location, u[1], edge, my_map, num_col, map_size, constraintTable, t3 + 2 * k  + 1);
		updateConstraintTable(node, a[1]);
		int t4 = getBypassLength(paths[a[1]]->front().location, u[0], edge_empty, my_map, num_col, map_size, constraintTable, INT_MAX);
		int t4_ = getBypassLength(paths[a[1]]->front().location, u[0], edge, my_map, num_col, map_size, constraintTable, t3 + k + 1);
		if (abs(t3 - t4) <= k && t3_ > t3 && t4_ > t4)
		{

			corridor = std::shared_ptr<Conflict>(new Conflict());
			corridor->corridorConflict(a[0], a[1], u[1], u[0], t3, t4, t3_, t4_, k);
			if (blocked(*paths[corridor->a1], corridor->constraint1) && blocked(*paths[corridor->a2], corridor->constraint2))
				return true;
		}
	}
	
	if (!corridor4)
		return false;
	else if (cardinalCorridorReasoning && !cardinal)
		return false;
	// try 4-way splitting then
	if (t[1] > t[0] + 2 * k - 1)
		return false;

	//get h
	std::tuple<int, int, int> key;
	if (u[0] < u[1])
	{
		key = std::make_tuple(u[0], u[1], k);
	}
	else
	{
		key = std::make_tuple(u[1], u[0], k);
	}
	
	CorridorTable::const_iterator got = corridorTable.find(key);
	int h;
	if (got == corridorTable.end())
	{
		h = getBypassLength(u[0], u[1], edge, my_map, num_col, map_size);
		corridorTable[key] = h;
	}
	else
	{
		h = got->second;
	}
	if (h > 0 && k >= h - 1)
		return false;
	
	for (int i = 0; i < 2; i++)
	{
		bool satisfied = false;
		for (int j = t[0] + k; j <= t[0] + std::min(2 * k, h - 1); j++)
		{
			if (getLocation(*paths[a[i]], j) == u[1-i])
			{
				satisfied = true;
				break;
			}
		}
		if (!satisfied)
			return false;
	}

	corridor = std::shared_ptr<Conflict>(new Conflict());
	corridor->corridorConflict(a[0], a[1], u[0], u[1], t[0], t[1], k, h);
	// TODO: test whether block all paths
	return true;
}

void ICBSSearch::classifyConflicts(ICBSNode &parent)
{
	if (parent.conflicts.empty() && parent.unknownConf.empty())
		return; // No conflict

	// Classify all conflicts in unknownConf
	while (!parent.unknownConf.empty())
	{
		std::shared_ptr<Conflict> con = parent.unknownConf.front();
		int a1 = con->a1, a2 = con->a2;
		int loc1, loc2, timestep;
		constraint_type type;
		std::tie(loc1, loc2, timestep, type) = con->constraint1.back();
		parent.unknownConf.pop_front();

		bool cardinal1 = false, cardinal2 = false;
		if (timestep >= paths[a1]->size())
			cardinal1 = true;
		else if (!paths[a1]->at(0).single)
		{
			MDD* mdd = buildMDD(parent, a1);
			for (int i = 0; i < mdd->levels.size(); i++)
				paths[a1]->at(i).single = mdd->levels[i].size() == 1;
			if (mddTable.empty())
				delete mdd;
		}
		if (timestep >= paths[a2]->size())
			cardinal2 = true;
		else if (!paths[a2]->at(0).single)
		{
			MDD* mdd = buildMDD(parent, a2);
			for (int i = 0; i < mdd->levels.size(); i++)
				paths[a2]->at(i).single = mdd->levels[i].size() == 1;
			if (mddTable.empty())
				delete mdd;
		}

		if (type == conflict_type::STANDARD && loc2 >= 0) // Edge conflict
		{
			cardinal1 = paths[a1]->at(timestep).single && paths[a1]->at(timestep - 1).single;
			cardinal2 = paths[a2]->at(timestep).single && paths[a2]->at(timestep - 1).single;
		}
		else // vertex conflict or target conflict
		{
			if (!cardinal1)
				cardinal1 = paths[a1]->at(timestep).single;
			if (!cardinal2)
				cardinal2 = paths[a2]->at(timestep).single;
		}

		if (cardinal1 && cardinal2)
		{
			con->p = conflict_priority::CARDINAL;
		}
		else if (cardinal1 || cardinal2)
		{
			con->p = conflict_priority::SEMI;
		}
		else
		{
			con->p = conflict_priority::NON;
		}
		if (con->p == conflict_priority::CARDINAL && cons_strategy == constraint_strategy::ICBS)
		{
			parent.conflicts.push_back(con);
			return;
		}
			
		if (con->type == conflict_type::TARGET)
		{
			parent.conflicts.push_back(con);
			continue;
		}

		// Corridor reasoning
		std::shared_ptr<Conflict> corridor;
		if ((corridor2 || corridor4) && isCorridorConflict(corridor, con, cardinal1 && cardinal2, &parent))
		{
			corridor->p = con->p;
			parent.conflicts.push_back(corridor);
			continue;
		}

		if (con->type == conflict_type::STANDARD &&
			paths[con->a1]->size() <= con->t || paths[con->a2]->size() <= con->t) //conflict happens after agent reaches its goal
		{
			parent.conflicts.push_back(con);
			continue;
		}

		//Rectangle reasoning for semi and non cardinal vertex conflicts
		if (rectangleMDD)
		{
			std::list<int>	s1s = getStartCandidates(*paths[a1], timestep, num_col);
			std::list<int>	g1s = getGoalCandidates(*paths[a1], timestep, num_col);
			std::list<int>	s2s = getStartCandidates(*paths[a2], timestep, num_col);
			std::list<int>	g2s = getGoalCandidates(*paths[a2], timestep, num_col);

			// Try all possible combinations
			bool found = false;
			std::shared_ptr<Conflict> rectangle; // = std::shared_ptr<Conflict>(new Conflict());
			int type = -1;
			int area = 0;
			for (int t1_start : s1s)
			{
				for (int t1_end : g1s)
				{
					int s1 = paths[a1]->at(t1_start).location;
					int g1 = paths[a1]->at(t1_end).location;
					if (!isManhattanOptimal(s1, g1, t1_end - t1_start, num_col))
						continue;
					for (int t2_start : s2s)
					{
						for (int t2_end : g2s)
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
								auto new_rectangle = std::shared_ptr<Conflict>(new Conflict());						
								int Rg_t = timestep + abs(Rg.first - loc1 / num_col) + abs(Rg.second - loc1 % num_col);
								new_rectangle->rectangleConflict(a1, a2, Rs, Rg, 
									make_pair(s1 / num_col, s1 % num_col), make_pair(s2 / num_col, s2 % num_col), Rg_t, paths, num_col);
								if (blocked(*paths[new_rectangle->a1], new_rectangle->constraint1) && blocked(*paths[new_rectangle->a2], new_rectangle->constraint2))
								{
									rectangle = new_rectangle;
									if (new_type == 2)
										rectangle->p = conflict_priority::CARDINAL;
									else if (new_type == 1) // && !findRectangleConflict(parent.parent, *conflict))
										rectangle->p = conflict_priority::SEMI;
									else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
										rectangle->p = conflict_priority::NON;
									type = new_type;
									area = new_area;
								}
								// rectangle = std::shared_ptr<tuple<int, int, int, int, int>>
								//	(new tuple<int, int, int, int, int>(get<0>(*con), get<1>(*con), -1 - Rg.first * num_col - Rg.second, t1_start, t2_start));						
							}
						}
					}
				}
			}
			if (type >= 0)
			{
				parent.conflicts.push_back(rectangle);
				continue;
			}
		}
		else if (cons_strategy == constraint_strategy::CBSH_GR) // generalized rectangles 
		{
			if (loc2 >= 0) // Edge conflict
				continue;
			if (paths[a1]->size() <= timestep || paths[a2]->size() <= timestep)//conflict happens after agent reaches its goal
				continue;

			int from1 =(*paths[a1])[timestep - 1].location;
			int from2 = (*paths[a2])[timestep - 1].location;
			if (from1 == from2 || // same direction
				from1 == loc1 || from2 == loc1 || //wait actions
				abs(from1 - from2) == 2 || abs(from1 - from2) == num_col * 2) //opposite direction
				continue;
			
			std::list<int>	s1s = getStartCandidates(*paths[a1], timestep, loc1 - from1, loc1 - from2);
			std::list<int>	g1s = getGoalCandidates(*paths[a1], timestep, loc1 - from1, loc1 - from2);
			std::list<int>	s2s = getStartCandidates(*paths[a2], timestep, loc1 - from1, loc1 - from2);
			std::list<int>	g2s = getGoalCandidates(*paths[a2], timestep, loc1 - from1, loc1 - from2);

			
			MDD* mdd1 = NULL, *mdd2 = NULL;
			ConstraintsHasher c(a1, &parent);
			MDDTable::const_iterator got = mddTable[c.a].find(c);
			if (got != mddTable[c.a].end())
			{
				mdd1 = got->second;
			}
			else
			{
				std::cout << "ERROR" << std::endl;
			}
			std::list<Constraint> B1;
			bool haveBarriers = ExtractBarriers(*mdd1, loc1 - from1, loc1 - from2, paths[a1]->at(s1s.back()).location, paths[a1]->at(g1s.back()).location, s1s.back(), num_col, B1);
			if (!haveBarriers)
				continue;
			
			c.a = a2;
			got = mddTable[c.a].find(c);
			if (got != mddTable[c.a].end())
			{
				mdd2 = got->second;
			}
			else
			{
				std::cout << "ERROR" << std::endl;
			}
			std::list<Constraint> B2;
			 haveBarriers = ExtractBarriers(*mdd2, loc1 - from2, loc1 - from1, paths[a2]->at(s2s.back()).location, paths[a2]->at(g2s.back()).location, s2s.back(), num_col, B2);
			 if (!haveBarriers)
				 continue;

			// Try all possible combinations
			int type = -1;
			list<Constraint>::const_iterator b1_entry = B1.cbegin();
			list<Constraint>::const_iterator b2_entry = B2.cbegin();
			std::pair<int, int> Rs, Rg;
			std::set<std::pair<int, int>> visitedRs;
			generalizedRectangle(*paths[a1], *paths[a2], *mdd1, *mdd2, b1_entry, b2_entry, B1, B2, timestep, num_col, type, Rs, Rg, std::clock() * CLOCKS_PER_SEC * 1.0 / 1000  + time_limit - runtime, visitedRs);
			if (type < 0)
				continue;
			int Rg_t = timestep + abs(Rg.first - loc1 / num_col) + abs(Rg.second - loc1 % num_col);
			std::shared_ptr<Conflict> rectangle = std::shared_ptr<Conflict>(new Conflict());
			rectangle->rectangleConflict(a1, a2, Rs, Rg, loc1 - from1, loc1 - from2, Rg_t, paths, num_col);
			if (type == 2)
				rectangle->p = conflict_priority::CARDINAL;
			else if (type == 1) // && !findRectangleConflict(parent.parent, *conflict))
				rectangle->p = conflict_priority::SEMI;
			else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
				rectangle->p = conflict_priority::NON;
			if (blocked(*paths[rectangle->a1], rectangle->constraint1) && blocked(*paths[rectangle->a2], rectangle->constraint2))
			{
				parent.conflicts.push_back(rectangle);
				continue;
			}
		}
		parent.conflicts.push_back(con);
	}

	// remove conflicts that cannot be chosen, to save some memory
	removeLowPriorityConflicts(parent.conflicts);
}

void ICBSSearch::removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts) const
{
	if (conflicts.empty())
		return;
	std::unordered_map<int, std::shared_ptr<Conflict> > keep;
	std::list<std::shared_ptr<Conflict>> to_delete;
	for (auto conflict : conflicts)
	{
		int a1 = conflict->a1, a2 = conflict->a2;
		int key = a1 * num_of_agents + a2;
		auto p = keep.find(key);
		if (p == keep.end())
		{
			keep[key] = conflict;
		}
		else if (*(p->second) < *conflict)
		{
			to_delete.push_back(p->second);
			keep[key] = conflict;
		}
		else
		{
			to_delete.push_back(conflict);
		}
	}

	for (auto conflict : to_delete)
	{
		conflicts.remove(conflict);
	}
}


bool ICBSSearch::traverse(const Path& path, int loc, int t) const
{
	if (t >= path.size())
		if (loc == path.back().location)
			return true;
		else
			return false;
	else if (t >= 0 && path[t].location == loc)
		return true;
	else 
		return false;
}


bool ICBSSearch::blocked(const Path& path, const std::list<Constraint>& constraints) const
{
	for (auto constraint : constraints)
	{
		int x, y, t;
		constraint_type type;
		tie(x, y, t, type) = constraint;
		if (type == constraint_type::RANGE) // time range constraint
		{
			for (int i = y; i < t; i++)
			{
				if (traverse(path, x, i))
					return true;
			}
		}
		else if (type == constraint_type::BARRIER) // barrier constraint
		{
			int x1 = x / num_col, y1 = x % num_col;
			int x2 = y / num_col, y2 = y % num_col;
			if (x1 == x2)
			{
				if (y1 < y2)
				{
					for (int i = 0; i <= std::min(y2 - y1, t); i++)
					{
						if (traverse(path, x1 * num_col + y2 - i, t - i))
							return true;
					}
				}
				else
				{
					for (int i = 0; i <= std::min(y1 - y2, t); i++)
					{
						if (traverse(path, x1 * num_col + y2 + i, t - i))
							return true;
					}
				}
			}
			else // y1== y2
			{
				if (x1 < x2)
				{
					for (int i = 0; i <= std::min(x2 - x1, t); i++)
					{
						if (traverse(path, (x2 - i) * num_col + y1, t - i))
							return true;
					}
				}
				else
				{
					for (int i = 0; i <= std::min(x1 - x2, t); i++)
					{
						if (traverse(path, (x2 + i) * num_col + y1, t - i))
							return true;
					}
				}
			}
		}
	}
	return false;
}


std::shared_ptr<Conflict> ICBSSearch::chooseConflict(ICBSNode &parent)
{

	std::shared_ptr<Conflict> choose;
	if (parent.conflicts.empty() && parent.unknownConf.empty())
		return NULL;
	else if (!parent.conflicts.empty())
	{
		choose = parent.conflicts.back();
		for (auto conflict : parent.conflicts)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	else
	{
		choose = parent.conflicts.back();
		for (auto conflict : parent.unknownConf)
		{
			if (conflict->t < choose->t)
				choose = conflict;
		}
	}
	return choose;
}

bool ICBSSearch::findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound)
{
	// extract all constraints on agent ag
	ICBSNode* curr = node;
	// vector < list< pair<int, int> > >* cons_vec = collectConstraints(curr, ag);
	updateConstraintTable(curr, ag);
	// build reservation table
	size_t max_plan_len = node->makespan + 1;
	bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
	updateReservationTable(res_table, ag, *node);
	// find a path
	vector<PathEntry> newPath;
	// cout << "**************" << endl;
	// cout << "Single agent : " << curr->agent_id << endl;
	bool foundSol = search_engines[ag]->findPath(newPath, focal_w, constraintTable, res_table, max_plan_len, lowerbound);
	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;
	// delete (cons_vec);
	delete[] res_table;
	if (foundSol)
	{
		node->paths.emplace_back(ag, newPath);
		node->g_val = node->g_val - paths[ag]->size() + newPath.size();
		paths[ag] = &node->paths.back().second;
		node->makespan = std::max(node->makespan, newPath.size() - 1);
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
	if (std::get<0>(node->constraints.front()) >= 0 &&
		std::get<3>(node->constraints.front()) == constraint_type::LENGTH)
	{
		int x, agent, t;
		constraint_type type;
		tie(x, agent, t, type) = node->constraints.front();
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
			{
				continue;
			}
			if (t < paths[ag]->size() && paths[ag]->at(t).location == x)
			{
				double lowerbound = (int)paths[ag]->size() - 1;
				if (!findPathForSingleAgent(node, ag, lowerbound))
					return false;
			}
		}
	}
	else
	{
		double lowerbound = (int)paths[node->agent_id]->size() - 1;
		// if (curr->conflict->p == conflict_priority::CARDINAL && curr->conflict->type != conflict_type::CORRIDOR2)
		//	lowerbound += 1;
		if (!findPathForSingleAgent(node, node->agent_id, lowerbound))
			return false;
	}




	

	
	runtime_lowlevel += (std::clock() - t1) * 1000.0 / CLOCKS_PER_SEC;
	
	//Estimate h value
	if (node->parent->g_val == node->g_val)
	{
		node->h_val = node->parent->h_val;
	}
	else if (node->parent->h_val > 1)
	{
		node->h_val = node->parent->h_val - 1;		
	}
	// else if (hasCardinalConflict(*node))
	// {
	// 	node->h_val = 1;
	// }
	else
		node->h_val = 0;
	node->f_val = node->g_val + node->h_val;

	t1 = std::clock();
	findConflicts(*node);
	runtime_conflictdetection += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;

	node->num_of_collisions = node->unknownConf.size() + node->conflicts.size();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->f_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);


	return true;
}

bool ICBSSearch::hasCardinalConflict(const ICBSNode& node) const
{
	for (auto conflict : node.conflicts)
	{
		if (conflict->p = conflict_priority::CARDINAL)
			return true;
	}
	return false;
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
	case constraint_strategy::CBSH_GR:
		cout << "  CBSH-GR:";
		break;
	default:
		exit(10);
	}
}

bool ICBSSearch::runICBSSearch() 
{
	printStrategy();
	// set timer
	std::clock_t start;
	start = std::clock();
	std::clock_t t1;
	runtime_computeh = 0;
	runtime_lowlevel = 0;
	runtime_listoperation = 0;
	runtime_conflictdetection = 0;
	runtime_updatepaths = 0;
	runtime_updatecons = 0;
	// start is already in the open_list

	generated_root_node();


	while (!focal_list.empty() && !solution_found) 
	{
		runtime = (std::clock() - start) * 1000.0 / CLOCKS_PER_SEC;
		if (runtime > time_limit)
		{  // timeout
			cout << "TIMEOUT  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << 
				num_standard << ";" << num_rectangle << "," <<
				num_corridor2 << ";" << num_corridor4 << "," << num_target << endl;
			break;
		}
		t1 = std::clock();
		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		runtime_listoperation += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		t1 = std::clock();
		updatePaths(curr);
		runtime_updatepaths += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;

		if (cons_strategy == constraint_strategy::CBS)
		{
			t1 = std::clock();
			curr->conflict = chooseConflict(*curr);
			runtime_conflictdetection += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;
		}
		else if (cons_strategy == constraint_strategy::ICBS) // No heuristics
		{
			t1 = std::clock();
			classifyConflicts(*curr);
			curr->conflict = chooseConflict(*curr);
			runtime_conflictdetection += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;
		}
		else if(curr->conflict == NULL) //CBSH based, and h value has not been computed yet
		{
			t1 = std::clock();
			classifyConflicts(*curr);
			curr->conflict = chooseConflict(*curr);
			runtime_conflictdetection += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;

			t1 = std::clock();
			curr->h_val = computeHeuristics(*curr);
			runtime_computeh += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;
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
				runtime_listoperation += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;
				continue;
			}
		}

		if (curr->conflict == NULL) //Fail to find a conflict => no conflicts
		{  // found a solution (and finish the while look)
			runtime = (std::clock() - start) * 1000.0 / CLOCKS_PER_SEC;
			solution_found = true;
			solution_cost = curr->g_val;
			if (screen == 2)
				printPaths();
			cout << solution_cost << " ; " << solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << 
				num_standard << ";" << num_rectangle << "," <<
				num_corridor2 << ";" << num_corridor4 << "," << num_target << endl;
			break;
		}

		 //Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;


		vector<ICBSNode*> children;

		if (curr->conflict->type == conflict_type::CORRIDOR4) // 4-way branching
		{
			children.resize(4);
			children[0] = new ICBSNode(curr->conflict->a1);
			children[0]->constraints.push_back(curr->conflict->constraint1.front());
			children[1] = new ICBSNode(curr->conflict->a1);
			children[1]->constraints.push_back(curr->conflict->constraint1.back());
			children[2] = new ICBSNode(curr->conflict->a2);
			children[2]->constraints.push_back(curr->conflict->constraint2.front());
			children[3] = new ICBSNode(curr->conflict->a2);
			children[3]->constraints.push_back(curr->conflict->constraint2.back());
			num_corridor4++;
		}
		else // 2-way branching
		{
			children.resize(2);
			children[0] = new ICBSNode(curr->conflict->a1);
			children[0]->constraints = curr->conflict->constraint1;
			children[1] = new ICBSNode(curr->conflict->a2);
			children[1]->constraints = curr->conflict->constraint2;
			if (curr->conflict->type == conflict_type::CORRIDOR2)
				num_corridor2++;
			else if (curr->conflict->type == conflict_type::STANDARD)
				num_standard++;
			else if (curr->conflict->type == conflict_type::RECTANGLE)
				num_rectangle++;
			else if (curr->conflict->type == conflict_type::TARGET)
				num_target++;
		}

		if (screen == 2)
		{
			std::cout << "Node " << curr->time_generated << " with f=" << curr->g_val << "+" << curr->h_val
				<< " has " << std::endl;  
			for (auto conflict : curr->conflicts)
				std::cout << *conflict;
			std::cout << "We choose " << *curr->conflict;
		}
		vector<vector<PathEntry>*> copy(paths);
		for (int i = 0; i < children.size(); i++)
		{
			if (generateChild(children[i], curr))
			{
				if (screen == 2)
					std::cout << "Generate #" << children[i]->time_generated
						<< " with cost " << children[i]->g_val
						<< " and " << children[i]->num_of_collisions << " conflicts " << std::endl;
			}
			else
			{
				delete children[i];
			}
			if (i < children.size() - 1)
				paths = copy;
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
		runtime_listoperation += (std::clock() - t1) * 1000.0 / CLOCKS_PER_SEC;

	}  // end of while loop


	if (focal_list.empty() && solution_cost < 0)
	{
		solution_cost = -2;
		cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
			HL_num_expanded << " ; " << HL_num_generated << " ; " <<
			LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << 
			num_standard << ";" << num_rectangle << ";" <<
			num_corridor2 << ";" << num_corridor4 << ";" << num_target << ";" <<
			"|Open|=" << open_list.size() << endl;
		solution_found = false;
	}
	return solution_found;
}


bool ICBSSearch::generated_root_node()
{
	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;


	// initialize paths_found_initially
	paths.resize(num_of_agents, NULL);
	paths_found_initially.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) {
		// cout<<"******************************"<<endl;
		// cout<<"Agent: "<<i<<endl;

		bool* res_table = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
		updateReservationTable(res_table, i, *dummy_start);

		if (search_engines[i]->findPath(paths_found_initially[i], focal_w, constraintTable, res_table, dummy_start->makespan + 1, 0) == false)
		{
			cout << "NO SOLUTION EXISTS";
			return false;
		}

		paths[i] = &paths_found_initially[i];
		dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);

		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
		delete[] res_table;
	}



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

	if (cons_strategy == constraint_strategy::CBSH_GR)
		mddTable.resize(num_of_agents);

		return true;
}

ICBSSearch::ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, constraint_strategy c, int time_limit, int screen): 
	focal_w(f_w), time_limit(time_limit), screen(screen)
{
	cons_strategy = c;
	HL_num_expanded = 0;
	HL_num_generated = 0;
	LL_num_expanded = 0;
	LL_num_generated = 0;
	this->num_col = ml.cols;
	this->al = al;
	this->my_map = ml.my_map;
	num_of_agents = al.num_of_agents;
	map_size = ml.rows*ml.cols;
	solution_found = false;
	solution_cost = -1;

	search_engines = vector < SingleAgentICBS* >(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) {
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.moves_offset);
		search_engines[i] = new SingleAgentICBS(init_loc, goal_loc, ml.get_map(), ml.rows*ml.cols,
			ml.moves_offset, ml.cols);
		ch.getHVals(search_engines[i]->my_heuristic);
	}
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

ICBSSearch::~ICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	releaseClosedListNodes();
	if (!mddTable.empty())
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			for (auto mdd : mddTable[i])
			{
				delete mdd.second;
			}
		}
	}
}
