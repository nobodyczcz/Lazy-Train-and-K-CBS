
#include "ICBSSearch.h"
#include "CorridorReasoning.h"
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

void ICBSSearch::conflict_between_2(ICBSNode& curr, int a1, int a2)
{
    size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
    for (size_t timestep = 0; timestep < min_path_length; timestep++)
    {
        for (int k = -kDelay; k<=kDelay;k++){
            int  t = timestep+k;
            if (t<0 || t >= paths[a1]->size() || t >= paths[a2]->size())
                continue;
            int loc1 = paths[a1]->at(t).location;
            int loc2 = paths[a2]->at(t).location;
            if (loc1 == loc2)
            {
                cout<<"Find "<<k<<" delay conflict between "<< a1 <<" and "<<a2<<endl;

            }
            else if (kDelay==0 && timestep < min_path_length - 1
                     && loc1 == paths[a2]->at(timestep + 1).location
                     && loc2 == paths[a1]->at(timestep + 1).location)
            {
                cout<<"Find edge conflict between "<< a1 <<" and "<<a2<<endl;

            }
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
                cout<<"Find target conflict between "<< a1_ <<" and "<<a2_<<endl;
            }
        }
    }
}

void ICBSSearch::finalConflictCheck(ICBSNode& curr)
{
        for(int a1 = 0; a1 < num_of_agents ; a1++) {
            for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
                conflict_between_2(curr, a1, a2);
            }
        }
        cout<<"Final conflict check done"<<endl;
}



void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (&curr != this->dummy_start)
	{
		if (debug_mode)
			cout << "copy from parent" << endl;
		// Copy from parent、
		list<int> new_agents;
		for (auto p : curr.paths)
		{
			new_agents.push_back(p.first);
		}
		copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);
        copyConflicts(curr.parent->cardinal_waiting, curr.cardinal_waiting, new_agents);
        copyConflicts(curr.parent->non_cardinal_waiting, curr.non_cardinal_waiting, new_agents);

		if (debug_mode)
			cout << "Find Conflicts" << endl;
		// detect new conflicts
		for (list<int>::iterator it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it;
			//collect conflict from path;
			for (size_t t = 0; t < paths[a1]->size(); t++) {
				if (paths[a1]->at(t).conflist != NULL && paths[a1]->at(t).conflist->size() != 0) {
					for (auto con : *(paths[a1]->at(t).conflist)) {
					    if (option.window_size >0 && get<4>(*con)>option.window_size)
					        continue;
						if (debug_mode)
							cout << "l<" << get<0>(*con) << "," << get<1>(*con) << ","
							<< "(" << get<2>(*con) / num_col << "," << get<2>(*con) % num_col << ")" << ","
							<< "(" << get<3>(*con) / num_col << "," << get<3>(*con) % num_col << ")" << ","
							<< get<4>(*con) << "," << get<5>(*con) << ">; ";


						if (option.ignore_target && (get<3>(*con) < 0) && (get<4>(*con) > paths[get<0>(*con)]->size() - 1 + kDelay)){
						    continue;
						}

                        std::shared_ptr<Conflict> newConf(new Conflict());

                        if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) >= paths[get<0>(*con)]->size() - 1)) {
							newConf->targetConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con)+ get<5>(*con), kDelay);
						}
						//else if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) >= paths[get<1>(*con)]->size() - 1)) {
						//	newConf->targetConflict(get<1>(*con), get<0>(*con), get<2>(*con), get<4>(*con)+ get<5>(*con), kDelay);
						//}
						else if (get<3>(*con) < 0) {
							if(get<4>(*con) >= paths[get<0>(*con)]->size() - 1)
								newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con) + get<5>(*con),0, kDelay);
							else
								newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), get<5>(*con), kDelay);
						}
						else {
							newConf->edgeConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<3>(*con), get<4>(*con));
						}

						curr.unknownConf.emplace_back(newConf);

                    }
					delete paths[a1]->at(t).conflist;
				}
			}

            if(option.ignore_target)
                continue;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{//low level search can't find target conflict if a1<a2
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
				findTargetConflicts(a1, a2, curr);

			}
		}
	}
	else
	{
		if (debug_mode)
			cout << "Find Conflicts" << endl;
		for (int a1 = 0; a1 < num_of_agents; a1++)
		{

			//collect conflicts from path
			if(paths[a1]->empty())
			    continue;
			for (size_t t = 0; t < paths[a1]->size(); t++) {

				if (paths[a1]->at(t).conflist == NULL || paths[a1]->at(t).conflist->size() == 0)
					continue;

				for (auto& con : *(paths[a1]->at(t).conflist)) {
                    if (option.window_size >0 && get<4>(*con)>option.window_size)
                        continue;

                    if (option.ignore_target && (get<3>(*con) < 0) && (get<4>(*con) > paths[get<0>(*con)]->size() - 1 + kDelay)){
                        continue;
                    }

					std::shared_ptr<Conflict> newConf(new Conflict());

					if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) > paths[get<0>(*con)]->size() - 1)) {
						newConf->targetConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), kDelay);
					}
					//else if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) > paths[get<1>(*con)]->size() - 1)) {
					//	newConf->targetConflict(get<1>(*con), get<0>(*con), get<2>(*con), get<4>(*con), kDelay);
					//}
					else if (get<3>(*con) < 0) {
						if (get<4>(*con) >= paths[get<0>(*con)]->size() - 1)
							newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con) + get<5>(*con), 0, kDelay);
						else
							newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), get<5>(*con), kDelay);
					}
					else {
						newConf->edgeConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<3>(*con), get<4>(*con));
					}
					if (debug_mode)
					cout << "<" << get<0>(*con) << "," << get<1>(*con) << ","
						<< "(" << get<2>(*con) / num_col << "," << get<2>(*con) % num_col << ")" << ","
						<< "(" << get<3>(*con) / num_col << "," << get<3>(*con) % num_col << ")" << ","
						<< get<4>(*con) << "," << get<5>(*con) << ">; ";
					curr.unknownConf.emplace_back(newConf);

                }
				delete paths[a1]->at(t).conflist;
			}

            if(option.ignore_target)
                continue;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{//low level search can't find target conflict if a1<a2
				if (a1 == a2)
					continue;
				findTargetConflicts(a1, a2, curr);
				
			}
		}
	}
	if (debug_mode)
		cout  << endl;
}

void ICBSSearch::findTargetConflicts(int a1, int a2, ICBSNode& curr) {
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	//collect conflict from path;
	if (a1 == a2) {
		return;
	}


	if (paths[a1]->size() < paths[a2]->size())
	{
		//short one a1_ longer one a2_
		//current short after goal code should work good for k robust
		int a1_ = a1;
		int a2_ = a2;
		int loc1 = paths[a1_]->back().location;// short one's goal location
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			//for (int k = 0; k <= kDelay; k++) {
			//	//in future longer one can't pass through the goal location of shorter one.
			//	if (timestep + k >= paths[a2_]->size())
			//		continue;
				int loc2 = paths[a2_]->at(timestep).location;
				if (loc1 == loc2)
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					if (targetReasoning)
					{
						conflict->targetConflict(a1_, a2_, loc1, timestep, kDelay);
					}
					else
					{
						conflict->vertexConflict(a1_, a2_, loc1, timestep,0,kDelay);
					}
					if (debug_mode)
						cout << "<" << a1_ << "," << a2_ << ","
						<< "(" << loc1 / num_col << "," << loc1 % num_col << ")" << ","
						<< "(,)" << ","
						<< timestep << "," "0" << ">; ";
					curr.unknownConf.push_back(conflict);
				}
			//}
			
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




template<class Map>
bool MultiMapICBSSearch<Map>::isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, ICBSNode* node)
{
	if (screen >= 4) {
		cout << "Check is corridor conflict" << endl;
		cout << *con << endl;
	}
	CorridorReasoning<Map> cp;
	int a[2] = {con->a1, con->a2};
	int  loc1, loc2, timestep;
	constraint_type type;
	int kConflict = con->k;
	std::tie(loc1, loc2, timestep, type) = con->constraint1.back();
	if (kDelay > 0)
		timestep = con->t;
	//cout << "loc1 " << con->originalConf1<< " loc2 "<< con->originalConf2 << " timestep " << con->t << " k " << kConflict << endl;
	// << "loc1 " << loc1 << " loc2 " << loc2 << " timestep " << timestep << " k " << kConflict << endl;

	int curr = -1;
	if (ml->getDegree(loc1) == 2)
	{
		curr = loc1;
		if (loc2 >= 0)
			timestep--;
	}
	else if (ml->getDegree(loc2) == 2)
		curr = loc2;
	if (curr <= 0 || !ml->notCorner(curr))
		return false;
	//cout << "iscorridor2" << endl;

	int t[2];
	t[0] = cp.getEnteringTime(*paths[a[0]], *paths[a[1-0]], timestep, ml);
	t[1] = cp.getEnteringTime(*paths[a[1]], *paths[a[1 - 1]], timestep+kConflict, ml);

    int e[2];
    e[0] = cp.getExitTime(*paths[a[0]], *paths[a[1-0]], timestep, ml);
    e[1] = cp.getExitTime(*paths[a[1]], *paths[a[1 - 1]], timestep+kConflict, ml);

	if (t[0] > t[1])
	{
		int temp = t[0]; t[0] = t[1]; t[1] = temp;
		temp = a[0]; a[0] = a[1]; a[1] = temp;
        temp = e[0]; e[0] = e[1]; e[1] = temp;

    }




    int u[2];//get entering location
	for (int i = 0; i < 2; i++)
		u[i] = paths[a[i]]->at(t[i]).location;
	if (u[0] == u[1])
		return false;
	//cout << "iscorridor3" << endl;

	for (int i = 0; i < 2; i++)//check does one entering location lead to another's entering location
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
	//cout << "iscorridor4" << endl;

	if (corridor2)
	{
		if (screen >= 4) {
			cout << "Compute corridor2 B E" << endl;
		}

		std::pair<int, int> edge_empty = make_pair(-1, -1);
		updateConstraintTable(node, a[0]);
		if (screen>=4)
			cout << "start: " << paths[a[0]]->front().location/num_col<<","<< paths[a[0]]->front().location % num_col << " end:" << u[1]/num_col<<"," << u[1]%num_col << " heading:" << paths[a[0]]->front().actionToHere << endl;
		int t3 = cp.getBypassLength(paths[a[0]]->front().location, u[1], edge_empty, ml, num_col, map_size, constraintTable, INT_MAX, search_engines[a[0]]->my_heuristic, paths[a[0]]->front().heading,paths[a[0]]->at(e[0]).heading,search_engines[a[0]]->kRobust);
		int t3_ = cp.getBypassLength(paths[a[0]]->front().location, u[1], edge, ml, num_col, map_size, constraintTable, t3 + 2 * k  + 1,search_engines[a[0]]->my_heuristic, paths[a[0]]->front().heading, paths[a[0]]->at(e[0]).heading, search_engines[a[0]]->kRobust);
		

		updateConstraintTable(node, a[1]);
		if (screen >= 4)
			cout << "start: " << paths[a[1]]->front().location / num_col << "," << paths[a[1]]->front().location % num_col << " end:" << u[0] / num_col << "," << u[0] % num_col << " heading:" << paths[a[1]]->front().actionToHere << endl;
		int t4 = cp.getBypassLength(paths[a[1]]->front().location, u[0], edge_empty, ml, num_col, map_size, constraintTable, INT_MAX,search_engines[a[1]]->my_heuristic, paths[a[1]]->front().heading, paths[a[1]]->at(e[1]).heading,search_engines[a[1]]->kRobust);
		int t4_ = cp.getBypassLength(paths[a[1]]->front().location, u[0], edge, ml, num_col, map_size, constraintTable, t4 + 2*k + 1, search_engines[a[1]]->my_heuristic, paths[a[1]]->front().heading, paths[a[1]]->at(e[1]).heading,search_engines[a[1]]->kRobust);
		if (screen >= 4) {
			cout << "t3: " << t3 << "," << "t3_: " << t3_ << "," << "t4: " << t4 << "," << "t4_: " << t4_ << endl;
			cout << "corridor length: " << k << endl;
		}
		//cout << k << endl;
		if (abs(t3 - t4) <= k+kDelay && t3_ > t3 && t4_ > t4)
		{
			//cout << "iscorridor5.5" << endl;

			corridor = std::shared_ptr<Conflict>(new Conflict());
			corridor->corridorConflict(a[0], a[1], u[1], u[0], t3, t4, t3_, t4_, k,kDelay);
			if (blocked(*(paths[corridor->a1]), corridor->constraint1) && blocked(*(paths[corridor->a2]), corridor->constraint2))
				return true;
			else {
				if(screen>=4)
					cout << "not blocked" << endl;
			}
		}
		else {
			if (screen>=4)
			cout << "t check not pass" << endl;
		}
		//cout << "iscorridor6" << endl;

	}
	
	return false;
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
	else if (t >= 0 && path[t].location == loc) {
		//cout << "t: " << t << " loc: " << loc << " path: " << path[t].location << endl;
		return true;

	}
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
		//cout << x << "," << y << "," << t << endl;
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
		else {
			if (traverse(path, x, t))
				return true;
		}
	}
	return false;
}


std::shared_ptr<Conflict> ICBSSearch::chooseConflict(ICBSNode &parent)
{
	if (debug_mode)
		cout << "Start choosing conflict" << endl;

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
		choose = parent.unknownConf.back();
		for (auto conflict : parent.unknownConf)
		{
			if (conflict->t < choose->t)
				choose = conflict;
		}
	}

	return choose;
}


bool ICBSSearch::generateChild(ICBSNode*  node, ICBSNode* curr)
{
	if (debug_mode)
		cout << "generate child" << endl;
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

			if (t > paths[ag]->size()) {
				continue;
			}
			bool replan = false;
			/*for (int k = 0; k <= kDelay; k++) {
				if (t + k < paths[ag]->size() && paths[ag]->at(t + k).location == x) {
					replan = true;
				}
			}*/
			for (int tg = t; tg < paths[ag]->size(); tg++) {
				if (paths[ag]->at(tg).location == x) {
					replan = true;
					break;
				}
			}

			if (replan)
			{
			    if (screen>=2){
			        cout<<"Replan for agent: "<< ag<<endl;
			    }
				double lowerbound = (int)paths[ag]->size() - kDelay - 1; //The shrink at goal is added after search complete
				if (!findPathForSingleAgent(node, ag, lowerbound))
					return false;
			}
		}
	}
	else
	{
		double lowerbound = (int)paths[node->agent_id]->size() - kDelay-1;//The shrink at goal is added after search complete
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
			std::cout << t <<"(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}
}

bool ICBSSearch::isValidTrain()
{
    bool valid = true;
    unordered_map<int,unordered_map<int,int>> rt;
    for (int i = 0; i < num_of_agents; i++)
    {

        for (int t = 0; t < paths[i]->size(); t++){

            for (int loc : (*paths[i])[t].occupations) {
                if (!rt.count(loc))
                    rt[loc] = unordered_map<int,int>();

                if (!rt[loc].count(t))
                    rt[loc][t]=i;
                else {
                        cout<<i<<" body conflict t "<< t<<" with "<<rt[loc][t]<<endl;
                    if (i == rt[loc][t])
                        num_self_conflict++;
                    num_body_conflict++;
                    valid = false;
                }

            }

        }

    }
    return valid;
}

void ICBSSearch::printPaths(Path& path) const
{

		for (int t = 0; t < path.size(); t++){
            std::cout << t;
            for (int loc: path[t].occupations){
                std::cout <<"(" << loc / num_col << "," << loc % num_col << ")";
            }
            std::cout <<"->";
        }
		std::cout << std::endl;
}

void ICBSSearch::printBT(const std::string& prefix, const ICBSNode* node, bool isLeft)
{
	if (node != NULL)
	{
		std::cout << prefix;
		std::cout << (isLeft ? "├──" : "└──");
		if (node->conflict != nullptr) {

			// print the value of the node
			std::cout << "<" << node->conflict->a1 << " "
				<< node->conflict->a2 << ",("
				<< node->conflict->originalConf1/num_col<<","
				<<node->conflict->originalConf1 % num_col<<")" << ",("
				<< node->conflict->originalConf2 / num_col << ","
				<< node->conflict->originalConf2 % num_col << ")"
				<< node->conflict->t  <<","<< node->conflict->k<<","<<node->conflict->type<< ">" << std::endl;

			// enter the next tree level - left and right branch

		}
		else {
			std::cout << "No choosen conflict" << std::endl;
		}

		for (int i = 0; i < node->children.size(); i++) {
			printBT(prefix + (isLeft ? "│   " : "    "), node->children[i], i == node->children.size() - 1 ? false: true);
		}
	}
}

void ICBSSearch::printHLTree()
{
	printBT("", dummy_start, false);
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

template<class Map>
bool MultiMapICBSSearch<Map>::search(){
    std::clock_t t1;
    if(!analysisInstance && !option.print_nodes)
        printStrategy();
    // set timer

    runtime_computeh = 0;
    runtime_lowlevel = 0;
    runtime_listoperation = 0;
    runtime_conflictdetection = 0;
    runtime_updatepaths = 0;
    runtime_updatecons = 0;
    // start is already in the open_list

    if (debug_mode)
        cout << "Start searching:" << endl;
    if (screen >= 3)
        al.printAgentsInitGoal();
    while (!focal_list.empty() && !solution_found)
    {
        runtime = (std::clock() - start);
        if (runtime > time_limit || (node_limit!=0 && HL_num_expanded >node_limit))
        {  // timeout
            if(!analysisInstance)
            cout << "TIMEOUT  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
                 HL_num_expanded << " ; " << HL_num_generated << " ; " <<
                 LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; "
                 << RMTime/CLOCKS_PER_SEC<<";"<<
                 num_standard << ";" << num_rectangle << "," <<
                 num_corridor2 << ";" << num_corridor4 << "," << num_target << "," << num_0FlipRectangle << "," <<
                 num_1FlipRectangle << "," << num_2FlipRectangle <<","<<num_chasingRectangle <<"," <<
                 less10 << ","<<less100 << ","<<less1000 << ","<<less10000 <<"," << less100000 <<","<<
                 larger100000 << "," <<num_pairs <<"," <<num_failed << endl;
            if(debug_mode)
                printHLTree();
            if (screen >= 1)
                printPaths();
            timeout = true;
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

        if (screen > 2) {
            cout << "#############" << endl;
            cout << "Choose node with cost " << curr->g_val
                 << " , h " << curr->h_val
                 << " , f " << curr->f_val
                 << endl;
        }

        if (cons_strategy == constraint_strategy::CBS)
        {
            t1 = std::clock();
            curr->conflict = chooseConflict(*curr);
            runtime_conflictdetection += std::clock() - t1;
        }
        else if (cons_strategy == constraint_strategy::ICBS) // No heuristics
        {
            t1 = std::clock();
            classifyConflicts(*curr);
            curr->conflict = chooseConflict(*curr);
            runtime_conflictdetection += std::clock() - t1;
        }
        else if(curr->conflict == NULL) //CBSH based, and h value has not been computed yet
        {
            t1 = std::clock();
            classifyConflicts(*curr);
            curr->conflict = chooseConflict(*curr);
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
                if ( open_head->f_val > min_f_val)
                {
                    min_f_val = open_head->f_val;
                    double new_focal_list_threshold = min_f_val * focal_w;
                    updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
                    focal_list_threshold = new_focal_list_threshold;
                    if (screen > 3) {
                        cout << "new focal threadhold: " << focal_list_threshold << endl;
                    }
                }
                runtime_listoperation += std::clock() - t1;
                continue;
            }
        }

        if(option.print_nodes){
            cout<<"********************************"<<endl;
            cout<<"Expand Node #: "<<curr->time_generated<<endl;
            if (curr->parent!=NULL)
                cout<<"Parent #: "<<curr->parent->time_generated<<endl;
            cout<<"g value: "<< curr->g_val<<endl;
            cout<<"h value: "<< curr->h_val<<endl;
            cout<<"Classified conflicts: ";
            for (auto conflict : curr->conflicts)
                std::cout << *conflict;
            if (curr->conflict == NULL)
                cout<<"No conflicts. This is the conflict free goal node"<<endl;
            else
                cout<<"We choose conflict: "<<*curr->conflict;

        }

        if (curr->conflict == NULL) //Fail to find a conflict => no conflicts
        {  // found a solution (and finish the while look)
            runtime = (std::clock() - start);
            solution_found = true;
            solution_cost = curr->g_val;
            if (debug_mode) {
                printHLTree();
                finalConflictCheck(*curr);
            }
            if (screen >= 1)
                printPaths();
            if(!analysisInstance)
                cout << solution_cost << " ; " << solution_cost - dummy_start->g_val << " ; " <<
                 HL_num_expanded << " ; " << HL_num_generated << " ; " <<
                 LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; "
                 << RMTime / CLOCKS_PER_SEC << ";"<<
                 num_standard << ";" << num_rectangle << "," <<
                 num_corridor2 << ";" << num_corridor4 << "," << num_target << "," << num_0FlipRectangle << "," <<
                 num_1FlipRectangle << "," << num_2FlipRectangle << "," << num_chasingRectangle << "," <<
                 less10 << ","<<less100 << ","<<less1000 << ","<<less10000 <<"," << less100000 <<","<<
                 larger100000 << "," <<num_pairs <<"," <<num_failed << endl;

            break;
        }

        if (debug_mode) {
            cout << "****************" << endl;
            cout << "choose conflict: " << "<" << curr->conflict->a1 << " "
                 << curr->conflict->a2 << ",("
                 << curr->conflict->originalConf1 / num_col << ","
                 << curr->conflict->originalConf1 % num_col << ")" << ",("
                 << curr->conflict->originalConf2 / num_col << ","
                 << curr->conflict->originalConf2 % num_col << ")"
                 << curr->conflict->t << "," << curr->conflict->k << "," << curr->conflict->type << ">" << std::endl;
            if (screen>=3)
                printPaths();

        }


        if (screen == 1 || screen==6 ) {
            if(debug_mode)
                cout << "check conflict repeatance" << endl;
            stringstream con;
            //con << *(curr->conflict);
            if (curr->conflict->k == 0) {
                con << min(curr->conflict->a1, curr->conflict->a2) << ",";
                con << max(curr->conflict->a1, curr->conflict->a2);
            }
            else {
                con << curr->conflict->a1 << ",";
                con << curr->conflict->a2;
            }
            con << ",("
                << curr->conflict->originalConf1 / num_col << ","
                << curr->conflict->originalConf1 % num_col << ")" << ",("
                << curr->conflict->originalConf2 / num_col << ","
                << curr->conflict->originalConf2 % num_col << "),"
                << curr->conflict->t<<"," << curr->conflict->k<<","<<curr->conflict->type ;


            curr->resolvedConflicts.insert(con.str());

            bool stop = false;
            bool noRepeat = true;
            ICBSNode* parent = curr->parent;
            if (parent != NULL) {
                if (debug_mode)
                    cout << "Try find " << con.str() << " in curr's parent nodes" << endl;
                while (!stop) {
                    if (debug_mode)
                        cout << "1";
                    if (parent->parent == NULL) {
                        stop = true;
                        break;
                    }
                    std::unordered_set<std::string>::const_iterator it = parent->resolvedConflicts.find(con.str());
                    if (it != parent->resolvedConflicts.end()) {
                        noRepeat = false;
                        printHLTree();
                    }
                    if (!noRepeat) {
                        cout << "Repeatance detected!!!" << endl;
                        exit(1);
                    }
                    parent = parent->parent;

                }
            }
            else {
                if (debug_mode)
                    cout << "no parent" << endl;

            }
            if (noRepeat) {
                if (debug_mode)
                    cout << "no repeatance" << endl;
            }
            else {
                if (debug_mode)
                    cout << "repeatance detected" << endl;
            }

        }

        //Expand the node
        HL_num_expanded++;
        curr->time_expanded = HL_num_expanded;
        if (curr->conflict->type == conflict_type::RECTANGLE) {

            numOfRectangle += 1;
        }

        if (screen >= 2)
        {
            std::cout << "Node " << curr->time_generated << " with f=" << curr->g_val << "+" << curr->h_val
                      << " has " << std::endl;
            for (auto conflict : curr->conflicts)
                std::cout << *conflict;
            std::cout << "We choose " << *curr->conflict;
        }

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
            //else if (curr->conflict->type == conflict_type::STANDARD && curr->conflict->t == 0 && !option.ignore_t0) {

            //	children.resize(1);
            //	children[0] = new ICBSNode(curr->conflict->a2);
            //	children[0]->constraints = curr->conflict->constraint2;
            //	num_standard++;
            //}
        else if (curr->conflict->type == conflict_type::RECTANGLE4) // 4-way branching
        {
            children.resize(curr->conflict->multiConstraint1.size()+ curr->conflict->multiConstraint2.size());
            int constraint1Size = curr->conflict->multiConstraint1.size();
            int constraint2Size = curr->conflict->multiConstraint2.size();


            if (screen >= 2) {
                cout << "Generate " << children.size() << " child nodes" << endl;
                cout << constraint1Size << " nodes"<< " for a1" << endl;
                cout << constraint2Size << " nodes"<< " for a2" << endl;
            }

            for (int i = 0; i < constraint1Size; i++) {
                children[i]= new ICBSNode(curr->conflict->a1);
                children[i]->constraints = curr->conflict->multiConstraint1[i];
            }

            for (int i = 0; i < constraint2Size; i++) {
                children[i+ constraint1Size] = new ICBSNode(curr->conflict->a2);
                children[i+ constraint1Size]->constraints = curr->conflict->multiConstraint2[i];
            }

            num_rectangle++;
            if (curr->conflict->isChasing) {
                num_chasingRectangle++;
            }

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
            else if (curr->conflict->type == conflict_type::RECTANGLE) {
                if (curr->conflict->flipType == 1) {
                    num_1FlipRectangle++;
                }
                else if (curr->conflict->flipType == 2) {
                    num_2FlipRectangle++;
                }
                else {
                    num_0FlipRectangle++;
                }
                num_rectangle++;
            }
            else if (curr->conflict->type == conflict_type::TARGET)
                num_target++;
        }


        vector<vector<PathEntry>*> copy(paths);
        for (int i = 0; i < children.size(); i++)
        {
            if (generateChild(children[i], curr))
            {
                curr->children.push_back(children[i]);
                if (screen >= 2)
                    std::cout << "Generate child "<<i<<" #" << children[i]->time_generated
                              << " with cost " << children[i]->g_val
                              << " and h "<< children[i]->h_val
                              << " and " << children[i]->num_of_collisions << " conflicts " << std::endl;
                if (option.pairAnalysis){
                    for(int a=0; a < num_of_agents;a++){
                        if (a!=children[i]->agent_id){
                            startPairAnalysis(children[i], children[i]->agent_id, a);
                        }
                    }
                }
            }
            else
            {
                if (screen >= 2)
                    std::cout << "Delete child "<< i <<" #" << children[i]->time_generated
                              << " with cost " << children[i]->g_val
                              << " and " << children[i]->num_of_collisions << " conflicts " << std::endl;
                delete children[i];
            }
            if (i < children.size() - 1)
                paths = copy;
        }
        if (debug_mode) {
            for (int i = 0; i < curr->children.size(); i++)
            {

                cout <<"Child "<<i<< " conflicts:"<< curr->children[i]->unknownConf.size();

                cout << endl;
                printPaths(curr->children[i]->paths.front().second);

            }
        }
        if(option.print_nodes){
            for (int i = 0; i < curr->children.size(); i++)
            {
                cout<<endl;
                cout <<"*** Generate Child Node #: "<<curr->children[i]->time_generated<<endl;
                cout <<"Add new constraints: ";
                for (auto constraint : curr->children[i]->constraints){
                    int x, y, z;
                    constraint_type type;
                    tie(x, y, z, type) = constraint;
                    string string_type;
                    if(type == 0)
                        string_type = "L";
                    else if(type == 1)
                        string_type = "R";
                    else if(type == 2)
                        string_type = "B";
                    else if(type == 3)
                        string_type = "V";
                    else if(type == 4)
                        string_type = "E";
                    cout<<"<"<<curr->children[i]->agent_id <<"," << x << ","<< y << "," << z << "," << string_type  <<">";
                }
                cout<<endl;
                cout <<"Find new path: "<<endl;
                for (auto p: curr->children[i]->paths){
                    cout<<"Agent id: "<<p.first<<" :";
                    for (auto loc: p.second){
                        cout << "("<<loc.location<<")->";
                    }
                    cout<<endl;
                }
                cout<<"Find new unclassified conflict: ";
                for (auto c: curr->children[i]->unknownConf){
                    cout << "<" << c->a1 << " "
                         << c->a2 << ","
                         << c->originalConf1  << ","
                         << curr->conflict->originalConf2 << ","
                         << curr->conflict->t << "," << curr->conflict->type << ">" ;
                }
                cout<<endl;

            }
        }
//        if (option.pairAnalysis){
//            for(int i = 0; i < curr->children.size(); i++){
//                for(int a=0; a < num_of_agents;a++){
//                    if (a!=curr->children[i]->agent_id){
//                        startPairAnalysis(curr->children[i], curr->children[i]->agent_id, a);
//                    }
//                }
//            }
//        }


        curr->clear();
        t1 = std::clock();
        if (open_list.size() == 0) {
            solution_found = false;
            break;
        }
        ICBSNode* open_head = open_list.top();
        assert( open_head->f_val >= min_f_val);
        if ( open_head->f_val > min_f_val)
        {
            min_f_val = open_head->f_val;
            double new_focal_list_threshold = min_f_val * focal_w;
            updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
            focal_list_threshold = new_focal_list_threshold;
            if (screen > 3) {
                cout << "new focal threadhold: " << focal_list_threshold << endl;
            }
        }
        runtime_listoperation += std::clock() - t1;

    }  // end of while loop


    if (focal_list.empty() && solution_cost < 0)
    {
        solution_cost = -2;
        if(!analysisInstance)
            cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
             HL_num_expanded << " ; " << HL_num_generated << " ; " <<
             LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; "
             << RMTime / CLOCKS_PER_SEC << ";" <<
             num_standard << ";" << num_rectangle << ";" <<
             num_corridor2 << ";" << num_corridor4 << ";" << num_target << "," << num_0FlipRectangle << "," <<
             num_1FlipRectangle << "," << num_2FlipRectangle << "," << num_chasingRectangle << "," <<
             less10 << ","<<less100 << ","<<less1000 << ","<<less10000 <<"," << less100000 <<","<<
             larger100000 << "," <<num_pairs <<"," <<num_failed <<
             "|Open|=" << open_list.size() << endl;
        timeout = true;
        solution_found = false;
        if (debug_mode)
            printHLTree();
    }
    return solution_found;

}


template<class Map>
bool MultiMapICBSSearch<Map>::runICBSSearch() 
{
    start = std::clock();
    initializeDummyStart();
    if (option.pairAnalysis){
        for (int i = 0 ; i < num_of_agents; i++){
            for (int j = i +1 ; j<num_of_agents;j++){
                startPairAnalysis(dummy_start, i, j);
            }
        }
    }
    return search();
}

template<class Map>
void MultiMapICBSSearch<Map>::printConstraints(ICBSNode* node,int agent_id,ofstream& out){
    ICBSNode* curr = node;
    bool first = true;
    while (curr != NULL)
    {
        if (curr->agent_id == agent_id)
        {
            for (auto constraint : curr->constraints){
                int x, y, z;
                constraint_type type;
                tie(x, y, z, type) = constraint;
                string string_type;
                if(type == 0)
                    string_type = "L";
                else if(type == 1)
                    string_type = "R";
                else if(type == 2)
                    string_type = "B";
                else if(type == 3)
                    string_type = "V";
                else if(type == 4)
                    string_type = "E";
                if (first)
                    first = false;
                else
                    out<<",";
                out<<"<"<<agent_id <<"," << x << ","<< y << "," << z << "," << string_type  <<">";
            }
        }

        curr = curr->parent;
    }
};


template<class Map>
void MultiMapICBSSearch<Map>::startPairAnalysis(ICBSNode* node,int agent1, int agent2)
{
    ConstraintsHasher c1(agent1,node,0,0);
    ConstraintsHasher c2(agent2,node,0,0);

    if (pairAnalysisTable[c1.a].count(c1) != 0 && pairAnalysisTable[c1.a][c1].count(c2) != 0) {
        repeated_pairs += 1;
        return;
    }

    if (pairAnalysisTable[c1.a].count(c1) == 0){
        pairAnalysisTable[c1.a][c1] = std::unordered_set<ConstraintsHasher>();
    }
    if (pairAnalysisTable[c2.a].count(c2) == 0){
        pairAnalysisTable[c2.a][c2] = std::unordered_set<ConstraintsHasher>();
    }
    pairAnalysisTable[c1.a][c1].insert(c2);
    pairAnalysisTable[c2.a][c2].insert(c1);
    if(screen>=2)
        cout<<"start pair analysis for "<< agent1 << " " <<agent2<<endl;

    bool result = analysisEngine->pairedAnalysis(node,agent1,agent2);
    countNodes(analysisEngine->HL_num_expanded);
    if (!result){
        num_failed+=1;
    }
    if (option.printFailedPair && analysisEngine->HL_num_expanded>=1000) {
        MDD<Map> *a1Mdd = new MDD<Map>();
        MDD<Map> *a2Mdd = new MDD<Map>();
        // vector < list< pair<int, int> > >* cons_vec = collectConstraints(&node, id);
        updateConstraintTable(node, agent1);
        int a1PathLength;
        int a2PathLength=0;
        if(node != dummy_start) {
            for (auto path : node->paths) {
                if (path.first ==agent1){
                    a1PathLength = path.second.size();
                }
                else if(path.first ==agent2){
                    a2PathLength = path.second.size();
                }
            }
        }
        else{
            a1PathLength = paths[agent1]->size();
        }
        if(a2PathLength==0){
            a2PathLength = paths[agent2]->size();
        }

        a1Mdd->buildMDD(constraintTable, a1PathLength + kDelay, *search_engines[agent1]);
        updateConstraintTable(node, agent2);
        a2Mdd->buildMDD(constraintTable,  a2PathLength + kDelay, *search_engines[agent2]);
//            MDD<Map> *a1Mdd = buildMDD(*node, agent1, kDelay);
//            MDD<Map> *a2Mdd = buildMDD(*node, agent2, kDelay);
        if (num_failed!=1)
            analysisOutput<<",";
        analysisOutput <<"{"<<endl;
        analysisOutput <<"\"a1\" :"<< agent1 <<"," << endl;
        analysisOutput <<"\"a1_mdd\" :" <<"["<<endl;
        a1Mdd->printNodes(analysisOutput);
        analysisOutput <<"],"<<endl;
        analysisOutput <<"\"a1_constraints\":"<<"\"";
        printConstraints(node, agent1,analysisOutput);
        analysisOutput <<"\","<<endl;
        analysisOutput << "\"a2\" :"<< agent2 <<"," << endl;
        analysisOutput <<"\"a2_mdd\" :" <<"["<<endl;
        a2Mdd->printNodes(analysisOutput);
        analysisOutput <<"],"<<endl;
        analysisOutput <<"\"a2_constraints\":"<<"\"";
        printConstraints(node, agent2,analysisOutput);
        analysisOutput <<"\","<<endl;
        analysisOutput <<"\"k\":"<<kDelay <<endl;
        analysisOutput <<"}"<<endl;

        delete a1Mdd;
        delete a2Mdd;
    }
    num_pairs+=1;
}

template<class Map>
void MultiMapICBSSearch<Map>::countNodes(int amount)
{
    if(amount < 10)
        less10+=1;
    else if(amount < 100)
        less100+=1;
    else if(amount < 1000)
        less1000+=1;
    else if(amount < 10000)
        less10000+=1;
    else if(amount < 100000)
        less100000+=1;
    else
        larger100000+=1;

}






template<class Map>
MultiMapICBSSearch<Map>::~MultiMapICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++){
	    if(search_engines[i]!=NULL) {
            delete (search_engines[i]);
            search_engines[i]=NULL;
        }
	}

	releaseClosedListNodes();
	if (!mddTable.empty())
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			for (auto mdd : mddTable[i])
			{
			    if (mdd.second != nullptr)
				delete mdd.second;
			}
		}
	}
	if (analysisEngine!=NULL){
	    analysisEngine->search_engines.clear();
	    delete analysisEngine;
	}
}

template<class Map>
MultiMapICBSSearch<Map>::MultiMapICBSSearch(Map* ml, AgentsLoader& al, double f_w, constraint_strategy c, int time_limit, int screen, int kDlay, options options1)
{
	this->option = options1;
	this->focal_w = f_w;
	this->time_limit = time_limit;
	this->screen = screen;
	if (screen >= 2||options1.debug)
		debug_mode = true;


	if (debug_mode)
		cout << "[CBS] Initializing CBS" << endl;
	cons_strategy = c;
	HL_num_expanded = 0;
	HL_num_generated = 0;

	LL_num_expanded = 0;
	LL_num_generated = 0;
	this->num_col = ml->cols;
	this->al = al;
	this->ml = ml;
	num_of_agents = al.num_of_agents;
	map_size = ml->rows*ml->cols;
	solution_found = false;
	solution_cost = -1;
	kDelay = kDlay;
	asymmetry_constraint = options1.asymmetry_constraint;
	ignore_t0 = options1.ignore_t0;
	shortBarrier = options1.shortBarrier;
	search_engines = std::vector<SingleAgentICBS<Map>* >(num_of_agents);
	if (debug_mode)
		cout << "Initializing search engines" << endl;
	for (int i = 0; i < num_of_agents; i++) {
		if (debug_mode)
			cout << "initializing agent "<< i << endl;
		int init_loc = ml->linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml->linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);

        ComputeHeuristic<Map> ch(init_loc, goal_loc, ml, al.headings[i]);
        search_engines[i] = new SingleAgentICBS<Map>(init_loc, goal_loc, ml,i, al.headings[i],kDelay);
        if (debug_mode)
            cout << "initializing heuristics for "<< i << endl;
        ch.getHVals(search_engines[i]->my_heuristic);


	}

	if (debug_mode) {
		cout << "Initializing search engines done" << endl;
	}
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
		//cout << "******************************" << endl;
		//cout << "Agent: " << i << endl;
		if (search_engines[i]->findPath(paths_found_initially[i], focal_w, constraintTable, res_table, dummy_start->makespan + 1, 0) == false && !analysisInstance)
			cout << "NO SOLUTION EXISTS";
		
		paths[i] = &paths_found_initially[i];
		/*if (paths[i]->at(2).conflist == NULL) {
			cout << "x" << endl;

		}
		else {
			cout << paths[i]->at(2).conflist->size() << endl;

		}*/
		res_table->addPath(i, paths[i]);
		dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);

		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;

	}
	delete (res_table);

	//printPaths();

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
	if (debug_mode)
		cout << "Find initial conflict" << endl;
	findConflicts(*dummy_start);
	if (debug_mode)
		cout << "Find initial conflict done" << endl;
	min_f_val = dummy_start->f_val;
	focal_list_threshold = min_f_val * focal_w;
	if (option.print_nodes)
	{
	    cout<<"****************************"<<endl;
	    cout<<"Initialize dummy start node, node #: 1"<<endl;
	    for (int i = 0; i<paths.size();i++){
	        cout<<"Agent id: " << i<<": ";
	        for (auto p : *paths[i]){
	            cout<<"("<<p.location <<")->";
	        }
	        cout<<endl;
	    }

		cout << "Dummy start unclassified conflicts:";
		for (auto &conit : dummy_start->unknownConf) {
			cout << "<" << conit->a1<< "," << conit->a2 << ","
				<< conit->originalConf1 << ","
				<< conit->originalConf2 << ","
				<< conit->t<<","<< conit->type<< ">; ";

		}
		cout << endl;
	}
//     if (cons_strategy == constraint_strategy::CBSH_GR )
 	if (cons_strategy == constraint_strategy::CBSH_GR || cons_strategy == constraint_strategy::CBSH_RM)
		mddTable.resize(num_of_agents);
 	if (option.pairAnalysis){
 	    pairAnalysisTable.resize(num_of_agents);
 	}
	if(debug_mode)
		cout << "Initializing done" << endl;

}


template<class Map>
MDD<Map>* MultiMapICBSSearch<Map>::buildMDD(ICBSNode& node, int id, int k)
{
    clock_t t = std::clock();
    TotalMDD+=1;
    if (k>0){
        TotalKMDD+=1;
    }
    int num_levels = paths[id]->size() + k;
	MDD<Map>* mdd = NULL;
    ConstraintsHasher c(id, &node,num_levels,k);

    if (!mddTable.empty())
	{
		typename std::unordered_map<ConstraintsHasher, MDD<Map>*>::const_iterator got = mddTable[c.a].find(c);
		if (got != mddTable[c.a].end())
		{
			mdd = got->second;
		}
	}
	if (mdd == NULL)
	{
		mdd = new MDD<Map>();
		// vector < list< pair<int, int> > >* cons_vec = collectConstraints(&node, id);
		updateConstraintTable(&node, id);
		mdd->buildMDD(constraintTable, num_levels, *search_engines[id]);
        if (!mddTable.empty())
            mddTable[c.a][c] = mdd;
	}
	else{
	    if(screen>=5)
	        cout<<"Find existing mdd"<<endl;
	    TotalExistMDD+=1;
        if (k>0){
            TotalExistKMDD+=1;
        }
	}
	if (k == 0) {
        for (int i = 0; i < mdd->levels.size(); i++)
            paths[id]->at(i).single = mdd->levels[i].size() == 1;

        if (cons_strategy == constraint_strategy::CBSH_RM || cons_strategy == constraint_strategy::CBSH_GR) {
            for (int i = 0; i < mdd->levels.size(); i++) {
                for (MDDNode *n : mdd->levels[i]) {
                    paths[id]->at(i).locations.push_back(n->locs.front());
                }
            }
        }
    }
    runtime_mdd += (std::clock()-t)/CLOCKS_PER_SEC;
	return mdd;
}

template<class Map>
bool MultiMapICBSSearch<Map>::findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound)
{
	if (debug_mode)
		cout << "Start LL search" << endl;
	// extract all constraints on agent ag
	ICBSNode* curr = node;
	// vector < list< pair<int, int> > >* cons_vec = collectConstraints(curr, ag);
	updateConstraintTable(curr, ag);
	// build reservation table
	size_t max_plan_len = node->makespan + 1;

	ReservationTable* res_table = new ReservationTable(map_size, &paths, ag);  // initialized to false

	// find a path
	vector<PathEntry> newPath;
	//cout << "**************" << endl;
	//cout << "Single agent : " << curr->agent_id << endl;
	bool foundSol = search_engines[ag]->findPath(newPath, focal_w, constraintTable, res_table, max_plan_len, lowerbound, start, time_limit);

	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;
	// delete (cons_vec);

	delete (res_table);


	if (foundSol)
	{
		node->paths.emplace_back(ag, newPath);

		node->g_val = node->g_val - paths[ag]->size() + newPath.size();

		paths[ag] = &node->paths.back().second;

		node->makespan = std::max(node->makespan, newPath.size() - 1);
		if (screen>=2){
		    cout<<"find path"<<endl;
		}

		return true;
	}
	else
	{
        if (screen>=2){
            cout<<"no path"<<endl;
        }

		return false;
	}
}


template<class Map>
void MultiMapICBSSearch<Map>::updateConstraintTable(ICBSNode* curr, int agent_id)
{
	constraintTable.clear();
	constraintTable.goal_location = search_engines[agent_id]->goal_location;
	int constraint_amount = 0;
	int layer = 0;
	while (curr != NULL)
	{
	    layer+=1;
		if (curr->agent_id == agent_id)
		{
			constraintTable.insert(curr->constraints,agent_id,num_col,map_size);
			constraint_amount += curr->constraints.size();
		}
		else if (!curr->constraints.empty())
		{
			int x, y, z;
			constraint_type type;
			tie(x, y, z, type) = curr->constraints.front();
			if (type == constraint_type::LENGTH && x >= 0 && y != agent_id)
			{
				constraintTable.insert(x, z, INT_MAX);
			}
		}
		curr = curr->parent;
	}
	if(screen>=2){
	    cout<<"agent: "<<agent_id<<" constraints amount: "<< constraint_amount<<" layers: "<<layer << endl;
	}
}

template<class Map>
void MultiMapICBSSearch<Map>::classifyConflicts(ICBSNode &parent)
{
	if (parent.conflicts.empty() && parent.unknownConf.empty())
		return; // No conflict
	int previousRetangle[4] = {0,0,0,0};
	// Classify all conflicts in unknownConf
	while (!parent.unknownConf.empty())
	{
		std::shared_ptr<Conflict> con = parent.unknownConf.front();
		int a1 = con->a1, a2 = con->a2;
		int loc1, loc2, timestep,timestep2;
		constraint_type type;
		loc1 = con->originalConf1;
		loc2 = con->originalConf2;
		timestep = con->t;
		timestep2 = timestep + con->k;
		//std::tie(loc1, loc2, timestep, type) = con->constraint1.front();
		parent.unknownConf.pop_front();

		int a1_p = 0, a2_p = 0;
		
		bool cardinal1 = false, cardinal2 = false;
		if (timestep >= paths[a1]->size())
			cardinal1 = true;
		else if (paths[a1]->at(0).singles.empty())
		{
            for (int l: paths[a1]->at(timestep).occupations){
                if (l == loc1 || l ==loc2){
                    break;
                }
                a1_p++;
            }

			MDD<Map>* mdd = buildMDD(parent, a1);
			for (int i = 0; i < mdd->levels.size(); i++) {
                paths[a1]->at(i).singles.resize(kDelay+1);
                for(int j=0;j<kDelay+1;j++){
                    paths[a1]->at(i).singles[j] = mdd->level_locs[i][j].size() == 1;
//                    cout<<mdd->level_locs[i][j].size()<<",";
//                    assert(mdd->level_locs[i][j].size() >=1);

                }
            }
//			cout<<endl;
			if (mddTable.empty())
				delete mdd;

        }
		if (timestep2 >= paths[a2]->size())
			cardinal2 = true;
		else if (paths[a2]->at(0).singles.empty())
		{
            for (int l: paths[a2]->at(timestep).occupations){
                if (l == loc1 || l ==loc2){
                    break;
                }
                a2_p++;
            }
			MDD<Map>* mdd = buildMDD(parent, a2);
            for (int i = 0; i < mdd->levels.size(); i++) {
                paths[a2]->at(i).singles.resize(kDelay+1);
                for(int j=0;j<kDelay+1;j++){
                    paths[a2]->at(i).singles[j] = mdd->level_locs[i][j].size() == 1;
//                    cout<<mdd->level_locs[i][j].size()<<",";
//                    assert(mdd->level_locs[i][j].size() >=1);

                }
            }
//            cout<< endl;
			if (mddTable.empty())
				delete mdd;
		}

		if (type == conflict_type::STANDARD && loc2 >= 0) // Edge conflict
		{
		    assert(a1_p == a2_p && a2_p ==0);
			cardinal1 = paths[a1]->at(timestep).singles[0] && paths[a1]->at(timestep - 1).singles[0];
			cardinal2 = paths[a2]->at(timestep).singles[0] && paths[a2]->at(timestep - 1).singles[0];
		}
		else // vertex conflict or target conflict
		{
			if (!cardinal1)
				cardinal1 = paths[a1]->at(timestep).singles[a1_p];
			if (!cardinal2)
				cardinal2 = paths[a2]->at(timestep).singles[a2_p];

			con->clear();
            int range = min(kDelay - a1_p,kDelay - a2_p);
            con->vertexConflictRange(a1,a2,loc1,timestep,con->k,range);
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
        else if (con->type == conflict_type::STANDARD &&
            paths[con->a1]->size() <= con->t || paths[con->a2]->size() <= con->t) //conflict happens after agent reaches its goal
        {
            parent.conflicts.push_back(con);
            continue;
        }
		else if(con->p == conflict_priority::CARDINAL){ // we don't need corridor reasoning for start conflicts
            parent.cardinal_waiting.push_back(con);
        }
        else{
            parent.non_cardinal_waiting.push_back(con);
        }

		
            
		parent.conflicts.push_back(con);
	}

    if (!(corridor2||rectangleMDD))
        return;
    bool found = false;
    while ( !parent.cardinal_waiting.empty()){
        std::shared_ptr<Conflict> conflict  = parent.cardinal_waiting.front();
        parent.cardinal_waiting.pop_front();
        // Corridor reasoning
        std::shared_ptr<Conflict> corridor;
        if(rectangleMDD ){
            std::shared_ptr<Conflict> rectangle = nullptr;
            bool isRectangle = rectangleReasoning(conflict,parent, rectangle);
            if (isRectangle) {
                parent.conflicts.remove(conflict);
                found = true;
                assert(rectangle != nullptr);
                if( rectangle->p  == conflict_priority::CARDINAL)
                    break;
            }
        }
        if( corridor2 && isCorridorConflict(corridor, conflict, &parent))
        {

            corridor->p = conflict->p;
            parent.conflicts.push_back(corridor);
            parent.conflicts.remove(conflict);

            found = true;
//            break;
        }
    }

    while ( !parent.non_cardinal_waiting.empty() && !found){
        std::shared_ptr<Conflict> conflict  = parent.non_cardinal_waiting.front();
        parent.non_cardinal_waiting.pop_front();
        std::shared_ptr<Conflict> corridor;
        if(rectangleMDD){
            std::shared_ptr<Conflict> rectangle = nullptr;
            bool isRectangle = rectangleReasoning(conflict,parent,rectangle);
            if (isRectangle) {
                assert(rectangle != nullptr);
                parent.conflicts.remove(conflict);
                break;
            }
        }
        if( corridor2 && isCorridorConflict(corridor, conflict, &parent))
        {

            corridor->p = conflict->p;
            parent.conflicts.push_back(corridor);
            parent.conflicts.remove(conflict);
//            break;
        }
    }

	// remove conflicts that cannot be chosen, to save some memory
	removeLowPriorityConflicts(parent.conflicts);
}

template<class Map>
bool MultiMapICBSSearch<Map>::rectangleReasoning(const std::shared_ptr<Conflict>& con,ICBSNode &parent,std::shared_ptr<Conflict>& rectangle){
    std::clock_t RM_Start = std::clock();
    int a1 = con->a1, a2 = con->a2;
    int loc1, loc2, timestep,timestep2;
    constraint_type type;
    loc1 = con->originalConf1;
    loc2 = con->originalConf2;
    timestep = con->t;
    timestep2 = timestep + con->k;


    if (screen >= 4) {
        cout << "new rm rectangle detecting" << endl;
        cout << "timestep: " << timestep << endl;
        cout << "timestep2: " << timestep2 << endl;

    }
    bool not_rectangle = false;
    int action1, action2, action_diff;
    if (timestep != 0 ) {

        action1 = getAction(paths.at(a1)->at(timestep).location, paths.at(a1)->at(timestep - 1).location, num_col);
    }

    if (timestep == 0 || action1 == action::WAIT) {

        action1 = getAction(paths.at(a1)->at(timestep + 1).location, paths.at(a1)->at(timestep).location, num_col);
    }

    if (timestep2 != 0 ) {
        action2 = getAction(paths.at(a2)->at(timestep2).location, paths.at(a2)->at(timestep2 - 1).location, num_col);
    }

    if (timestep2 == 0 || action2 == action::WAIT) {

        action2 = getAction(paths.at(a2)->at(timestep2 + 1).location, paths.at(a2)->at(timestep2).location, num_col);
    }



    if (screen >= 4) {
        cout << "action1: " << action1 << endl;
        cout << "action2: " << action2 << endl;
    }

    action_diff = abs(action1 - action2);


    if ((action1!=action::WAIT && action2!=action::WAIT) &&(action_diff == 1 || action_diff == 3)) {
        int t1_start, t1_end, t2_start, t2_end;
        int t1_waits = 0;
        int t2_waits = 0;
        int s1, g1, s2, g2;

        //find furthest correct direction action along the path.
        pair<int, int> t1s_result = get_st(*paths[a1], timestep, num_col, action1, action2, kDelay, false, false);
        pair<int, int> t1e_result = get_gt(*paths[a1], timestep, num_col, action1, action2, kDelay, false, false);
        pair<int, int> t2s_result = get_st(*paths[a2], timestep2, num_col, action1, action2, kDelay,false, false);
        pair<int, int> t2e_result = get_gt(*paths[a2], timestep2, num_col, action1, action2, kDelay,false, false);
        t1_start = t1s_result.first;
        t1_end = t1e_result.first;
        t2_start = t2s_result.first;
        t2_end = t2e_result.first;

        if(t1_start==-1 || t2_start==-1 ||t1_end==-1||t2_end==-1){
            RMTime += std::clock() - RM_Start;
            RMFailBeforeRec+=1;
            if(screen>=5)
                cout<<"failed to extract rectangle starts or goals"<<endl;
            return false;
        }
        s1 = paths[a1]->at(t1_start).location;
        g1 = paths[a1]->at(t1_end).location;
        s2 = paths[a2]->at(t2_start).location;
        g2 = paths[a2]->at(t2_end).location;


        //calculate areas and so on. Rg and Rs are D and E in the paper.
        int new_type, new_area,a1_Rs_t,a1_Rg_t;
        std::pair<int, int> Rg, Rs;
        Rg = getRg(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(g1 / num_col, g1 % num_col),
                   std::make_pair(g2 / num_col, g2 % num_col));
        Rs = getRs(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(s2 / num_col, s2 % num_col),
                   std::make_pair(g1 / num_col, g1 % num_col));

        a1_Rs_t = getMahattanDistance(s1 / num_col, s1%num_col, Rs.first, Rs.second) + t1_start;
        a1_Rg_t = getMahattanDistance(s1 / num_col, s1%num_col, Rg.first, Rg.second) + t1_start;



        new_area = (abs(Rs.first - Rg.first)+1) * (abs(Rs.second - Rg.second)+1);

        if (s1 == s2 || s1 == g1 || s2 == g2 || new_area <= 2) {
            RMTime += std::clock() - RM_Start;
            RMFailBeforeRec+=1;
            if(screen>=5)
                cout<<"s or g are same or area<=2"<<endl;
            return false;
        }


        RMDetectionCount+=1;

        int rt1, rt2; //root time

        int total_k = -1;

        if(screen >=4){
            cout << "Start finding best k rectangle." << endl;
        }

        //get earliest timestep agent visit s1 s2 (B1 B2 in the paper) from mdd.
        const MDD<Map>* MDD1 = buildMDD(parent,a1,0);
        const MDD<Map>* MDD2 = buildMDD(parent,a2,0);
        t1_start = get_opt_mdd(MDD1->levels,s1,paths[a1]->at(t1_start).heading);
        t2_start = get_opt_mdd(MDD2->levels,s2,paths[a2]->at(t2_start).heading);

        //ideal root time for each agent visit D (in real D might be unavaliable)
        rt1 = t1_start + getMahattanDistance(Rs.first, Rs.second, s1 / num_col, s1%num_col);//TODO: Here can improve.
        rt2 = t2_start + getMahattanDistance(Rs.first, Rs.second, s2 / num_col, s2%num_col);


        int root = std::min(rt1,rt2);
        int diff = std::abs(rt1-rt2);
        int a1kMax = kDelay;
        int a2kMax = kDelay;
        new_type = -1;

        //try all combination of possible k to check does any barriers build by the combination satisfy condition 1 in the paper.
        for (int a1k=0; a1k<=a1kMax; a1k++){
            for (int a2k=0; a2k<=a2kMax; a2k++){
                int real_a1k = rt1==root ? a1k : a1k - diff > 0? a1k - diff:0;
                int real_a2k = rt2==root ? a2k : a2k - diff > 0? a2k - diff:0;
                const MDD<Map>* a1MDD = buildMDD(parent,a1,real_a1k);
                const MDD<Map>* a2MDD = buildMDD(parent,a2,real_a2k);
                if(screen >=4){
                    cout << "root: " << root << endl;
                    cout << "diff: " << diff << endl;

                    cout << "a1k: " << a1k << endl;
                    cout << "a2k: " << a2k << endl;
                    cout << "real_a1k: " << real_a1k << endl;
                    cout << "real_a2k: " << real_a2k << endl;
                    cout << "a1mdd levels: "<< a1MDD->levels.size()<<endl;
                    cout << "a2mdd levels: "<< a2MDD->levels.size()<<endl;
                }
                std::shared_ptr<Conflict> temp = std::shared_ptr<Conflict>(
                        new Conflict(loc1, con->k, timestep));

                //build barriers based on k
                int result = temp->generalKRectangleConflict(a1, a2, Rs, Rg,
                                                             make_pair(s1 / num_col, s1 % num_col),
                                                             make_pair(s2 / num_col, s2 % num_col),
                                                             root, root, paths, t1_start, t2_start,
                                                             make_pair(g1 / num_col, g1 % num_col),
                                                             make_pair(g2 / num_col, g2 % num_col),
                                                             num_col, a1k, a2k,real_a2k/2, real_a1k/2,
                                                             &(a1MDD->levels), &(a2MDD->levels));
                if(screen >=4){
                    cout << "result: " << result << endl;
                }

                //result == 1 or 2 means the barriers are shifted over the any s1/s2/g1/g2 (B1/B2/A1/A2 in the paper)
                if (result == 1){
                    a1kMax = a1k;
                    break;
                }
                else if(result ==2){
                    a2kMax = a2k;
                    continue;
                }

                //check is barriers empty, the barriers follows jiaoyang's rm work that only constraint locations exist in k-mdd
                //The multiConstraint1 contains both entry and exit barrier.
                bool empty_constraint = false;
                for (auto constraint : temp->multiConstraint1){
                    if (constraint.size()==0){
                        if(screen >=4){
                            cout << "constraint for a1 is empty" << endl;
                        }
                        empty_constraint = true;
                    }
                }

                for (auto constraint : temp->multiConstraint2){
                    if (constraint.size()==0){
                        if(screen >=4){
                            cout << "constraint for a2 is empty" << endl;
                        }
                        empty_constraint = true;
                    }
                }
                if(empty_constraint){
                    continue;
                }

                assert(temp->multiConstraint1.size() == 2 && "a1 no two constraints");
                assert(temp->multiConstraint2.size() == 2 && "a2 no two constraints");

                //check does entry and exit satisfy condition 1 in the paper, also classify the rectangle.
                int a1_type = classifyBarrierAndRectangle(a1MDD->levels,
                                                          temp->multiConstraint1.back(),
                                                          temp->multiConstraint1.front(),
                                                          num_col, map_size);
                int a2_type = classifyBarrierAndRectangle(a2MDD->levels,
                                                          temp->multiConstraint2.back(),
                                                          temp->multiConstraint2.front(),
                                                          num_col, map_size);
                //-1 means does not satisfy condition 1.
                if(a1_type == -1 || a2_type == -1){
                    if(screen >=4){
                        cout << "a1_type: " << a1_type << endl;
                        cout << "a2_type: " << a2_type << endl;
                        cout << "Not satisfy condition1 and condition2" << endl;
                    }
                    if (a1_type == -1){
                        a1kMax = a1k;
                        break;
                    }
                    if (a2_type == -1) {
                        a2kMax = a2k;
                        continue;
                    }
                }


                int temp_type = a1_type + a2_type;
                int temp_total_k = a1k+a2k;

                if(screen >= 4){
                    cout << "temp type: "<<temp_type<<" temp total k: "<< temp_total_k << endl;
                }

                if (temp_total_k > total_k || (temp_total_k == total_k && temp_type > new_type)){
                    rectangle = temp;
                    new_type = temp_type;
                    total_k = temp_total_k;

                    if(screen >=4){
                        cout << "Select this one" << endl;
                        cout<<"rectangle: "<< *rectangle << endl;
                    }

                }
                else{
                    if(screen >=4){
                        cout << "Abandon this one"<<endl;
                    }
                }

            }


        }


        if (screen >= 5) {
            cout << "initial_Rs: " << Rs.first << " " << Rs.second << endl;
            cout << "initial_Rg: " << Rg.first << " " << Rg.second << endl;
            cout << "s1: " << s1 / num_col << " " << s1 % num_col << endl;
            cout << "g1: " << g1 / num_col << " " << g1 % num_col << endl;
            cout << "s1_t: " << t1_start << endl;
            cout << "rt1: " << rt1 << endl;
            cout <<"a1 waits"<<t1_waits<<endl;


            cout << "s2: " << s2 / num_col << " " << s2 % num_col << endl;
            cout << "g2: " << g2 / num_col << " " << g2 % num_col << endl;
            cout << "s2_t: " << t2_start << endl;
            cout << "rt2: " << rt2 << endl;
            cout <<"a2 waits"<<t2_waits<<endl;


            cout << "Rs: " << Rs.first << " " << Rs.second << endl;
            cout << "Rg: " << Rg.first << " " << Rg.second << endl;

            cout << "area:" << new_area << endl;
        }

        if (rectangle == nullptr){
            not_rectangle = true;
            if (screen >= 4) {
                cout <<"rectangle == nullptr"<<endl;
            }
            RMTime += std::clock() - RM_Start;
            return false;
        }
        else{
            //remove entry barriers
            rectangle->multiConstraint1.pop_back();
            rectangle->multiConstraint2.pop_back();
        }

        //check does final constraint block the current path
        bool isBlocked = true;
        if (screen >= 4) {
            cout <<"Check constraint and current path"<<endl;
            cout << "Constraint 1 size: " << rectangle->multiConstraint1.size()<<endl;
            cout << "Constraint 2 size: " << rectangle->multiConstraint2.size()<<endl;
            cout << "a1: " <<endl;
            for (auto i : rectangle->multiConstraint1.front()){
                cout << "<(" << std::get<0>(i)/num_col <<","<<std::get<0>(i)%num_col << ")," << std::get<1>(i) << "," <<
                     std::get<2>(i) << "," << std::get<3>(i) << ">";
            }
            cout <<endl;
            cout << "a2: " <<endl;
            for (auto i : rectangle->multiConstraint2.front()){
                cout << "<(" << std::get<0>(i)/num_col <<","<<std::get<0>(i)%num_col << ")," << std::get<1>(i) << "," <<
                     std::get<2>(i) << "," << std::get<3>(i) << ">";
            }
            cout<<endl;
        }
        for (auto constraint : rectangle->multiConstraint1) {
            isBlocked = isBlocked && blocked(*paths[rectangle->a1], constraint);
        }
        for (auto constraint : rectangle->multiConstraint2) {
            isBlocked = isBlocked && blocked(*paths[rectangle->a2], constraint);
        }


        if (!not_rectangle && isBlocked) {

            if (new_type == 2)
                rectangle->p = conflict_priority::CARDINAL;
            else if (new_type == 1)
                rectangle->p = conflict_priority::SEMI;
            else
                rectangle->p = conflict_priority::NON;

            parent.conflicts.push_back(rectangle);
            if (screen >= 4){
                cout << "add " << *rectangle << endl;
                cout << "conflicts amount " << parent.conflicts.size()<<endl;
            }
            RMSuccessCount+=1;
            RMTime += std::clock() - RM_Start;
            return true;
        }
        else {
            if (screen >= 4) {
                cout<<"rectangle: "<< *rectangle << endl;
                cout << "not rectangle" << endl;
                if (!isBlocked)
                    cout << "not blocked" << endl;

            }
            RMTime += std::clock() - RM_Start;
            return false;
        }
    }
    return false;

}
        




template class MultiMapICBSSearch<MapLoader>;
template class MultiMapICBSSearch<FlatlandLoader>;

