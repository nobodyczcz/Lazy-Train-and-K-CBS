#include "MDD.h"
#include "flat_map_loader.h"

#include <iostream>

template<class Map>
bool MDD<Map>::buildMDD( ConstraintTable& constraints, int numOfLevels, SingleAgentICBS<Map> & solver, bool train)
{
	MDDNode* root = new MDDNode(std::list<int>(), nullptr,train); // Root
	root->locs.resize(solver.kRobust+1, solver.start_location);
	root->heading = solver.start_heading;
	root->row = solver.start_location / solver.num_col;
	root->col = solver.start_location % solver.num_col;
	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	//cout << "start: " << solver.start_heading << endl;
	//cout << "goal: " << solver.goal_location << endl;
	bool done = false;
//	cout<<"start"<<endl;
    int max_level = 0;
	while (!open.empty())
	{
		MDDNode* node = open.front();
		open.pop();
		if (node->level > max_level){
//		    cout<<max_level<<endl;
		    max_level = node->level;
		}
		// Here we suppose all edge cost equals 1
		if (node->level == numOfLevels - 1)
		{
			levels[numOfLevels - 1].push_back(node);
			if(!open.empty())
			{
				while (!open.empty())
				{
					MDDNode* node = open.front();
					open.pop();
					cout << "loc: " << node->locs.front()<<","<< node->locs.back()<<","<<node->locs.size()<<" level: "<< node->level << " heading: " << node->heading<<" h "<< solver.my_heuristic[node->locs.front()].heading[node->heading] <<" "<< solver.my_heuristic[node->locs.front()].heading.count(node->heading)<< endl;

				}
				
				std::cerr << "Failed to build MDD!" << std::endl;
				assert(false);
				exit(1);
			}
			done = true;
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
//		double heuristicBound = numOfLevels - node->level - 2+ 0.001;

//        cout << " Expand node: " << node->locs.front()<<","<< node->locs.back()<<","<<node->locs.size() << " heading: " << node->heading<<" h "<< solver.my_heuristic[node->locs.front()].heading[node->heading]<<", level: "<<node->level << endl;


        vector<pair<int, int>> transitions = solver.ml->get_transitions(node->locs.front(),node->heading,false);
		for (const pair<int, int> move : transitions)
		{
			int new_heading;
			if (node->heading == -1) //heading == -1 means no heading info
				new_heading = -1;
			else
				if (move.second == 4) //move == 4 means wait
					new_heading = node->heading;
				else
					new_heading = move.second;
			int newLoc = move.first;

			if (node->locs.front() == solver.goal_location && newLoc != solver.goal_location){
			    continue;
			}

			std::list<int> new_locs;
            bool no_self_collision = true;
			if(node->locs.front() == solver.goal_location && newLoc == solver.goal_location){
			    new_locs = node->locs;
			    new_locs.pop_back();
			}
			else{
                no_self_collision = getOccupations(new_locs,newLoc,node,solver.kRobust);

			}

			if ((train && !no_self_collision) || new_locs.empty())
			    continue;
            //check does head have edge constraint or body have vertex constraint.
            bool constrained = false;
            if (constraints.is_constrained(node->locs.front() * solver.map_size + newLoc, node->level + 1))
                constrained = true;


            for(auto loc:new_locs){
                if (constraints.is_constrained(loc, node->level + 1, loc != new_locs.front()) )
                    constrained = true;
                if(!train)
                    break;
            }
            double heuristicBound;
            if (train)
                heuristicBound =  double(numOfLevels)-double(new_locs.size()) - double(node->level) - 1.0+ 0.001;
            else
                heuristicBound = numOfLevels - node->level - 2+ 0.001;


//            cout << "newLoc " << newLoc << " heading " << new_heading<<" h "<< solver.my_heuristic[newLoc].heading[new_heading] <<", hb"<<heuristicBound<<", c "<<constrained <<endl;

			if (solver.my_heuristic[newLoc].heading.count(new_heading) && solver.my_heuristic[newLoc].heading[new_heading] < heuristicBound &&
				!constrained) // valid move
			{
				std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if (equal_occupation((*child)->locs, new_locs) || (!train && (*child)->locs.front()==new_locs.front()))  // If the child node exists
					{
						if ((*child)->heading == -1) { //if no heading info
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->locs.front() == solver.goal_location) { //if goal location ignore heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->heading == new_heading) {//child heading equal to node heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}

						
					}
				if (!find) // Else generate a new mdd node
				{
					MDDNode* childNode = new MDDNode(new_locs, node,train);
					childNode->parent = node;
					childNode->heading = new_heading;
					childNode->row = newLoc / solver.num_col;
					childNode->col = newLoc % solver.num_col;

					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	if(!train)
	    assert(done);
	// Backward
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
		{
			for (std::list<MDDNode*>::iterator parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
			{
				if ((*parent)->children.empty()) // a new node
				{
					levels[t - 1].push_back(*parent);
				}
				(*parent)->children.push_back(*it); // add forward edge	
			}
		}
	}
    if(!train)
        assert(!levels[0].empty());


    // Delete useless nodes (nodes who don't have any children)
	for (std::list<MDDNode*>::iterator it = closed.begin(); it != closed.end(); ++it)
		if ((*it)->children.empty() && (*it)->level < numOfLevels - 1){
//            auto node = std::find(levels[(*it)->level].begin(),levels[(*it)->level].end(),*it);
//            if(node != levels[(*it)->level].end()){
//                levels[(*it)->level].erase(node);
//            }
			delete (*it);
		}

	//in train pathfinding, agent may reach same location with di

	if(done) {
        level_locs.resize(levels.size());
        for (int t = numOfLevels - 1; t >= 0; t--) {
            level_locs[t].resize(solver.kRobust + 1);
            assert(!levels[t].empty());
            for (std::list<MDDNode *>::iterator it = levels[t].begin(); it != levels[t].end(); ++it) {
                int position = 0;
                for (auto loc_it = (*it)->locs.begin(); loc_it != (*it)->locs.end(); loc_it++, position++) {
                    if (!level_locs[t][position].count(*loc_it))
                        level_locs[t][position].insert(*loc_it);
                }

            }
        }
    }

	return done;
}

template<class Map>
bool MDD<Map>::getOccupations(list<int>& next_locs, int next_id, MDDNode* curr, int k){
    next_locs.push_back(next_id);
    auto parent = curr;
    int pre_loc = next_id;
    bool conf_free = true;
    while( next_locs.size() <= k){
        if (parent == nullptr){
            next_locs.push_back(next_locs.back());
        }
        else {
            if (pre_loc != parent->locs.front()) {
                next_locs.push_back(parent->locs.front());
                pre_loc = parent->locs.front();
                if (next_locs.front() == next_locs.back()) {
                    conf_free = false;
                }
            }
            parent = parent->parent;
        }
    }
//    for (auto i : next_locs){
//        cout<<i<<",";
//    }
//    cout<<endl;
    return conf_free;
}


////build mdd with specific start heading
//template<class Map>
//bool MDD<Map>::buildMDD( ConstraintTable& constraints,
//	int numOfLevels, SingleAgentICBS<Map> & solver,int start,int start_time, int start_heading)
//{
//	MDDNode* root = new MDDNode(start, NULL); // Root
//	root->heading = start_heading;
//	root->row = solver.start_location / solver.num_col;
//	root->col = solver.start_location % solver.num_col;
//	std::queue<MDDNode*> open;
//	std::list<MDDNode*> closed;
//	open.push(root);
//	closed.push_back(root);
//	levels.resize(numOfLevels);
//	//cout << "start: " << solver.start_heading << endl;
//	//cout << "goal: " << solver.goal_location << endl;
//
//	while (!open.empty())
//	{
//		MDDNode* node = open.front();
//		open.pop();
//		// Here we suppose all edge cost equals 1
//		if (node->level == numOfLevels - 1)
//		{
//			levels[numOfLevels - 1].push_back(node);
//			if(!open.empty())
//			{
//				//while (!open.empty())
//				//{
//				//	MDDNode* node = open.front();
//				//	open.pop();
//				//	cout << "loc: " << node->location << " heading: " << node->heading<<" h "<< solver.my_heuristic[node->location].heading[node->heading] <<" "<< solver.my_heuristic[node->location].heading.count(node->heading)<< endl;
//				//
//				//}
//
//				std::cerr << "Failed to build MDD!" << std::endl;
//				exit(1);
//			}
//			break;
//		}
//		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
//		double heuristicBound = numOfLevels - node->level - 2+ 0.001;
//
//		vector<pair<int, int>> transitions = solver.ml->get_transitions(node->location,node->heading,false);
//		//cout << "current " << node->location << " heading " << node->heading << endl;
//		for (const pair<int, int> move : transitions)
//		{
//			int new_heading;
//			if (node->heading == -1) //heading == -1 means no heading info
//				new_heading = -1;
//			else
//				if (move.second == 4) //move == 4 means wait
//					new_heading = node->heading;
//				else
//					new_heading = move.second;
//			int newLoc = move.first;
//			//cout << "newLoc " << newLoc << " heading " << new_heading<<" h "<< solver.my_heuristic[newLoc].heading[new_heading] << endl;
//
//			if (solver.my_heuristic[newLoc].heading.count(new_heading) && solver.my_heuristic[newLoc].heading[new_heading] < heuristicBound &&
//				!constraints.is_constrained(newLoc, start_time+node->level + 1) &&
//				!constraints.is_constrained(node->location * solver.map_size + newLoc, start_time+node->level + 1)) // valid move
//			{
//				std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
//				bool find = false;
//				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
//					if ((*child)->location == newLoc)  // If the child node exists
//					{
//						if ((*child)->heading == -1) { //if no heading info
//							(*child)->parents.push_back(node); // then add corresponding parent link and child link
//							find = true;
//							break;
//						}
//						else if ((*child)->location == solver.goal_location) { //if goal location ignore heading
//							(*child)->parents.push_back(node); // then add corresponding parent link and child link
//							find = true;
//							break;
//						}
//						else if ((*child)->heading == new_heading) {//child heading equal to node heading
//							(*child)->parents.push_back(node); // then add corresponding parent link and child link
//							find = true;
//							break;
//						}
//
//
//					}
//				if (!find) // Else generate a new mdd node
//				{
//					MDDNode* childNode = new MDDNode(newLoc, node);
//					childNode->heading = new_heading;
//					childNode->row = newLoc / solver.num_col;
//					childNode->col = newLoc % solver.num_col;
//					open.push(childNode);
//					closed.push_back(childNode);
//				}
//			}
//		}
//	}
//	// Backward
//	for (int t = numOfLevels - 1; t > 0; t--)
//	{
//		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
//		{
//			for (std::list<MDDNode*>::iterator parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
//			{
//				if ((*parent)->children.empty()) // a new node
//				{
//					levels[t - 1].push_back(*parent);
//				}
//				(*parent)->children.push_back(*it); // add forward edge
//			}
//		}
//	}
//
//	// Delete useless nodes (nodes who don't have any children)
//	for (std::list<MDDNode*>::iterator it = closed.begin(); it != closed.end(); ++it)
//		if ((*it)->children.empty() && (*it)->level < numOfLevels - 1)
//			delete (*it);
//	return true;
//}
//
//
//// build mdd for reaching specific goal
//template<class Map>
//bool MDD<Map>::buildMDD(ConstraintTable& constraints,
//	int numOfLevels, SingleAgentICBS<Map> & solver, int start, int start_time, int goal, int start_heading)
//{
//	MDDNode* root = new MDDNode(start, NULL); // Root
//	root->heading = start_heading;
//	root->row = solver.start_location / solver.num_col;
//	root->col = solver.start_location % solver.num_col;
//	std::queue<MDDNode*> open;
//	std::list<MDDNode*> closed;
//	open.push(root);
//	closed.push_back(root);
//	levels.resize(numOfLevels);
//	//cout << "start: "<< start<<" heading: " << solver.start_heading << endl;
//	//cout << "goal: " << goal << endl;
//	//cout << "numOfLevels: " << numOfLevels << endl;
//
//	while (!open.empty())
//	{
//		MDDNode* node = open.front();
//		open.pop();
//		// Here we suppose all edge cost equals 1
//		if (node->level == numOfLevels - 1)
//		{
//			levels[numOfLevels - 1].push_back(node);
//			if (!open.empty())
//			{
//				while (!open.empty())
//				{
//					MDDNode* node = open.front();
//					open.pop();
//					//cout << "loc: " << node->location <<" goal: "<< goal << " heading: " << node->heading<<" level: "<<node->level<<" h "<< getMahattanDistance(node->location, goal, solver.num_col) <<" "<< solver.my_heuristic[node->location].heading.count(node->heading)<< endl;
//				}
//
//				std::cerr << "Failed to build MDD!" << std::endl;
//				exit(1);
//			}
//			break;
//		}
//		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
//		double heuristicBound = numOfLevels - node->level - 2 + 0.001;
//
//		vector<pair<int, int>> transitions = solver.ml->get_transitions(node->location, node->heading, false);
//		//cout << "current " << node->location << " heading " << node->heading << endl;
//		for (const pair<int, int> move : transitions)
//		{
//			int new_heading;
//			if (node->heading == -1) //heading == -1 means no heading info
//				new_heading = -1;
//			else
//				if (move.second == 4) //move == 4 means wait
//					new_heading = node->heading;
//				else
//					new_heading = move.second;
//			int newLoc = move.first;
//			int next_h_val = getMahattanDistance(newLoc, goal, solver.num_col);
//			if (next_h_val < heuristicBound &&
//				!constraints.is_constrained(newLoc, start_time+node->level + 1) &&
//				!constraints.is_constrained(node->location * solver.map_size + newLoc, start_time + node->level + 1)) // valid move
//			{
//				std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
//				bool find = false;
//				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
//					if ((*child)->location == newLoc)  // If the child node exists
//					{
//						if ((*child)->heading == -1) { //if no heading info
//							(*child)->parents.push_back(node); // then add corresponding parent link and child link
//							find = true;
//							break;
//						}
//						else if ((*child)->location == goal) { //if goal location ignore heading
//							(*child)->parents.push_back(node); // then add corresponding parent link and child link
//							find = true;
//							break;
//						}
//						else if ((*child)->heading == new_heading) {//child heading equal to node heading
//							(*child)->parents.push_back(node); // then add corresponding parent link and child link
//							find = true;
//							break;
//						}
//
//
//					}
//				if (!find) // Else generate a new mdd node
//				{
//					MDDNode* childNode = new MDDNode(newLoc, node);
//					childNode->heading = new_heading;
//					childNode->row = newLoc / solver.num_col;
//					childNode->col = newLoc % solver.num_col;
//					open.push(childNode);
//					closed.push_back(childNode);
//				}
//			}
//		}
//	}
//	// Backward
//	for (int t = numOfLevels - 1; t > 0; t--)
//	{
//		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
//		{
//			for (std::list<MDDNode*>::iterator parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
//			{
//				if ((*parent)->children.empty()) // a new node
//				{
//					levels[t - 1].push_back(*parent);
//				}
//				(*parent)->children.push_back(*it); // add forward edge
//			}
//		}
//	}
//
//	// Delete useless nodes (nodes who don't have any children)
//	for (std::list<MDDNode*>::iterator it = closed.begin(); it != closed.end(); ++it)
//		if ((*it)->children.empty() && (*it)->level < numOfLevels - 1) {
//		    auto node = std::find(levels[(*it)->level].begin(),levels[(*it)->level].end(),*it);
//		    if(node != levels[(*it)->level].end()){
//		        levels[(*it)->level].erase(node);
//		    }
//            delete (*it);
//
//
//        }
//	return true;
//}

template<class Map>
void MDD<Map>::deleteNode(MDDNode* node)
{
	levels[node->level].remove(node);
	for (std::list<MDDNode*>::iterator child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if((*child)->parents.empty())
			deleteNode(*child);
	}
	for (std::list<MDDNode*>::iterator parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent);
	}
}

template<class Map>
void MDD<Map>::clear()
{
	if(levels.empty())
		return;
	for (int i = 0; i < levels.size(); i++)
	{

		for (std::list<MDDNode*>::iterator it = levels[i].begin(); it != levels[i].end(); ++it)
        {

                if (*it != nullptr) {
                    delete (*it);
                }

        }
	}
}

template<class Map>
MDDNode* MDD<Map>::find(list<int> locs, int level)
{
	if(level < levels.size())
		for (std::list<MDDNode*>::iterator it = levels[level].begin(); it != levels[level].end(); ++it)
			if( equal_occupation((*it)->locs, locs))
				return (*it);
	return NULL;
}

template<class Map>
MDD<Map>::MDD(MDD & cpy) // deep copy
{
	levels.resize(cpy.levels.size());
	MDDNode* root = new MDDNode(cpy.levels[0].front()->locs, NULL,cpy.levels[0].front()->train);
	levels[0].push_back(root);
	for(int t = 0; t < levels.size() - 1; t++)
	{
		for (std::list<MDDNode*>::iterator node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->locs, (*node)->level);
			for (std::list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = find((*cpyChild)->locs, (*cpyChild)->level);
				if (child == NULL)
				{
					child = new MDDNode((*cpyChild)->locs, (*node),(*cpyChild)->train);
					levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}
		
	}
}

template<class Map>
MDD<Map>::~MDD()
{
	clear();
}

template class MDD<MapLoader>;
template class MDD<FlatlandLoader>;


