#include "ReservationTable.h"
#include <iostream>

ReservationTable::ReservationTable(int mapSize,AgentsLoader* agentsLoader) {
	res_table = map_table(mapSize);
	this->agentsLoader = agentsLoader;
}

ReservationTable::ReservationTable(int mapSize, vector<vector<PathEntry>*>* paths,AgentsLoader* agentsLoader, int exclude) {

	res_table = map_table(mapSize);
	this->agentsLoader = agentsLoader;

	addPaths(paths,exclude);
}

void ReservationTable::addPath(int agent_id, std::vector<PathEntry>* path) {
	if (path == NULL)
		return;
	AgentStep* preStep = NULL;
	for (int t = 0; t < path->size(); t++) {
	    bool head = true; //first location is head.
	    for (int loc : path->at(t).occupations) {

            if (!res_table.count(loc)) {
                res_table[loc] = timeline();
            }
            if (!res_table[loc].count(t)) {
                res_table[loc][t] = agentList();
            }


            res_table[loc][t][agent_id] = AgentStep(agent_id, loc, t,head);

            if(head) {
                if (preStep != NULL) {
                    preStep->nextStep = &(res_table[loc][t][agent_id]);
                }
                res_table[loc][t][agent_id].preStep = preStep;
                preStep = &(res_table[loc][t][agent_id]);
            }
            if (t == path->size() - 1) {
                if(head){
                    goalTable[loc][agent_id] = t;
                }
                else{
                    goalTable[loc][-agent_id -1] = t; //Non-head occupation
                }
            }

            head=false;// except firt occupication all false (not head).
        }
	}
}

void ReservationTable::addPaths(vector<vector<PathEntry>*>* paths,int exclude) {
	for (int agent = 0; agent < paths->size(); agent++) {
		if (agent == exclude || (*paths)[agent]==NULL)
			continue;
		addPath(agent, (*paths)[agent]);
	}
}

void ReservationTable::deletePath(int agent_id, std::vector<PathEntry>* path) {
	for (int t = 0; t < path->size(); t++) {
        for (int loc : path->at(t).occupations) {

            if (res_table.count(loc)) {
                if (res_table[loc].count(t)) {
                    res_table[loc][t].erase(agent_id);
                }
            }
            if (t == path->size() - 1) {
                if (goalTable[loc].count(agent_id)) {
                    goalTable[loc].erase(agent_id);
                }
            }
        }
	}
}

std::list<Conflict> ReservationTable::findConflict(int agent, int currLoc, list<int> next_locs, int currT, bool parking) {
    std::list<Conflict> confs;
	int nextT = currT + 1;
	int max_k = this->agentsLoader->max_k;
	int k_1 = this->agentsLoader->k[agent];
	int k_2;
	int a_2;
	//cout << "currloc " << currLoc << " nextloc " << nextLoc << endl;
    if (res_table.count(next_locs.front())) {
        //detect vertex conflict and k delay vertex conflict
        for (int k = -max_k; k <= k_1; k++) {

            int t = nextT + k;
            if (t < 0)
                continue;

            if (res_table[next_locs.front()].count(t)) {
                agentList::iterator it;
                for (it = res_table[next_locs.front()][t].begin(); it != res_table[next_locs.front()][t].end(); ++it) {
                    if (!it->second.head)
                        continue;
                    a_2 = it->second.agent_id;
                    k_2 = this->agentsLoader->k[a_2];


                    if (t >= nextT) {
                        confs.push_back(Conflict(agent,  a_2,next_locs.front(), -1, nextT, k, false));
                    }
                    else if(-k_2 <= k){
                        confs.push_back(Conflict( a_2,agent,next_locs.front(), -1, t , -k, false));
                    }


                }
            }
        }
        if (k_1 == 0){
            int nextLoc = next_locs.front();
            if (res_table.count(nextLoc) && res_table[nextLoc].count(currT)) {
                agentList::iterator it;
                for (it = res_table[nextLoc][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {
                    a_2 = it->second.agent_id;
                    k_2 = this->agentsLoader->k[a_2];
                    if (k_2 == 0 && it->second.nextStep != NULL && it->second.nextStep->loc == currLoc) {
                        confs.push_back(Conflict(agent, a_2, currLoc, nextLoc, nextT, 0,false));
                    }


                }
            }
        }

        if (goalTable.count(next_locs.front())) {
            goalAgentList::iterator it;
            for (it = goalTable[next_locs.front()].begin(); it != goalTable[next_locs.front()].end(); ++it) {

                if (nextT > it->second) {
                    if (it->first >=0){
                        confs.push_back(Conflict(it->first, agent, next_locs.front(), -1, nextT, 0,false));
                    }

                }
            }
        }
        if(parking){
            for (auto& occupation : res_table[next_locs.front()]){
                if (occupation.first > nextT){
                    for (auto& it: occupation.second){
                        if(it.second.head){
                            confs.push_back(Conflict( agent, it.first , next_locs.front(), -1, occupation.first, 0,false));
                        }

                    }
                }
            }
        }
    }

	int train_cell_number = 0;
    int head = next_locs.front();
	for(int nextLoc : next_locs) {
        if (res_table.count(nextLoc)) {
            //detect vertex conflict and k delay vertex conflict

            if (res_table[nextLoc].count(nextT)) {
                agentList::iterator it;
                for (it = res_table[nextLoc][nextT].begin(); it != res_table[nextLoc][nextT].end(); ++it) {
                    if (nextLoc==head || it->second.head)
                        confs.push_back(Conflict(it->second.agent_id, agent, nextLoc,
                                        -1, nextT, 0,true));

                }
            }

            if(parking && nextLoc!=head){//if parking check any head run into parking body
                for (auto& occupation : res_table[nextLoc]){
                    if (occupation.first > nextT){
                        for (auto& it: occupation.second){
                            if (it.second.head)
                                confs.push_back(Conflict( agent, it.first , nextLoc, -1, occupation.first, 0,true));

                        }
                    }
                }
            }
        }

        if (nextLoc == head && goalTable.count(nextLoc)) {// only cares current head run into other's parking head/body
            goalAgentList::iterator it;
            for (it = goalTable[nextLoc].begin(); it != goalTable[nextLoc].end(); ++it) {
                if (nextT > it->second) {
                    if (it->first >=0){
                        continue;
//                        confs.push_back(Conflict(it->first, agent, nextLoc, -1, nextT, 0,true));

                    }
                    else {// body parking
                        confs.push_back(Conflict(-(it->first+1), agent, nextLoc, -1, nextT, 0,true));

                    }
                }
            }
        }



        train_cell_number ++;
    }
			
			



    //detect edge conflict
//    if (kDelay == 0) {
//        int nextLoc = next_locs.front();
//        if (res_table.count(nextLoc) && res_table[nextLoc].count(currT)) {
//            agentList::iterator it;
//            for (it = res_table[nextLoc][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {
//
//                if (it->second.nextStep != NULL && it->second.nextStep->loc == currLoc) {
//                    confs->push_back(std::shared_ptr<tuple<int, int, int, int, int, int,bool>>(
//                            new tuple<int, int, int, int, int, int,bool>(
//                                    agent, it->second.agent_id, currLoc, nextLoc, nextT, 0,false)));
//                }
//
//
//            }
//        }
//    }

	return confs;
}

int ReservationTable::countConflict(int agent, int currLoc, list<int> next_locs, int currT) {
    int nextT = currT + 1;
    if(next_locs.front() == -1)
        return 0;
    int count = 0;

    for (int nextLoc : next_locs) {
        if (res_table.count(nextLoc)) {
            //detect vertex conflict and k delay vertex conflict
            if (res_table[nextLoc].count(nextT)) {
                count++;
            }
        }

        if (goalTable.count(nextLoc)) {
            goalAgentList::iterator it;
            for (it = goalTable[nextLoc].begin(); it != goalTable[nextLoc].end(); ++it) {

                if (nextT > it->second) {
                    count++;
                }
            }
        }
    }

    int nextLoc = next_locs.front();

    if (res_table.count(nextLoc) && res_table[next_locs.front()].count(currT)) {
        agentList::iterator it;
        for (it = res_table[next_locs.front()][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {

            if (it->second.nextStep != NULL && it->second.nextStep->loc == currLoc) {
                count++;
            }

        }
    }




    return count;
}





