#include "ReservationTable.h"
#include <iostream>

ReservationTable::ReservationTable(int mapSize) {
	res_table = map_table(mapSize);
}

ReservationTable::ReservationTable(int mapSize, vector<vector<PathEntry>*>* paths, int exclude) {

	res_table = map_table(mapSize);

	addPaths(paths,exclude);
}

void ReservationTable::addPath(int agent_id, std::vector<PathEntry>* path) {
	if (path == NULL)
		return;
	AgentStep* preStep = NULL;
	for (int t = 0; t < path->size(); t++) {
	    for (int loc : path->at(t).occupations) {
            if (!res_table.count(loc)) {
                res_table[loc] = timeline();
            }
            if (!res_table[loc].count(t)) {
                res_table[loc][t] = agentList();
            }


            res_table[loc][t][agent_id] = AgentStep(agent_id, loc, t);
            if (preStep != NULL) {
                preStep->nextStep = &(res_table[loc][t][agent_id]);
            }
            res_table[loc][t][agent_id].preStep = preStep;
            preStep = &(res_table[loc][t][agent_id]);

            if (t == path->size() - 1) {
                goalTable[loc][agent_id] = t;
            }
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

OldConfList* ReservationTable::findConflict(int agent, int currLoc, list<int> next_locs, int currT,int kDelay, bool ignore_goal_table) {
	OldConfList* confs =  new OldConfList;
	int nextT = currT + 1;
	//cout << "currloc " << currLoc << " nextloc " << nextLoc << endl;
	int train_cell_number = 0;
	for(int nextLoc : next_locs) {
        if (res_table.count(nextLoc)) {
            //detect vertex conflict and k delay vertex conflict



            if (res_table[nextLoc].count(nextT)) {
                agentList::iterator it;
                for (it = res_table[nextLoc][nextT].begin(); it != res_table[nextLoc][nextT].end(); ++it) {
                    confs->push_back(std::shared_ptr<tuple<int, int, int, int, int, int>>(
                            new tuple<int, int, int, int, int, int>(
                                    it->second.agent_id, agent, nextLoc,
                                    -1, nextT, train_cell_number)));

                }
            }
        }

        if (goalTable.count(nextLoc)) {
            goalAgentList::iterator it;
            for (it = goalTable[nextLoc].begin(); it != goalTable[nextLoc].end(); ++it) {

                if (nextT > it->second) {
                    confs->push_back(std::shared_ptr<tuple<int, int, int, int, int, int>>(
                            new tuple<int, int, int, int, int, int>(
                                    it->first, agent, nextLoc, -1, nextT, 0)));

                }
            }
        }

        train_cell_number ++;
    }
			
			



    //detect edge conflict
    if (kDelay == 0) {
        int nextLoc = next_locs.front();
        if (res_table.count(nextLoc) && res_table[nextLoc].count(currT)) {
            agentList::iterator it;
            for (it = res_table[nextLoc][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {

                if (it->second.nextStep != NULL && it->second.nextStep->loc == currLoc) {
                    confs->push_back(std::shared_ptr<tuple<int, int, int, int, int, int>>(
                            new tuple<int, int, int, int, int, int>(
                                    agent, it->second.agent_id, currLoc, nextLoc, nextT, 0)));
                }


            }
        }
    }

	return confs;
}

int ReservationTable::countConflict(int agent, int currLoc, list<int> next_locs, int currT,int kDelay) {
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





