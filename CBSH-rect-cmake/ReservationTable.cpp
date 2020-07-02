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
		int loc = path->at(t).location;
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

void ReservationTable::addPaths(vector<vector<PathEntry>*>* paths,int exclude) {
	for (int agent = 0; agent < paths->size(); agent++) {
		if (agent == exclude || (*paths)[agent]==NULL)
			continue;
		addPath(agent, (*paths)[agent]);
	}
}

void ReservationTable::deletePath(int agent_id, std::vector<PathEntry>* path) {
	for (int t = 0; t < path->size(); t++) {
		int loc = (*path)[t].location;
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

OldConfList* ReservationTable::findConflict(int agent, int currLoc, int nextLoc, int currT,int kDelay, bool ignore_goal_table) {
	OldConfList* confs =  new OldConfList;
	int nextT = currT + 1;
	//cout << "currloc " << currLoc << " nextloc " << nextLoc << endl;
	if (res_table.count(nextLoc)) {
		//detect vertex conflict and k delay vertex conflict
		for (int k = -kDelay;  k <= kDelay; k++) {

			int t = nextT + k;

			if (res_table[nextLoc].count(t)) {
				agentList::iterator it;
				for (it = res_table[nextLoc][t].begin(); it != res_table[nextLoc][t].end(); ++it) {
					confs->push_back(std::shared_ptr<tuple<int, int, int, int, int,int>>(
						new tuple<int, int, int, int, int,int>(
							k < 0 ? it->second.agent_id : agent, k < 0 ? agent : it->second.agent_id, nextLoc, -1, k < 0 ? t : nextT,k>=0?k:-k)));

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
		//detect edge conflict, we do not detect k delay edge conflict, because, every k delay edge conflict cause k delay vertex conflit.
		//every edge conflict cause two k delay vertex conflict, thus don't need to detect edge conflicct when k is not 0.
		if (kDelay == 0) {
			if (res_table[nextLoc].count(currT)) {
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
		
		
	}
	return confs;


}

int ReservationTable::countConflict(int agent, int currLoc, int nextLoc, int currT,int kDelay) {
    int nextT = currT + 1;
    if(nextLoc == -1)
        return 0;
    int count = 0;

    if (res_table.count(nextLoc)) {
        //detect vertex conflict and k delay vertex conflict
        for (int k = -kDelay;  k <= kDelay; k++) {

            int t = nextT + k;

            if (res_table[nextLoc].count(t)) {
                count++;
            }


        }

        //detect edge conflict, we do not detect k delay edge conflict, because, every k delay edge conflict cause k delay vertex conflict.
        //every edge conflict cause two k delay vertex conflict, thus don't need to detect edge conflict when k is not 0.
        if (kDelay == 0) {
            if (res_table[nextLoc].count(currT)) {
                agentList::iterator it;
                for (it = res_table[nextLoc][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {

                    if (it->second.nextStep != NULL && it->second.nextStep->loc == currLoc) {
                        count++;
                    }

                }
            }
        }


    }
    return count;
}





