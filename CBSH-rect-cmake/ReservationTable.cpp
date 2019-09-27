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
	}
}

ConflictList* ReservationTable::findConflict(int agent, int currLoc, int nextLoc, int currT,int kDelay) {
	ConflictList* confs =  new ConflictList;
	int nextT = currT + 1;
	if (res_table.count(nextLoc)) {
		for (int k = -kDelay;  k <= kDelay; k++) {
			int t = nextT + k;
			if (res_table[nextLoc].count(t)) {
				agentList::iterator it;
				for (it = res_table[nextLoc][t].begin(); it != res_table[nextLoc][t].end(); ++it) {
					confs->push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(k < 0 ? it->second.agent_id : agent, k < 0 ? agent : it->second.agent_id, nextLoc, -1, k < 0 ? t : nextT)));

				}
			}
		}
		if (kDelay == 0) {
			if (res_table[nextLoc].count(currT)) {
				agentList::iterator it;
				for (it = res_table[nextLoc][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {
					if(it->second.nextStep!= NULL && it->second.nextStep->loc == currLoc)
						confs->push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(agent, it->second.agent_id, currLoc, nextLoc, nextT)));
				}
			}
		}
		
	}
	return confs;


}




