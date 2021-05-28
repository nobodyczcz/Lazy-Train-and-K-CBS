#pragma once

#include <boost/unordered_map.hpp>
#include <unordered_set> 
#include "common.h"
#include "agents_loader.h"

#include <memory>
using namespace std;



class AgentStep {
public:
	AgentStep() {};
	AgentStep(int id, int l, int t0,bool head0) {
		agent_id = id;
		loc = l;
		t = t0;
		head = head0;
	}
	int agent_id;
	int loc;
	int t;
	bool head;
	AgentStep* preStep = NULL;
	AgentStep* nextStep = NULL;

	bool operator == (AgentStep const& s2)
	{
		return agent_id == s2.agent_id;
	}
};


class ReservationTable {
public:
	struct eqint
	{
		bool operator()(int s1, int s2) const
		{
			return s1 == s2;
		}
	};

	typedef boost::unordered_map<int, AgentStep> agentList;//stores agents in a loc at a certain timestep
	typedef boost::unordered_map<int, agentList> timeline;//key is time step, value is agentlist
	typedef boost::unordered_map<int, timeline> map_table;//hash table, key is map location, value is time line
	typedef boost::unordered_map<int, int> goalAgentList;
	boost::unordered_map<int, goalAgentList> goalTable;
	map_table res_table;
	AgentsLoader* agentsLoader;
	void addPath(int agent_id, std::vector<PathEntry>* path);
	void addPaths(vector<vector<PathEntry>*>* paths, int exclude = -1);
	void deletePath(int agent_id, std::vector<PathEntry>* path);
	OldConfList* findConflict(int agent, int currLoc, list<int> next_locs, int currT, bool ignore_goal_table = false);
	int countConflict(int agent, int currLoc, list<int> next_locs, int currT);



	ReservationTable(int mapSize, AgentsLoader* agentsLoader);
	ReservationTable(int mapSize, vector<vector<PathEntry>*>* paths,AgentsLoader* agentsLoader, int exclude=-1);

};