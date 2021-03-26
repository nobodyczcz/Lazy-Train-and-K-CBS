#pragma once
#include "Conflict.h"
#include <climits>
#include <unordered_set>
#include <unordered_map>

class ConstraintTable
{
public:
	int length_min = 0;
	int length_max = INT_MAX;
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.
	bool has_train = false;

	void clear(){
		CT.clear(); 
		length_min = 0, 
		length_max = INT_MAX; 
		latest_timestep = 0;
		has_train = false;
		CT_Single.clear();
	}
	void insert(int loc, int t_min, int t_max);
	void insert(std::list<Constraint>& constraints, int agent_id, int num_col, int map_size);
	bool is_constrained(int loc, int t);
	void printSize() {
		std::cout << CT.size() << std::endl;;
	};

private:
	std::unordered_map<size_t, std::unordered_set<int> > CT_Single;
	std::unordered_map<size_t, list<pair<int, int> > > CT;

	
};

