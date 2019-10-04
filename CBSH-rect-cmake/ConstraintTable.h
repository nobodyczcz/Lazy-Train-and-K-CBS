#pragma once
#include "Conflict.h"
#include <climits>
#include <unordered_set>
class ConstraintTable
{
public:
	int length_min = 0;
	int length_max = INT_MAX;
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.

	void clear(){CT.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}
	void insert(int loc, int t_min, int t_max);
	bool is_constrained(int loc, int t);
	void printSize() {
		std::cout << CT.size() << std::endl;;
	};

private:
	unordered_map<size_t, std::unordered_set<int> > CT;
	
};

