#pragma once
#include "Conflict.h"
#include <limits.h>

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

private:
	unordered_map<size_t, list<pair<int, int> > > CT;
	
};
