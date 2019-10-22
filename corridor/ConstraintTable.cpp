#include "ConstraintTable.h"

void ConstraintTable::insert(int loc, int t_min, int t_max)
{
	CT[loc].emplace_back(t_min, t_max);
	if (loc == goal_location && t_max > length_min)
	{
		length_min = t_max;
	}
	if (t_max < INT_MAX && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
}

bool ConstraintTable::is_constrained(int loc, int t)
{
	auto it = CT.find(loc);
	if (it == CT.end())
	{
		return false;
	}
	for (auto constraint: it->second)
	{
		if (constraint.first <= t && t < constraint.second)
			return true;
	}
	return false;
}