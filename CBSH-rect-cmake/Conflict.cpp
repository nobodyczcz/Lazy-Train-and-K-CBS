#include "Conflict.h"

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint) << "," <<
		std::get<2>(constraint) << "," << std::get<3>(constraint) << ">";
	return os;
}

// add a horizontal modified barrier constraint
bool addModifiedHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)path.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
		std::list<int>::const_iterator it = std::find(path[t2].locations.begin(), path[t2].locations.end(), loc);
		if (it == path[t2].locations.end() && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			int loc2 = (Ri_y + (t2 - 1 - Ri_t) * sign) + x * num_col;
			constraints.emplace_back(loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != path[t2].locations.end() && t1 < 0)
		{
			t1 = t2;
		}
		if (it != path[t2].locations.end() && t2 == t_max)
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			constraints.emplace_back(loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

// add a vertival modified barrier constraint
bool addModifiedVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)path.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
		std::list<int>::const_iterator it = std::find(path[t2].locations.begin(), path[t2].locations.end(), loc);
		if (it == path[t2].locations.end() && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			int loc2 = (Ri_x + (t2 - 1 - Ri_t) * sign) * num_col + y;
			constraints.emplace_back(loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != path[t2].locations.end() && t1 < 0)
		{
			t1 = t2;
		}
		if (it != path[t2].locations.end() && t2 == t_max)
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			constraints.emplace_back(loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}


std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	switch (conflict.p)
	{
		case conflict_priority::CARDINAL:
			os << "cardinal ";
			break;
		case conflict_priority::SEMI:
			os << "semi-cardinal ";
			break;
		case conflict_priority::NON:
			os << "non-cardinal ";
			break;
	}
	switch (conflict.type)
	{
		case conflict_type::STANDARD:
			os << "standard";
			break;
		case conflict_type::RECTANGLE:
			os << "rectangle";
			break;
		case conflict_type::CORRIDOR2:
			os << "corrdior2";
			break;
		case conflict_type::CORRIDOR4:
			os << "corrdior4";
			break;
		case conflict_type::TARGET:
			os << "target";
	}
	os << " conflict:  " << conflict.a1 << " with ";
	for (auto con : conflict.constraint1)
		os << con << ",";		
	os << " and " << conflict.a2 << " with ";
	for (auto con : conflict.constraint2)
		os << con << ",";		
	os << std::endl;
	return os;
}

bool operator < (const Conflict& conflict1, const Conflict& conflict2) // return true if conflict2 has higher priority
{
	if (conflict1.type == conflict_type::TARGET && conflict2.type == conflict_type::TARGET)
	{
		if (conflict1.p < conflict2.p)
			return false;
		else
			return true;
	}
	else if (conflict1.type == conflict_type::TARGET)
		return false;
	else if (conflict2.type == conflict_type::TARGET)
		return true;
	
	if (conflict1.p < conflict2.p)
		return false;
	else if (conflict1.p > conflict2.p)
		return true;
	else if (conflict1.p == conflict_priority::CARDINAL) // both are cardinal
	{
		if (conflict1.type == conflict_type::CORRIDOR2)
		{
			if (conflict2.type != conflict_type::CORRIDOR2)
				return false;
		}
		else if (conflict1.type == conflict_type::CORRIDOR4)
		{
			if (conflict2.type == conflict_type::CORRIDOR2)
			{
				return true;
			}
			else if (conflict2.type != conflict_type::CORRIDOR4)
				return false;
		}
		else if (conflict2.type == conflict_type::CORRIDOR4 || conflict2.type == conflict_type::CORRIDOR2)
		{
			return true;
		}
	}
	else // both are semi or both are non 
	{
		if (conflict2.type == conflict_type::CORRIDOR2 &&  conflict1.type != conflict_type::CORRIDOR2)
		{
			return true;
		}
		else	if (conflict2.type != conflict_type::CORRIDOR2 &&  conflict1.type == conflict_type::CORRIDOR2)
		{
			return false;
		}
		/*if (conflict2.type == conflict_type::RECTANGLE &&  conflict1.type != conflict_type::RECTANGLE)
		{
			return true;
		}*/
	}
	if (conflict2.t < conflict1.t)
	{
		return true;
	}
	else
		return false;
}