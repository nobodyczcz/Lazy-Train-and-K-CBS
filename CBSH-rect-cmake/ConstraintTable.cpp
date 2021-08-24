#include "ConstraintTable.h"
void ConstraintTable::insert(int loc, int t_min, int t_max)
{
	if (t_max == 0) {
		CT_Single[loc].emplace(t_min);
		t_max = t_min + 1;
	}
	else {
		CT[loc].emplace_back(t_min, t_max);
	}
	
	if (loc == goal_location && t_max > length_min)
	{
		length_min = t_max;
	}
	if (t_max < INT_MAX && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
}

void ConstraintTable::insert_train(int loc, int t_min, int t_max)
{
    CT_Train[loc].emplace_back(t_min,t_max);

    if (loc == goal_location && t_max > length_min)
    {
        length_min = t_max;
    }
    if (t_max < INT_MAX && t_max > latest_timestep)
    {
        latest_timestep = t_max;
    }
}

void ConstraintTable::insert_parking(int loc, int t_min, int t_max)
{

    CT_Parking[loc].emplace_back(t_min, t_max);
    if (-loc == goal_location && t_max > length_min)
    {
        length_min = t_max;
    }
    if (t_max < INT_MAX && t_max > latest_timestep)
    {
        latest_timestep = t_max;
    }
}

void ConstraintTable::insert(std::list<Constraint> &constraints, int agent_id, int num_col, int map_size, int k) {
    for (auto constraint : constraints)
    {
        int x, y, z;
        constraint_type type;
        tie(x, y, z, type) = constraint;
        if (type == constraint_type::RANGE) // time range constraint
        {
            this->insert(x, y, z + 1);
        }
        else if (type == constraint_type::BARRIER) // barrier constraint
        {
            int x1 = x / num_col, y1 = x % num_col;
            int x2 = y / num_col, y2 = y % num_col;
            if (x1 == x2)
            {
                if (y1 < y2)
                    for (int i = 0; i <= std::min(y2 - y1, z); i++)
                    {
                        this->insert(x1 * num_col + y2 - i, z - i, 0);
                    }
                else
                    for (int i = 0; i <= std::min(y1 - y2, z); i++)
                    {

                        this->insert(x1 * num_col + y2 + i, z - i, 0);
                    }
            }
            else // y1== y2
            {
                if (x1 < x2)
                    for (int i = 0; i <= std::min(x2 - x1, z); i++)
                    {

                        this->insert((x2 - i) * num_col + y1, z - i, 0);
                    }
                else
                    for (int i = 0; i <= std::min(x1 - x2, z); i++)
                    {

                        this->insert((x2 + i) * num_col + y1, z - i, 0);
                    }
            }
        }
        else if (type == constraint_type::LENGTH)
        {
            if (x < 0 && y == agent_id)
            { // <-1, agent_id, t>: path of agent_id should be of length at least t + 1
                this->length_min = max(this->length_min, z + 1);
                if(this->length_min > this->latest_timestep)
                    this->latest_timestep = this->length_min;

            }
            else if (x >= 0 && y == agent_id)
            { // <loc, agent_id, t>: path of agent_id should be of length at most t
                this->length_max = min(this->length_max, z);

            }
            else if (x >= 0 && y != agent_id)
            { // <loc, agent_id, t>: any other agent cannot be at loc at or after timestep t - k
                this->insert(x, z - k, INT_MAX);
            }
        }
        else if (type == constraint_type::PARKING)
        {
            this->has_train = true;
            if (y == agent_id)
            { // a1 must not park at x from [0,z]
                this->insert_parking(x, 0, z+1);
            }
            else if (x >= 0 && y != agent_id)
            { // other must not use x after z
                this->insert_train(x, z, INT_MAX);
            }
        }
        else if (type == constraint_type::VERTEX)
        {
            this->insert(x, z, 0);
        }
        else if (type == constraint_type::TRAIN_VERTEX)
        {
            this->has_train = true;
            if (!(x == -1 && z==-1))
                this->insert_train(x, z, z+1);
        }
        else // edge
        {
            this->insert(x * map_size + y, z, 0);
        }
    }
}

bool ConstraintTable::is_constrained(int loc, int t, bool body)
{
    if (CT_Train.count(loc)){
        for (auto constraint: CT_Train[loc]){
            if (constraint.first <= t && t < constraint.second)
                return true;
        }
    }

    if (body){
        return false;
    }

	if (CT_Single.count(loc)) {
		if (CT_Single[loc].count(t)) {
			return true;
		}
	}
	auto it = CT.find(loc);
	if (it == CT.end())
	{
		return false;
	}
	for (auto constraint : it->second)
	{
		if (constraint.first <= t && t < constraint.second)
			return true;
	}
	return false;
}

bool ConstraintTable::is_parking_constrained(list<int> occupation, int t)
{
    int loc = 0;

    //check for if all positive parking constraints satisfied
    for(auto constraints: CT_Parking){
        loc = constraints.first;

        if (loc < 0)
            continue;
        for (auto constraint: constraints.second){
            bool satisfied = false;
            for (int l : occupation){ //if any occupation cell satisfy positive constraint
                if (l == loc && t >= constraint.first && t < constraint.second){
                    satisfied = true;
                }
            }
            if (!satisfied){
                return true;
            }
        }
    }

    for(auto l: occupation){
        if (CT_Parking.count(-l)){//negative parking, must not park
            for (auto constraint: CT_Parking[-l]){
                if (t > constraint.first && t < constraint.second ){
                    return true;
                }
            }
        }
    }
    return false;
}
