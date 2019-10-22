#pragma once
#include "common.h"
// #include "RectangleReasoning.h"

enum conflict_type { TARGET, CORRIDOR2, CORRIDOR4, RECTANGLE, STANDARD, TYPE_COUNT };
enum conflict_priority { CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };

enum constraint_type { LENGTH, RANGE, BARRIER, VERTEX, EDGE, CONSTRAINT_COUNT };

typedef std::tuple<int, int, int, constraint_type> Constraint;
// <loc, -1, t, VERTEX>
// <from, to, t, EDGE> 
// <B1, B2, t, RECTANGLE>
// <loc, t1, t2, CORRIDOR> 
// <loc, agent_id, t, TARGET>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <-1, agent_id, t>: path of agent_id should be of length at least t + 1 



std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

// add a horizontal modified barrier constraint
bool addModifiedHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints);

// add a vertival modified barrier constraint
bool addModifiedVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints);



class Conflict
{
public:
	int a1;
	int a2;
	int t;
	std::list<Constraint> constraint1;
	std::list<Constraint> constraint2;
	conflict_type type;
	conflict_priority p = conflict_priority::UNKNOWN;

	void vertexConflict(int a1, int a2, int v, int t)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(v, -1, t, constraint_type::VERTEX);
		this->constraint2.emplace_back(v, -1, t, constraint_type::VERTEX);
		type = conflict_type::STANDARD;
	}
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	void corridorConflict(int a1, int a2, int v1, int v2, int t3, int t4, int t3_, int t4_, int k)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = std::min(t3, t4);
		this->constraint1.emplace_back(v1, t3, std::min(t3_ - 1, t4 + k), constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t4, std::min(t4_ - 1, t3 + k), constraint_type::RANGE);
		type = conflict_type::CORRIDOR2;
	}


	void corridorConflict(int a1, int a2, int v1, int v2, int t1, int t2, int k, int h)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = std::min(t1, t2);
		this->constraint1.emplace_back(v1, t1, t1 + 2 * k - 1, constraint_type::RANGE);
		this->constraint1.emplace_back(v2, t1 + k, std::min(t2 + 2 * k, t1 + h - 1), constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t2, t2 + 2 * k - 1, constraint_type::RANGE);
		this->constraint2.emplace_back(v1, t2 + k, std::min(t1 + 2 * k, t2 + h - 1), constraint_type::RANGE);
		type = conflict_type::CORRIDOR4;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int move1, int move2, int Rg_t, const std::vector<Path*>& paths, int num_col) // For GR
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		if (abs(move1) == 1 || abs(move2) > 1) // first agent moves horizontally and second agent moves vertically
		{
			if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		else
		{
			if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		type = conflict_type::RECTANGLE;
		return true;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t, const std::vector<Path*>& paths, int num_col) // For RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);

		if (s1.first == s2.first)
		{
			if ((s1.second - s2.second) * (s2.second - Rg.second) >= 0)
			{
				// first agent moves horizontally and second agent moves vertically
				if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
				{
					return false;
				}
				if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
				{
					return false;
				}
			}
			else
			{
				// first agent moves vertically and second agent moves horizontally
				if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
				{
					return false;
				}
				if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
				{
					return false;
				}
			}
		}
		else if ((s1.first - s2.first)*(s2.first - Rg.first) >= 0)
		{
			// first agent moves vertically and second agent moves horizontally
			if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		else
		{
			// first agent moves horizontally and second agent moves vertically
			if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		type = conflict_type::RECTANGLE;
		return true;
	}

	void targetConflict(int a1, int a2, int v, int t)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(-1, a1, t, constraint_type::LENGTH); // length of a1 should be larger than t
		this->constraint2.emplace_back(v, a1, t, constraint_type::LENGTH); // length of a1 should be no larger than t, and other agents can not use v at and after timestep t
		type = conflict_type::TARGET;
	}
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);

