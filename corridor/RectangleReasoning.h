#pragma once
#include <tuple>
#include <vector>
#include <list>

#include "LLNode.h"
#include "ICBSNode.h"
#include "MDD.h"

// add a pair of barrier constraints
void addBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

// add a pair of modified barrier constraints
bool addModifiedBarrierConstraints(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, 
	int S1_t, int S2_t, int Rg, int num_col, 
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

//// add a vertival modified barrier constraint
//bool addModifiedVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
//	int Ri_x, int Rg_x, int Rg_t, int num_col,
//	std::list<std::tuple<int, int, int>>& constraints);
//
//// add a horizontal modified barrier constraint
//bool addModifiedHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
//	int Ri_y, int Rg_y, int Rg_t, int num_col,
//	std::list<std::tuple<int, int, int>>& constraints);

//Identify rectangle conflicts
bool isRectangleConflict(const std::pair<int,int>& s1, const std::pair<int, int>& s2, 
	const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t);// for CR and R
bool isRectangleConflict(int s1, int s2, int g1, int g2, int num_col);// for RM

//Classify rectangle conflicts
int classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2);// for CR and R
int classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg, int num_col);// for RM

//Compute rectangle corners
std::pair<int, int> getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2);
std::pair<int, int> getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1);

//Compute start and goal candidates for RM
std::list<int> getStartCandidates(const std::vector<PathEntry>& path, int timestep, int num_col);
std::list<int> getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int num_col);

// whether the path between loc1 and loc2 is Manhattan-optimal
bool isManhattanOptimal(int loc1, int loc2, int dt, int num_col);

//// whther two rectangle conflicts are idenitical
//bool equalRectangleConflict(const std::tuple<int, int, int, int, int>& c1, const std::tuple<int, int, int, int, int>& c2);
//
//// find duplicate rectangle conflicts, used to detect whether a semi-/non-cardinal rectangle conflict is unique
//bool findRectangleConflict(const ICBSNode* curr, const std::tuple<int, int, int, int, int>& conflict); 

std::list<int>  getStartCandidates(const std::vector<PathEntry>& path, int timestep, int dir1, int dir2);

std::list<int>  getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int dir1, int dir2);

bool ExtractBarriers(const MDD& mdd, int dir1, int dir2, int start, int goal, int start_time, int num_col, std::list<Constraint>& B);

std::pair<int, int> getIntersection(const Constraint& b1, const Constraint& b2, int num_col);
bool isEntryBarrier(const Constraint& b1, const Constraint& b2, int dir1, int num_col);
bool isExitBarrier(const Constraint& b1, const Constraint& b2, int dir1, int num_col);

void getCorners(const Constraint& b1, const Constraint& b2, int dir1, int dir2, int num_col,
	std::pair<int, int>& R1, std::pair<int, int>& R2, std::pair<int, int>& Rs, std::pair<int, int>& Rg);

bool isCut(const Constraint b, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int num_col);

bool blockedNodes(const MDD& mdd, const Constraint b, int num_col);
bool blockedNodes(const std::vector<PathEntry>& path, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int Rg_t, int dir, int num_col);

void generalizedRectangle(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, 
	const MDD& mdd1, const MDD& mdd2,
	const std::list<Constraint>::const_iterator& b1_entry, const std::list<Constraint>::const_iterator& b2_entry,
	const std::list<Constraint>& B1, const std::list<Constraint>& B2, int timetep, int num_col,
	int& best_type, std::pair<int, int>& best_Rs, std::pair<int, int>& best_Rg, int time_limit, std::set<std::pair<int, int>> &visitedRs);