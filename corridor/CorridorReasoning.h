#pragma once
#include "common.h"
#include "ConstraintTable.h"

typedef std::tuple<int, int, int> CottidorTable_Key; // endpoint1, endpoint2, length
typedef std::unordered_map<CottidorTable_Key, int, three_tuple_hash> CorridorTable; // value = length of the bypass


bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons);

int getDegree(int loc, const bool*map, int num_col, int map_size);

int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
	const bool*map, int num_col, int map_size);
int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge);


int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size);
int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size, ConstraintTable& constraint_table, int upper_bound);

bool validMove(int curr, int next, int map_cols, int map_size);

int getMahattanDistance(int loc1, int loc2, int map_cols);