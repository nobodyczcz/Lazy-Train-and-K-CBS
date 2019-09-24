#pragma once

#include <vector>
#include <utility>
#include <stdlib.h>
#include "map_loader.h"

using namespace std;

struct hvals {
	int heading[5];
};
template<class Map>
class ComputeHeuristic 
{
 public:
  int start_location;
  int goal_location;
  int start_heading;
  Map* ml;
  int map_rows;
  int map_cols;
  ComputeHeuristic();
  ComputeHeuristic(int start_location, int goal_location, Map* ml0, int start_heading = 4);
 
 bool validMove(int curr, int next) const;

 void getHVals(vector<hvals>& res);


  ~ComputeHeuristic();

};

//class FlatlandComputeHeuristic :public ComputeHeuristic
//{
//public:
//	FlatlandComputeHeuristic(int start_location, int goal_location, FlatlandLoader* fl0, int start_heading = 4);
//	vector<pair<int, int>> get_transitions(int loc, int heading, int noWait = false);
//private:
//	FlatlandLoader* fl;
//
//};

