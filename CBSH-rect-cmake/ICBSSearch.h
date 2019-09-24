#pragma once

#include "ICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"
#include "RectangleReasoning.h"
#include <boost/python.hpp>


struct options {
	bool asymmetry_constraint;
	bool debug;
	bool ignore_t0;
	bool shortBarrier;
};




class ICBSSearch
{
public:
	double runtime = 0;
	double runtime_lowlevel;
	double runtime_conflictdetection;
	double runtime_computeh;
	double runtime_listoperation;
	double runtime_updatepaths;
	double runtime_updatecons;

	ICBSNode* dummy_start;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	bool solution_found;
	int solution_cost;
	double min_f_val;
	double focal_list_threshold;

	// Runs the algorithm until the problem is solved or time is exhausted 
	bool runICBSSearch();
	ICBSSearch() {};

	

	boost::python::list outputPaths();
	bool isTimeout() { return timeout; };


protected:

	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;

	constraint_strategy cons_strategy;
	int time_limit;
	std::clock_t start;

	double focal_w = 1.0;
	

	bool* my_map;
	int map_size;
	int num_of_agents;
	int* actions_offset;
	int* moves_offset;
	int num_col;
	AgentsLoader al;

	// k robust planning
	int kDelay;
	bool asymmetry_constraint;
	bool ignore_t0;
	bool debug_mode;
	bool shortBarrier;

	vector<vector<PathEntry>*> paths;
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found


	// high level search
	bool generateChild(ICBSNode* child, ICBSNode* curr);

	//conflicts
	void findConflicts(ICBSNode& curr);
	std::shared_ptr<tuple<int, int, int, int, int>> chooseEarliestConflict(ICBSNode &parent);
	std::shared_ptr<tuple<int, int, int, int, int>> classifyConflicts(ICBSNode &parent);
	void copyConflicts(const std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>>& conflicts,
		std::list<std::shared_ptr<std::tuple<int, int, int, int, int>>>& copy, int excluded_agent) const;

	// add heuristics for the high-level search
	int computeHeuristics(const ICBSNode& curr);
	bool KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k);

	
	//update information
	vector < list< pair<int, int> > >* collectConstraints(ICBSNode* curr, int agent_id);
	inline void updatePaths(ICBSNode* curr);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);
	void updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();

	// print
	void printPaths() const;
	void printStrategy() const;
	virtual void buildMDD(ICBSNode& curr, int id) {};
	virtual bool findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound = 0) { return false; };
	bool timeout = false;

};

template<class Map>
class MultiMapICBSSearch :public ICBSSearch
{
public:
	vector<SingleAgentICBS<Map> *> search_engines;  // used to find (single) agents' paths and mdd
	MultiMapICBSSearch(Map* ml, AgentsLoader& al, double f_w, constraint_strategy c, int time_limit, int kDlay, options options1);
	~MultiMapICBSSearch();
	// build MDD
	void buildMDD(ICBSNode& curr, int id);
	void initializeDummyStart();

	bool findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound = 0);



};

