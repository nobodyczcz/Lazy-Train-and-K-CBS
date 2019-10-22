#pragma once

#include "ICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include "ConstraintTable.h"

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

	uint64_t num_corridor2 = 0;
	uint64_t num_corridor4 = 0;
	uint64_t num_rectangle = 0;
	uint64_t num_target = 0;
	uint64_t num_standard = 0;

	bool solution_found;
	int solution_cost;
	double min_f_val;
	double focal_list_threshold;
	bool cardinalRect = false;
	bool rectangleMDD = false;
	bool corridor2 = false;
	bool corridor4 = false;
	bool cardinalCorridorReasoning = false;
	bool targetReasoning;

	// Runs the algorithm until the problem is solved or time is exhausted 
	bool runICBSSearch();

	ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, constraint_strategy c, int time_limit, int screen);
	~ICBSSearch();

private:
	
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;

	CorridorTable corridorTable;

	ConstraintTable constraintTable;

	std::vector<MDDTable> mddTable;
	constraint_strategy cons_strategy;
	const int time_limit;
	double focal_w = 1.0;
	int screen;

	const bool* my_map;
	int map_size;
	int num_of_agents;
	const int* actions_offset;
	const int* moves_offset;
	int num_col;
	AgentsLoader al;

	vector<vector<PathEntry>*> paths;
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found
	vector < SingleAgentICBS* > search_engines;  // used to find (single) agents' paths and mdd


	// high level search
	bool findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound = 0);
	bool generateChild(ICBSNode* child, ICBSNode* curr);
	bool generated_root_node();
	//conflicts
	void findConflicts(ICBSNode& curr);
	void findConflicts(ICBSNode& curr, int a1, int a2);
	void  classifyConflicts(ICBSNode &parent);
	 std::shared_ptr<Conflict> chooseConflict(ICBSNode &parent);
	void copyConflicts(const std::list<std::shared_ptr<Conflict>>& conflicts,
		std::list<std::shared_ptr<Conflict>>& copy, const list<int>& excluded_agent) const;
	// void copyConflicts(const std::list<std::shared_ptr<CConflict>>& conflicts,
	// 	std::list<std::shared_ptr<CConflict>>& copy, int excluded_agent) const;
	// void deleteRectConflict(ICBSNode& curr, const Conflict& conflict);
	bool isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node);
	bool hasCardinalConflict(const ICBSNode& node) const;
	bool blocked(const Path& path, const std::list<Constraint>& constraint) const;
	bool traverse(const Path& path, int loc, int t) const;
	void removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts) const;

	// add heuristics for the high-level search
	int computeHeuristics(const ICBSNode& curr);
	bool KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k);

	// build MDD
	MDD* buildMDD(ICBSNode& curr, int id); 

	//update information
	// vector < list< pair<int, int> > >* collectConstraints(ICBSNode* curr, int agent_id);
	void updateConstraintTable(ICBSNode* curr, int agent_id);
	inline void updatePaths(ICBSNode* curr);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);
	void updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();

	// print
	void printPaths() const;
	void printStrategy() const;
};

