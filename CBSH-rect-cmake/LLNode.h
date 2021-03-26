#pragma once
#include <boost/heap/fibonacci_heap.hpp>
#include <list>
#include <functional>  // for std::hash (c++11 and above)
#include <memory>
#include "common.h"
using boost::heap::fibonacci_heap;
using boost::heap::compare;




class LLNode
{
public:

	list<int> locs;
	int g_val;
	int h_val = 0;
	int heading;
	bool train_mode = false;
	int actionToHere = 4;
	std::vector<int> possible_next_heading;
	LLNode* parent=NULL;
	int timestep = 0;
	int time_generated=0;
	int num_internal_conf = 0; 
	bool in_openlist = false;
	bool in_focallist = false;
	OldConfList* conflist=NULL;


	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const 
		{
			if (n1->getFVal() == n2->getFVal())
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			return n1->getFVal() >= n2->getFVal();
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

	// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node
	{
		bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
			//if (n1->g_val == n2->g_val)
			//{
			//	if (n1->num_internal_conf == n2->num_internal_conf) {

			//		if (n1->timestep == n2->timestep) {
			//			return n1->time_generated > n2->time_generated; // break ties towards earlier generated nodes - 
			//															//explore the tree more broadly (despite preferring depth)
			//	 														 //(time_generated can't be equal)
			//		}
			//		return n1->timestep < n2->timestep;  // break ties towards *more* depth - more work was done, even if it didn't reduce the number of conflicts or increase the cost yet

			//	}
			//	return n1->num_internal_conf > n2->num_internal_conf;  // break ties towards fewer conflicts
			//}
			//return n1->g_val < n2->g_val;// break ties towards *larger* g_val 

			//
			//if (n1->num_internal_conf == n2->num_internal_conf) {
			//	if (n1->g_val == n2->g_val)
			//	{

			//		return n1->time_generated > n2->time_generated; // break ties towards earlier generated nodes -
			//														// explore the tree more broadly (despite preferring depth)
			//														// (time_generated can't be equal)
			//	}
			//	return n1->g_val < n2->g_val;// break ties towards *larger* g_val 
			//}
			//return n1->num_internal_conf > n2->num_internal_conf;  // break ties towards fewer conflicts
			////
			//
			//if (n1->num_internal_conf == n2->num_internal_conf) {
			//	if (n1->g_val == n2->g_val)
			//	{
			//		if (n1->h_val == n2->h_val) {
			//		

			//				return n1->timestep <= n2->timestep;  // break ties towards *more* depth - more work was done, even if it didn't reduce the number of conflicts or increase the cost yet

			//		

			//		}
			//		return n1->h_val >= n2->h_val;
			//	}
			//	return n1->g_val < n2->g_val;// break ties towards *larger* g_val 
			//}
			//return n1->num_internal_conf > n2->num_internal_conf;  // break ties towards fewer conflicts
			

		
			if (n1->num_internal_conf == n2->num_internal_conf)
			{
				
				if (n1->g_val == n2->g_val)
				{
                    return n1->h_val >= n2->h_val;



//                    if (rand() % 2 == 0)
//						return true;
//					else
//						return false;
				}
				return n1->g_val <= n2->g_val;
					
			}
			return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


	// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef boost::heap::fibonacci_heap< LLNode*, compare<LLNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< LLNode*, compare<LLNode::secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;


	LLNode();
	LLNode(const LLNode& other);
	LLNode(list<int> locs, int g_val, int h_val, LLNode* parent, int timestep,
		int num_internal_conf = 0, bool in_openlist = false, bool train_mode = false);
	inline double getFVal() const { return g_val + h_val; }
	~LLNode(){
	}

	// The following is used by googledensehash for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestep and same heading
	struct eqnode 
	{
		bool operator()(const LLNode* s1, const LLNode* s2) const 
		{
		    if (s1 == s2)
		        return true;
		    if (!s1 || !s2 || s1->timestep != s2->timestep || s1->locs.size() != s2->locs.size() || s1->heading != s2->heading)
		        return false;
		    assert(s1->train_mode == s2->train_mode);
            bool same = true;
		    if(s1->train_mode && s2->train_mode){
                auto s1_it = s1->locs.begin();
                auto s2_it = s2->locs.begin();
                while (s1_it != s1->locs.end() && s2_it != s2->locs.end()){
                    if(*s1_it != *s2_it) {
                        same = false;
                        break;
                    }
                    s1_it++;
                    s2_it++;
                }
		    }
		    else{
		        same = s1->locs.front() == s2->locs.front();
		    }

			return same;
		}
	};

	// The following is used by googledensehash for generating the hash value of a nodes
	struct NodeHasher 
	{
		std::size_t operator()(const LLNode* n) const 
		{
            int loc_multi = 1;

            if (n->train_mode) {
                for (int loc : n->locs) {
                    loc_multi = loc_multi * loc;
                }
            }
		    else{
		        loc_multi = n->locs.front();
		    }
			size_t loc_hash = std::hash<int>()(loc_multi);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			size_t heading = std::hash<int>()(n->heading);

			return (loc_hash ^ (timestep_hash << 1)*(heading << 1));
		}
	};

};

