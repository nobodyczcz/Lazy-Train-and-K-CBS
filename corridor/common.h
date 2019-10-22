#pragma once
#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>
#include <google/dense_hash_map>
#include <iostream>
#include <list>
#include <unordered_map>


enum constraint_strategy { CBS, ICBS, CBSH, CBSH_CR, CBSH_R, CBSH_RM, CBSH_GR, STRATEGY_COUNT };

using std::pair;
using std::make_pair;
using std::unordered_map;
using std::list;
using std::vector;
namespace N
{
	template<typename T>
	void get(T); //no need to provide definition
				 // as far as enabling ADL is concerned!
}

struct PathEntry
{
	int location;
	bool single;
	PathEntry(int loc = -1) { location = loc; single = false; }
	std::list<int> locations; // all possible locations at the same time step
};


typedef std::vector<PathEntry> Path;

// Only for three-tuples of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
struct three_tuple_hash {
	template <class T1, class T2, class T3>
	std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
		using N::get;

		auto h1 = std::hash<T1>{}(get<0>(p));
		auto h2 = std::hash<T2>{}(get<1>(p));
		auto h3 = std::hash<T3>{}(get<2>(p));
		// Mainly for demonstration purposes, i.e. works but is overly simple
		// In the real world, use sth. like boost.hash_combine
		return h1 ^ h2 ^ h3;
	}
};
int getLocation(const std::vector<PathEntry>& path, int timestep);
