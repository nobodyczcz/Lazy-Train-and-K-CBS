#include <iostream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include "flat_map_loader.h"


#include "ICBSSearch.h"


namespace p = boost::python;

template <class Map>
class PythonCBS {
public:
	PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, bool debug);

	p::object printResult();

	p::list search();
private:
	p::object railEnv;
	FlatlandLoader* ml;
	AgentsLoader* al;
	constraint_strategy s;
	options options1;
	int timeLimit;
	int kRobust;

	
};


