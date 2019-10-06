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
	PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, bool debug, string corridor);

	p::list getResult();

	bool search();
	p::dict getResultDetail();
private:
	std::string algo;
	p::object railEnv;
	FlatlandLoader* ml;
	AgentsLoader* al;
	constraint_strategy s;
	options options1;
	int timeLimit;
	int kRobust;
	MultiMapICBSSearch<Map>* icbs;
	bool corridor2=false;
	bool corridor4=false;
	bool trainCorridor1 = false;
	bool trainCorridor2 = false;


	
};


