#include <iostream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include "FlatAgentLoader.h.h"


#include "ICBSSearch.h"


namespace p = boost::python;

template <class Map>
class PythonCBS {
public:
	PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, bool debug, float f_w, string corridor,int RM_4Way);

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
	float f_w;
	MultiMapICBSSearch<Map>* icbs;
	bool corridor2=false;
	bool corridor4=false;
	bool trainCorridor1 = false;
	bool trainCorridor2 = false;


	
};


