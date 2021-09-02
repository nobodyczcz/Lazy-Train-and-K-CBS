#include <iostream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include "agents_loader.h"


#include "ICBSSearch.h"


namespace p = boost::python;

template <class Map>
class PythonCBS {
public:
    PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, int screen, float f_w,bool corridor, bool diff_k, bool lltp_only);
    PythonCBS(){};
	p::list getResult();

	bool search();
	void write_result(string path);
private:
	std::string algo;
	p::object railEnv;
	FlatlandLoader* ml;
    AgentsLoader* al;
	constraint_strategy s;
	options options1;
	int timeLimit;
	int kRobust;
	int screen;
	float f_w;
	MultiMapICBSSearch<Map>* icbs;
	bool corridor2=false;
	bool corridor4=false;
	bool trainCorridor1 = false;
	bool trainCorridor2 = false;


	
};


