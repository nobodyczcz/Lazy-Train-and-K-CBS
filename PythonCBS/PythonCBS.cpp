#include "PythonCBS.h"
#include "flat_map_loader.h"
#include "MDD.h"

namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, bool debug) :railEnv(railEnv1) {
	std::cout << "algo: " << algo << std::endl;
	options1.debug = debug;
	options1.ignore_t0 = false;
	options1.shortBarrier = false;
	options1.asymmetry_constraint = false;
	timeLimit = t;
	this->algo = algo;
	this->kRobust = kRobust;
	if (algo == "ICBS")
		s = constraint_strategy::ICBS;
	else if (algo == "CBS")
		s = constraint_strategy::CBS;
	else if (algo == "CBSH")
		s = constraint_strategy::CBSH;
	else if (algo == "CBSH-CR")
		s = constraint_strategy::CBSH_CR;
	else if (algo == "CBSH-R")
		s = constraint_strategy::CBSH_R;
	else if (algo == "CBSH-RM")
		s = constraint_strategy::CBSH_RM;
	else
	{
		std::cout << "WRONG SOLVER NAME! Use CBSH as default" << std::endl;
		s = constraint_strategy::CBSH;
	}

	std::cout << "get width height " << std::endl;
	p::long_ rows(railEnv.attr("height"));
	p::long_ cols(railEnv.attr("width"));

	std::cout << "load map " << p::extract<int>(rows)<<" x "<< p::extract<int>(cols) << std::endl;
	//ml =  new MapLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));
	ml = new FlatlandLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));

	al =  new AgentsLoader(railEnv.attr("agents"));
	std::cout << "load done " << std::endl;
	if (debug) {
		al->printAgentsInitGoal();
	}

}

template <class Map>
p::list PythonCBS<Map>::getResult() {

	return icbs->outputPaths();
}

template <class Map>
bool PythonCBS<Map>::search() {
	icbs = new MultiMapICBSSearch <Map> (ml, *al, 1.0, s, timeLimit * CLOCKS_PER_SEC, kRobust, options1);
	bool res =false;
	res = icbs->runICBSSearch();

	
	return res;

}


template <class Map>
p::dict PythonCBS<Map>::getResultDetail() {
	p::dict result;

	result["runtime"] = icbs->runtime / CLOCKS_PER_SEC;
	result["HL_expanded"] = icbs->HL_num_expanded;
	result["HL_generated"] = icbs->HL_num_generated;

	result["LL_expanded"] = icbs->LL_num_expanded;
	result["LL_generated"] = icbs->LL_num_generated;
	if (icbs->isTimeout())
		result["solution_cost"] = -1;
	else
		result["solution_cost"] = icbs->solution_cost;
	result["algorithm"] = algo;
	result["No_f_rectangle"] = icbs->numOfRectangle;
	return result;

}


BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, int, int, bool>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail);
}