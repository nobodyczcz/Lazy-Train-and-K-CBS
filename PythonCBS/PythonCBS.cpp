#include "PythonCBS.h"
#include "MDD.h"

namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, int screen, float f_w,bool corridor, bool diff_k, bool lltp_only) :railEnv(railEnv1) {

	timeLimit = t;
	this->f_w = f_w;
	this->algo = algo;
	this->kRobust = kRobust;
	this->corridor2 = corridor;
    this->screen = screen;
    this->algo = algo;

	options1.ignore_target = true;
	options1.shrink = true;
	options1.parking = false;
	options1.lltp_only = lltp_only;




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
	else if (algo == "CBSH-GR")
		s = constraint_strategy::CBSH_GR;
	else
	{
		std::cout << "WRONG SOLVER NAME! Use CBSH as default" << std::endl;
		s = constraint_strategy::CBSH;
	}

	p::long_ rows(railEnv.attr("height"));
	p::long_ cols(railEnv.attr("width"));

	//ml =  new MapLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));
	ml = new FlatlandLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));

	al =  new AgentsLoader(railEnv.attr("agents"),kRobust, diff_k);
	if (screen >=2) {
		al->printAgentsInitGoal();
	}

}

template <class Map>
p::list PythonCBS<Map>::getResult() {

//	return icbs->outputPaths();
}

template <class Map>
bool PythonCBS<Map>::search() {

	icbs = new MultiMapICBSSearch <Map> (ml, *al, f_w, s, timeLimit * CLOCKS_PER_SEC,screen, kRobust, options1);
	if(s == constraint_strategy::CBSH_RM)
		icbs->rectangleMDD = true;
	icbs->corridor2 = corridor2;
	bool res =false;
	res = icbs->runICBSSearch();

	
	return res;

}


template <class Map>
void PythonCBS<Map>::write_result(string path) {
    bool valid_train = icbs->isValidTrain();
    icbs->write_data(path,"", algo, valid_train);
}

template class PythonCBS<FlatlandLoader>;

BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<p::object, std::string, int, int, int, float,bool, bool, bool>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("write_result", &PythonCBS<FlatlandLoader>::write_result);
}