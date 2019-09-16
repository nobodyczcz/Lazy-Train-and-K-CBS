/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, Dec 2018
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/

#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSSearch.h"

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>


int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("debug", "debug mode")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("solver,s", po::value<std::string>()->required(), "solvers (CBS, ICBS, CBSH, CBSH-CR, CBSH-R, CBSH-RM")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<int>()->default_value(7200), "cutoff time (seconds)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("kDelay", po::value<int>()->default_value(0), "generate k-robust plan")
		("ignore-t0-conflict","Ignore t==0 k-delay conflict resolving")
		("asyConstraint","Using asymmetry range constraint to resolve k-delay conflict. Else use symmetry range constraint ")
		("short", "do not use long barrier constraint to resolve k delay rectangle conflict")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	// read the map file and construct its two-dim array
	MapLoader ml(vm["map"].as<string>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), ml, vm["agentNum"].as<int>());
 
	srand(vm["seed"].as<int>());
	options options1;
	if (vm.count("asyConstraint")) {
		options1.asymmetry_constraint = true;
	}
	else {
		options1.asymmetry_constraint = false;
	}
	if (vm.count("debug")) {
		options1.debug = true;
	}
	else {
		options1.debug = false;
	}
	if (vm.count("ignore-t0-conflict")) {
		options1.ignore_t0 = true;
	}
	else {
		options1.ignore_t0 = false;
	}
	if (vm.count("short")) {
		options1.shortBarrier = true;
	}
	else {
		options1.shortBarrier = false;
	}
	constraint_strategy s;
	if (vm["solver"].as<string>() == "ICBS")
		s = constraint_strategy::ICBS;
	else if (vm["solver"].as<string>() == "CBS")
		s = constraint_strategy::CBS;
	else if (vm["solver"].as<string>() == "CBSH")
		s = constraint_strategy::CBSH;
	else if (vm["solver"].as<string>() == "CBSH-CR")
		s = constraint_strategy::CBSH_CR;
	else if (vm["solver"].as<string>() == "CBSH-R")
		s = constraint_strategy::CBSH_R;
	else if (vm["solver"].as<string>() == "CBSH-RM")
		s = constraint_strategy::CBSH_RM;
	else
	{
		std::cout <<"WRONG SOLVER NAME!" << std::endl;
		return -1;
	}
	std::cout << "Start Searching" << endl;
	ICBSSearch icbs(ml, al, 1.0, s, vm["cutoffTime"].as<int>() * CLOCKS_PER_SEC, vm["kDelay"].as<int>(), options1);
	// what is the 1.0
	
	bool res;
	res = icbs.runICBSSearch();
	ofstream stats;
	stats.open(vm["output"].as<string>(), ios::app);
	stats << icbs.runtime / CLOCKS_PER_SEC << "," <<
		icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
		icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
		vm["agents"].as<string>() << "," << icbs.solution_cost << "," << 
		icbs.min_f_val - icbs.dummy_start->g_val << "," <<
		vm["solver"].as<string>()  << endl;
	stats.close();

	return 0;

}
