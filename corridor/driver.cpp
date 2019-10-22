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
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("solver,s", po::value<std::string>()->required(), "solvers (CBS, ICBS, CBSH, CBSH-CR, CBSH-R, CBSH-RM, CBSH-GR")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<int>()->default_value(7200), "cutoff time (seconds)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("screen", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("cardinalRect", po::value<bool>(), "only consider cardinal rectangle conflicts")
		("corridor2", po::value<bool>(), "reason about 2-way branching corridor conflicts")
		("corridor4", po::value<bool>(), "reason about 4-way branching corridor conflicts")
		("cardinalCorridor", po::value<bool>(), "only reason about cardinal corridor conflicts")
		("target", po::value<bool>(), "reason about target conflict")
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
	else if (vm["solver"].as<string>() == "CBSH-GR")
		s = constraint_strategy::CBSH_GR;
	else
	{
		std::cout <<"WRONG SOLVER NAME!" << std::endl;
		return -1;
	}

	ICBSSearch icbs(ml, al, 1.0, s, vm["cutoffTime"].as<int>() * 1000, vm["screen"].as<int>());
	if (vm["solver"].as<string>() == "CBSH-RM")
	{
		icbs.rectangleMDD = true;
	}
	if (vm.count("cardinalRect"))
	{
		icbs.cardinalRect = vm["cardinalRect"].as<bool>();
	}
	if (vm.count("corridor2"))
	{
		icbs.corridor2 = vm["corridor2"].as<bool>();
	}
	if (vm.count("corridor4"))
	{
		icbs.corridor4 = vm["corridor4"].as<bool>();
	}
	if (vm.count("cardinalCorridor"))
	{
		icbs.cardinalCorridorReasoning = vm["cardinalCorridor"].as<bool>();
	}
	if (vm.count("target"))
	{
		icbs.targetReasoning = vm["target"].as<bool>();
	}
	bool res;
	res = icbs.runICBSSearch();
	ofstream stats;
	stats.open(vm["output"].as<string>(), ios::app);
	stats << icbs.runtime << "," <<
		icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
		icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
		vm["agents"].as<string>() << "," << icbs.solution_cost << "," << 
		icbs.min_f_val - icbs.dummy_start->g_val << "," <<
		vm["solver"].as<string>()  << "," <<
		icbs.num_standard << "," << icbs.num_rectangle << "," <<
		icbs.num_corridor2 << "," << icbs.num_corridor4 << "," << icbs.num_target << endl;
	stats.close();

	return 0;

}
