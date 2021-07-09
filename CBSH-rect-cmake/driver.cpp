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
#include "ICBSHSearchPairAnalysis.h"

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
		("cutoffTime,t", po::value<float>()->default_value(7200), "cutoff time (seconds)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("screen", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("corridor", po::value<bool>(), "reason about 2-way branching corridor conflicts")
		("target", po::value<bool>(), "reason about target conflict")
		("kDelay", po::value<int>()->default_value(0), "Set max_k for k robust plan")
		("diff-k",  "All agent have different k")
		("only_generate_instance", po::value<std::string>()->default_value(""),"no searching")
		("debug", "debug mode")
		("statistic","print statistic data")
		("pairAnalysis",po::value<int>(),"perform 2 agent analysis")
		("printFailedPair","print mdd and constraints for failed pair")


            ;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	int max_k = vm["kDelay"].as<int>();
	bool diff_k = vm.count("diff-k");

	if (vm["screen"].as<int>() == 2) {
		cout << "[CBS] Loading map and agents " << endl;
	}
	// read the map file and construct its two-dim array
	MapLoader* ml = new MapLoader(vm["map"].as<string>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), *ml,max_k,diff_k, vm["agentNum"].as<int>());
	if (vm["screen"].as<int>() == 2) {
		cout << "[CBS] Loading map and agents don2! " << endl;
	}
	srand(vm["seed"].as<int>());

	options options1;

	if (vm.count("debug")) {
		options1.debug = true;
	}
	else {
		options1.debug = false;
	}

	if (vm.count("printFailedPair")){
	    options1.printFailedPair = true;
	}


    if (vm.count("pairAnalysis")) {
        options1.pairAnalysis = true;
    }


	if (vm["only_generate_instance"].as<string>()!="") {
		al.saveToFile(vm["only_generate_instance"].as<string>());
		return 0;
	}

	constraint_strategy s;
	if (vm["solver"].as<string>() == "ICBS")
		s = constraint_strategy::ICBS;
	else if (vm["solver"].as<string>() == "CBS")
		s = constraint_strategy::CBS;
	else if (vm["solver"].as<string>() == "CBSH")
		s = constraint_strategy::CBSH;
	else if (vm["solver"].as<string>() == "CBSH-RM")
		s = constraint_strategy::CBSH_RM;
	else
	{
		std::cout <<"WRONG SOLVER NAME!" << std::endl;
		return -1;
	}

    ICBSSearchWithPairedAnalysis<MapLoader> icbs(ml, al, 1.0, s, vm["cutoffTime"].as<float>() * CLOCKS_PER_SEC, vm["screen"].as<int>(), vm["kDelay"].as<int>(), options1);
	if (vm["solver"].as<string>() == "CBSH-RM")
	{
		icbs.rectangleMDD = true;
	}

	if (vm.count("corridor"))
	{
		icbs.corridor2 = vm["corridor"].as<bool>();
	}

	if (vm.count("target"))
	{
		icbs.targetReasoning = vm["target"].as<bool>();

	}

    if (vm.count("pairAnalysis")) {
        icbs.analysisEngine = new ICBSSearchWithPairedAnalysis<MapLoader>(&icbs, vm["pairAnalysis"].as<int>());
        icbs.analysisOutput = ofstream();
        icbs.analysisOutput.open(vm["output"].as<string>()+".FailedPairs",ios::trunc);
        icbs.analysisOutput<<"["<<endl;
    }
	
	bool res;
	res = icbs.runICBSSearch();
	bool validTrain = icbs.isValidTrain();
	ofstream stats;
	stats.open(vm["output"].as<string>(), ios::trunc);
    if (vm["screen"].as<int>() >= 1) {
        cout<<"Valid Train Plan: "<< validTrain<<" body conflicts: "<<icbs.num_body_conflict<<" goal conflict: "<<icbs.num_goal_conflict<<" self conflict: "<<icbs.num_self_conflict<<endl;
    }
	stats  << icbs.runtime/ CLOCKS_PER_SEC << "," <<
		icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
		icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
		vm["agents"].as<string>() << "," << icbs.solution_cost << "," << 
		icbs.min_f_val - icbs.dummy_start->g_val << "," <<
		vm["solver"].as<string>()  << "," <<
		icbs.num_standard <<","<<icbs.num_train_standard<< "," << icbs.num_rectangle << "," <<
		icbs.num_corridor2 << "," << icbs.num_corridor4 << "," << 
		icbs.num_target<<","<< icbs.num_chasingRectangle << "," <<
		icbs.less10 << ","<<icbs.less100 << ","<<icbs.less1000 << ","<<icbs.less10000 <<"," << icbs.less100000 <<","
		<<icbs.larger100000 << "," <<icbs.num_pairs <<"," <<icbs.num_failed << ","<< validTrain<<","
		<<icbs.num_body_conflict<<","<<icbs.num_goal_conflict<<","<<icbs.num_self_conflict<<","<<icbs.runtime_mdd<< endl;
	stats.close();

    if(vm.count("statistic")){
        cout<<"Total RM time: "<<icbs.RMTime/CLOCKS_PER_SEC <<endl;
        cout<<"Total time on extract sg: " <<icbs.runtime_findsg/CLOCKS_PER_SEC <<endl;
        cout<<"Total time on find rectangle: " <<icbs.runtime_find_rectangle/CLOCKS_PER_SEC <<endl;
        cout<<"Total time on build mdd in RM: " <<icbs.RMBuildMDDTime/CLOCKS_PER_SEC <<endl;

        cout<<"Total RM detection: "<<icbs.RMDetectionCount <<endl;
        cout<<"Total RM find: "<<icbs.RMSuccessCount <<endl;
        cout<<"Total RM rejected before find rectangle: "<<icbs.RMFailBeforeRec <<endl;
        cout<<"Total MDD: "<<icbs.TotalMDD <<endl;
        cout<<"Total Exist MDD: "<<icbs.TotalExistMDD <<endl;
        cout<<"Total K MDD: "<<icbs.TotalKMDD <<endl;
        cout<<"Total Exist K MDD: "<<icbs.TotalExistKMDD <<endl;
        cout<<"Repeated pairs: "<<icbs.repeated_pairs<<endl;
        for (int i=0; i<icbs.KRectangleCount.size();i++){
            cout<< "total k: "<< i <<" rectangle: " << icbs.KRectangleCount[i]<<endl;
        }
    }

    if (vm.count("pairAnalysis")) {
        icbs.analysisOutput<<"]";
        icbs.analysisOutput.close();
    }

    if (vm["screen"].as<int>() == 2)
		cout << "Done!!" << endl;
	delete ml;
	return 0;

}
