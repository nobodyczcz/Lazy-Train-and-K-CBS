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
		("corridor", po::value<bool>()->default_value(false), "reason about 2-way branching corridor conflicts")
		("target", po::value<bool>()->default_value(false), "reason about target conflict")
		("parking", po::value<bool>()->default_value(false), "reason about target conflict")
		("kDelay", po::value<int>()->default_value(0), "Set max_k for k robust plan")
		("shrink", "shrink to hole on reaching target")
		("ignore-target",  "ignore all target conflict")
		("ignore-train", "ignore train conflict, act only robust cbs")
		("no-train-classify", "ignore train conflict, act only robust cbs")
		("lltp-only",  "only use lltp and occupation conflict find solution.")
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

	options1.debug = vm.count("debug");
	options1.printFailedPair = vm.count("printFailedPair");
	options1.pairAnalysis = vm.count("pairAnalysis");
    options1.ignore_target = vm.count("ignore-target");
    options1.lltp_only = vm.count("lltp-only");
    options1.parking = vm["parking"].as<bool>();;
    options1.shrink = vm.count("shrink");
    options1.ignore_train = vm.count("ignore-train");
    options1.no_train_classify = vm.count("no-train-classify");



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
	if (vm["screen"].as<int>() >= 1) {
	    cout<<"Valid Train Plan: "<< validTrain<<" body conflicts: "<<icbs.num_body_conflict<<" goal conflict: "<<icbs.num_goal_conflict<<" self conflict: "<<icbs.num_self_conflict<<endl;
	}

	icbs.write_data(vm["output"].as<string>(),vm["agents"].as<string>(),vm["solver"].as<string>(), validTrain);

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
	return 0;

}
