#!/usr/bin/env python

#The compiled python lib must be placed together with this script.

import numpy as np
import os, random, subprocess,time,glob
from multiprocessing import Process
from itertools import product
from flatland.envs.observations import GlobalObsForRailEnv
# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator,rail_from_file
from flatland.envs.schedule_generators import sparse_schedule_generator,schedule_from_file
from flatland.envs.malfunction_generators  import malfunction_from_params, MalfunctionParameters,malfunction_from_file

# We also include a renderer because we want to visualize what is going on in the environment
from flatland.utils.rendertools import RenderTool, AgentRenderVariant

from flatland.evaluators.client import FlatlandRemoteClient
from flatland.core.env_observation_builder import DummyObservationBuilder

from flatland.envs.agent_utils import RailAgentStatus
from flatland.envs.rail_env_shortest_paths import get_shortest_paths
from flatland.envs.predictions import ShortestPathPredictorForRailEnv

# We also include a renderer because we want to visualize what is going on in the environment
from flatland.utils.rendertools import RenderTool, AgentRenderVariant
from libPythonCBS import PythonCBS


def cbs(width,height,nr_trains,cities_in_map,seed, algo,k,t, corridor, diff_k, lltp_only, out):
    grid_distribution_of_cities = False  # Type of city distribution, if False cities are randomly placed
    max_rails_between_cities = 2  # Max number of tracks allowed between cities. This is number of entry point to a city
    max_rail_in_cities = 3  # Max number of parallel tracks within a city, representing a realistic trainstation

    rail_generator = sparse_rail_generator(max_num_cities=cities_in_map,
                                           grid_mode=grid_distribution_of_cities,
                                           max_rails_between_cities=max_rails_between_cities,
                                           max_rails_in_city=max_rail_in_cities,
                                           )

    # The schedule generator can make very basic schedules with a start point, end point and a speed profile for each agent.
    # The speed profiles can be adjusted directly as well as shown later on. We start by introducing a statistical
    # distribution of speed profiles

    # Different agent types (trains) with different speeds.
    speed_ration_map = {1.: 1,  # Fast passenger train
                        1. / 2.: 0,  # Fast freight train
                        1. / 3.: 0,  # Slow commuter train
                        1. / 4.: 0}  # Slow freight train

    # We can now initiate the schedule generator with the given speed profiles

    schedule_generator = sparse_schedule_generator(speed_ration_map)

    # We can furthermore pass stochastic data to the RailEnv constructor which will allow for stochastic malfunctions
    # during an episode.
    stochastic_data = MalfunctionParameters(malfunction_rate=0,  # Rate of malfunction occurence
                                            min_duration=20,  # Minimal duration of malfunction
                                            max_duration=50  # Max duration of malfunction
                                            )
    #print(stochastic_data)

    # Custom observation builder without predictor
    observation_builder = DummyObservationBuilder

    # Custom observation builder with predictor, uncomment line below if you want to try this one
    # observation_builder = TreeObsForRailEnv(max_depth=2, predictor=ShortestPathPredictorForRailEnv())

    # Construct the enviornment with the given observation, generataors, predictors, and stochastic data
    env = RailEnv(width=width,
                  height=height,
                  rail_generator=rail_generator,
                  schedule_generator=schedule_generator,
                  number_of_agents=nr_trains,
                  random_seed = seed,
                  obs_builder_object=DummyObservationBuilder(),
                  remove_agents_at_target=True  # Removes agents at the end of their journey to make space for others
                  )
    env.reset()

    cbs = PythonCBS(env, algo, k, t, 0, 1.0, corridor, diff_k, lltp_only)
    cbs.search()
    cbs.write_result(out)
    
    
processPool = []
maps = [
        ("small",[4,28,4,25,[50,10]]),
        ("large",[10,50,5,25,[100,20]]),ß
]
for mapname,setting in maps:
    print(mapname,setting)
    outputFolder = os.path.abspath('./2022-may-variant1-m1/{}-90'.format(mapname))

    try:
        os.makedirs(outputFolder)
    except:
        pass

    algos = ["CBS","CBSH-RM"]#"CBS","CBSH-CR"
    time_limits = [90]

    count = 0

    for time_limit in time_limits:
        for algo in algos:
            for lltp_only in [False]:
                if (lltp_only and algo != "CBS"):
                    continue
                for shrink in [True]:
                    for ignore_target in [True]:
                        for target in [False]:
                            if(target and algo != "CBSH-RM"):
                                continue
                            for corri in [False,True]:
                                if(corri and algo != "CBSH-RM"):
                                    continue
                                if (not corri and algo == "CBSH-RM"):
                                    continue
                                if (not corri and target):
                                        continue
                                for ignore_train in [False, True]:
                                    if (ignore_train and algo != "CBSH-RM" and not corri):
                                        continue
                                    
                                    for parking in [False]:
                                        if (parking and algo != "CBSH-RM"):
                                            continue
                                        if (parking and not target):
                                            continue
                                        if (parking and shrink):
                                            continue
                                        if ((parking or target) and ignore_target):
                                            continue
                                        for instance in range(0,25):
                                            for agentsNo in range(setting[0],setting[1],setting[2]):

                                                for k in [4,8]:
                                                    out = os.path.join(outputFolder,
                                                                   'algo={}_agents={}_ins={}_k={}_shrink={}_ignore-target={}_corridor={}_target={}_parking={}_lltp-only={}_ignore-train={}'.format(algo+"-DK",agentsNo,instance,k,shrink,ignore_target,corri,target,parking, lltp_only, ignore_train))
#                                                     if os.path.exists(out):
# #                                                         print("exist")
#                                                         continue
                            #                             with open(out,"r") as f:
                            #                                 lines = f.readlines()
                            #                                 if len(lines)!=0:
                            # #                                     print("Pass: agents: {} instance:{} algo: {} k dealy: {}".format(agentsNo,instance,algo,k))
                            #                                     continue

                                                    
                                                    po = Process(target=cbs, args=(setting[-1][0],setting[-1][0],agentsNo,setting[-1][1],instance if instance != 0 else 25, algo,k,time_limit, corri, True, lltp_only, out))
                                                    if (len(processPool)>= setting[3]):
                                                        finish = False
                                                        while not finish:
                                                            time.sleep(0.5)
                                                            for p in range(0,len(processPool)):
                                                                if p >= len(processPool):
                                                                    break
                                                                if not processPool[p].is_alive():
                                                                    processPool.pop(p)
                                                                    p-=1
                                                                    finish = True
                                                    else:
                                                        for p in range(0,len(processPool)):
                                                            if p >= len(processPool):
                                                                break
                                                            if not processPool[p].is_alive():
                                                                processPool.pop(p)
                                                                p-=1
                                                    print("Start: ",mapname, ' algo={}_agents={}_ins={}_k={}_shrink={}_ignore-target={}_corridor={}_target={}_parking={}_lltp-only={}_ignore-train={}'.format(algo+"-DK",agentsNo,instance,k,shrink,ignore_target,corri,target,parking, lltp_only, ignore_train))
                                                    po.start()
                                                    processPool.append(po)


