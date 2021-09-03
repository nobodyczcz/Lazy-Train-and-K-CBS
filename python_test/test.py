#!/usr/bin/env python

#Compile codes in PythonCBS in folder CBS-corridor with cmake

import numpy as np
import time, glob
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

def linearize_loc(in_env, loc):
    """
    This method linearize the locaiton(x,y) into an int.
    :param in_env: local environment of flatland
    :param loc: locaiton pair(x,y)
    :return: linearized locaiton, int.
    """
    return loc[0]*in_env.width + loc[1]


width = 50  # With of map
height = 50  # Height of map
nr_trains = 24  # Number of trains that have an assigned task in the env
cities_in_map = 10  # Number of cities where agents can start or end
seed = 19  # Random seed
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
# env_renderer = RenderTool(env, gl="PGL",
#                           agent_render_variant=AgentRenderVariant.ONE_STEP_BEHIND,
#                           show_debug=True,
#                           screen_height=env.height*15,  # Adjust these parameters to fit your resolution
#                           screen_width=env.width*15)  # Adjust these parameters to fit your resolution
# env_renderer.reset()
# env_renderer.render_env(show=True, show_observations=False, show_predictions=False)

cbs = PythonCBS(env, "CBSH-RM", 2, 60, 3, 1.0, True, True, False)
cbs.search()