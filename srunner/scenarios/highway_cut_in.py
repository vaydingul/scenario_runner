#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenarios in which another (opposite) vehicle 'illegally' takes
priority, e.g. by running a red traffic light.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorDestroy,  WaypointFollower, LaneChange, AccelerateToVelocity, SetInitSpeed
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, WaitUntilInFront
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.background_manager import HandleStartCutInScenario, HandleEndCutInScenario

def convert_dict_to_transform(actor_dict):
    """
    Convert a JSON string to a CARLA transform
    """
    return carla.Transform(
        carla.Location(
            x=float(actor_dict['x']),
            y=float(actor_dict['y']),
            z=float(actor_dict['z'])
        ),
        carla.Rotation(
            roll=0.0,
            pitch=0.0,
            yaw=float(actor_dict['yaw'])
        )
    )

class HighwayEntryCutIn(BasicScenario):
    """
    This class holds everything required for a scenario in which another vehicle changes lane
    abruptly in front of the ego, forcing it to react.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self._drive_distance = 120
        self._offset = 0.75
        super(HighwayEntryCutIn, self).__init__("HighwayEntryCutIn",
                                                ego_vehicles,
                                                config,
                                                world,
                                                debug_mode,
                                                criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        starting_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        # Create the cut-in vehicle
        displacement = self._offset * starting_waypoint.lane_width
        r_vec = starting_waypoint.transform.get_right_vector()
        w_loc = starting_waypoint.transform.location
        w_loc += carla.Location(x=displacement * -r_vec.x, y=displacement * -r_vec.y)
        car_transform = carla.Transform(w_loc, starting_waypoint.transform.rotation)
        other_vehicle = CarlaDataProvider.request_new_actor('vehicle.mercedes.coupe_2020', car_transform)
        self.other_actors.append(other_vehicle)

    def _create_behavior(self):
        """
        Hero vehicle is on an highway, or a road with at least two lanes, and another vehicle cuts in front of it.
        """

        root = py_trees.composites.Sequence()
        if CarlaDataProvider.get_ego_vehicle_route():
            root.add_child(HandleStartCutInScenario(self.other_actors))
        root.add_child(DriveDistance(self.ego_vehicles[0], self._drive_distance))
        if CarlaDataProvider.get_ego_vehicle_route():
            root.add_child(HandleEndCutInScenario())
        root.add_child(ActorDestroy(self.other_actors[0]))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
