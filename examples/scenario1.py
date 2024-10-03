#!/usr/bin/env python3

import os
import argparse
import numpy as np
import time
from threading import Thread

from pyrobosim.core import Robot, World, WorldYamlLoader 
from pyrobosim.gui import start_gui 
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties 
from pyrobosim.navigation import ConstantVelocityExecutor, OccupancyGrid, PathPlanner 
from pyrobosim.utils.general import get_data_folder 
from pyrobosim.utils.pose import Pose 

data_folder = get_data_folder()

def create_world(multirobot=False):

    world = World()
    
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    world.add_room(
        footprint=r1coords,
        color=[1, 0, 0],
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
        name="living_room",
    )

    table = world.add_location(
        category="table",
        parent="living_room",
        pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-np.pi / 2.0),
    )
    
    #world.add_object(Object(name="table", shape="rect", width=1, height=2), room_name="living_room"))
    
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    # Add a robot to the world
    robot = Robot(
        name="robot",
        radius=0.2,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            #validate_during_execution=False,
        ),
        grasp_generator=GraspGenerator(grasp_props),
        #partial_observability=args.partial_observability,
    )
    planner_config_rrt = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    rrt_planner = PathPlanner("rrt", **planner_config_rrt)
    robot.set_path_planner(rrt_planner)
    world.add_robot(robot, loc="living_room")

    if multirobot:
        robot1 = Robot(
            name="robot1",
            radius=0.08,
            color=(0.8, 0.8, 0),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_observability=args.partial_observability,
        )
        planner_config_prm = {
            "world": world,
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 1.5,
            "max_nodes": 100,
            "compress_path": False,
        }
        prm_planner = PathPlanner("prm", **planner_config_prm)
        robot1.set_path_planner(prm_planner)
        world.add_robot(robot1, loc="living_room")

        robot2 = Robot(
            name="robot2",
            radius=0.06,
            color=(0, 0.8, 0.8),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_observability=args.partial_observability,
        )
        planner_config_astar = {
            "grid": OccupancyGrid.from_world(
                world, resolution=0.05, inflation_radius=0.15
            ),
            "diagonal_motion": True,
            "heuristic": "euclidean",
        }
        astar_planner = PathPlanner("astar", **planner_config_astar)
        robot2.set_path_planner(astar_planner)
        world.add_robot(robot2, loc="living_room")

    return world
    
    
def create_world_from_yaml(world_file):
    return WorldYamlLoader().from_yaml(os.path.join(data_folder, world_file))

def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description="Main pyrobosim demo.")
    parser.add_argument(
        "--multirobot",
        action="store_true",
        help="If no YAML file is specified, this option will add "
        "multiple robots to the world defined in this file.",
    )
    parser.add_argument(
        "--world-file",
        default="",
        help="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    parser.add_argument(
        "--partial-observability",
        action="store_true",
        help="If True, robots have partial observability and must detect objects.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Create a world or load it from file.
    if args.world_file == "":
        world = create_world(args.multirobot)
    else:
        world = create_world_from_yaml(args.world_file)

    # Start the program either as ROS node or standalone.
    start_gui(world)













