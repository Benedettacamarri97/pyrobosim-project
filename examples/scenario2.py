#!/usr/bin/env python3

import os
import argparse
import numpy as np
import time
from threading import Thread

from pyrobosim.core import Robot, World, WorldYamlLoader, Object
from pyrobosim.gui import start_gui 
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties 
from pyrobosim.navigation import ConstantVelocityExecutor, OccupancyGrid, PathPlanner 
from pyrobosim.utils.general import get_data_folder 
from pyrobosim.utils.pose import Pose 



data_folder = get_data_folder()


def create_world(multirobot=True):
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    world.add_room(
        name="living_room",
        footprint=r1coords,
        color=[1, 0, 0],
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )
   
    r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
    world.add_room(name="bedroom", footprint=r2coords, color=[0, 0.6, 0])
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])
    r3coords = [(4.15, -0.66), (6.13, -0.66), (6.13, 1.68), (4.15,1.68)]
    world.add_room(name="kitchen", footprint=r3coords, color=[0.6, 0, 0])


    world.add_hallway(room_start="living_room", room_end="bathroom", width=0.7)
    world.add_hallway(
       room_start="bathroom",
        room_end="bedroom",
        width=0.5,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
    )

    world.add_hallway(
        room_start="living_room",
        room_end="kitchen",
        width=0.6,
    )

    world.add_hallway(room_start="kitchen", room_end="bedroom", width=0.5)
   
    desk = world.add_location(
        name="scrivania",category="desk", parent="bedroom", pose=Pose(x=2.10, y=3.65, z=0.0, yaw=0.0)
    )
    counter = world.add_location(
        name="lavandino",
        category="counter",
        parent="bathroom",
        pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
    )

    world.add_object(
        name="dentifricio",
        category="banana",
        parent=counter,
        pose=Pose(x=-2.58, y=2.82, z=0.0),
    )

    world.add_object(
        name="libro",
        category="apple",
        parent=desk,
        pose=Pose(x=2.14, y=3.78, z=0.0),
    )

    world.add_object(name="penna",category="water", parent=desk)
    world.add_object(name="saponetta",category="apple", parent=counter)
    world.add_object(name="dentifricio",category="banana", parent=counter)
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    robot0 = Robot(
        name="robot0", 
        radius=0.08, 
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
    robot0.set_path_planner(rrt_planner)
    world.add_robot(robot0, loc="kitchen")

    robot1 = Robot(
        name="robot1",
        radius=0.08, 
        color=(0.8, 0.8, 0),
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            #validate_during_execution=False,
        ),
        grasp_generator=GraspGenerator(grasp_props),
        #partial_observability=args.partial_observability,
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
    world.add_robot(robot1, loc="bathroom")

    robot2 = Robot(
        name="robot2",
        radius=0.08, 
        color=(0.8, 0.8, 0),
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            #validate_during_execution=False,
        ),
        grasp_generator=GraspGenerator(grasp_props),
        #partial_observability=args.partial_observability,
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
    world.add_robot(robot2, loc="bedroom")

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