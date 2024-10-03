#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with pyrobosim
"""
import os
import argparse
import numpy as np 

from pyrobosim.core import Robot, World, WorldYamlLoader 
from pyrobosim.gui import start_gui 
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties 
from pyrobosim.navigation import ConstantVelocityExecutor, OccupancyGrid, PathPlanner 
from pyrobosim.utils.general import get_data_folder 
from pyrobosim.utils.pose import Pose 

from pyrobosim.planning.actions import TaskAction, TaskPlan


data_folder = get_data_folder()

class RobotWithBattery(Robot):
    def __init__(self, name, radius, path_executor, grasp_generator, battery_capacity, consumption_rate):
        super().__init__(name=name, radius=radius, path_executor=path_executor, grasp_generator=grasp_generator)
        self.battery_capacity = battery_capacity
        self.consumption_rate = consumption_rate  # quanto consuma il robot al secondo
        self.battery_level = battery_capacity  # inizializza la batteria al massimo

    def update_battery(self, time_elapsed):
        """Aggiorna il livello della batteria in base al tempo trascorso."""
        self.battery_level -= self.consumption_rate * time_elapsed
        if self.battery_level < 0:
            self.battery_level = 0

    def is_battery_empty(self):
        """Controlla se la batteria è scarica."""
        return self.battery_level <= 0

    def go_standby(self):
        """Metti il robot in standby."""
        print(f"{self.name} is in standby due to low battery.")
        # Qui puoi implementare logiche aggiuntive, come fermare il movimento

def create_world(multirobot=False):
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
        name="kitchen",
        footprint=r1coords,
        color=[1, 0, 0],
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )
    r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
    world.add_room(name="bedroom", footprint=r2coords, color=[0, 0.6, 0])
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])

    # Add hallways between the rooms
    world.add_hallway(room_start="kitchen", room_end="bathroom", width=0.7)
    world.add_hallway(
        room_start="bathroom",
        room_end="bedroom",
        width=0.5,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
    )
    world.add_hallway(
        room_start="kitchen",
        room_end="bedroom",
        width=0.6,
        conn_method="points",
        conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
    )

    # Add locations
    table = world.add_location(
        category="table",
        parent="kitchen",
        pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-np.pi / 2.0),
    )
    desk = world.add_location(
        category="desk", parent="bedroom", pose=Pose(x=3.15, y=3.65, z=0.0, yaw=0.0)
    )
    counter = world.add_location(
        category="counter",
        parent="bathroom",
        pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
    )

    # Add objects
    world.add_object(
        category="banana",
        parent=table,
        pose=Pose(x=1.0, y=-0.5, z=0.0, q=[0.9238811, 0.0, 0.0, 0.3826797]),
    )
    world.add_object(
        category="apple", parent=desk, pose=Pose(x=3.2, y=3.5, z=0.0, yaw=0.0)
    )
    world.add_object(category="apple", parent=table)
    world.add_object(category="apple", parent=table)
    world.add_object(category="water", parent=counter)
    world.add_object(category="banana", parent=counter)
    world.add_object(category="water", parent=desk)
    #ho provato ad aggiungere un oggetto
    world.add_object(category="banana", parent=desk)

    # Add robots
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )

    robot0 = RobotWithBattery(
        name="robot0",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            #validate_during_execution=False,
        ),
        grasp_generator=GraspGenerator(grasp_props),
        battery_capacity=100.0,  # Capacità massima della batteria
        consumption_rate=0.5, 
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

    if multirobot:
        robot1 = Robot(
            name="robot1",
            radius=0.08,
            color=(0.8, 0.8, 0),
            path_executor=ConstantVelocityExecutor(),
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
            radius=0.06,
            color=(0, 0.8, 0.8),
            path_executor=ConstantVelocityExecutor(),
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

    robot = world.get_robot_by_name("robot0")

    actions = [
        TaskAction(
            "navigate",
            source_location="kitchen",
            target_location="desk",
        ),
        TaskAction("pick", object="banana2"),
        TaskAction(
            "navigate",
            source_location="desk",
            target_location="counter",
        ),
        TaskAction("place", "object", "banana2"),
    ]
    plan = TaskPlan(actions=actions)

    for action in actions:
        result, num_completed = robot.execute_plan(plan)
        
        # Aggiorna la batteria dopo ogni azione
        robot.update_battery(50)  # Supponiamo che ogni azione richieda 2 secondi
        print(f"Battery level after action: {robot.battery_level:.2f}")

        # Controlla se la batteria è scarica
        if robot.is_battery_empty():
            robot.go_standby()
            break

    # Controllo finale del livello della batteria
    print(f"Final battery level: {robot.battery_level:.2f}")

    # Start the program either as ROS node or standalone.
    start_gui(world)
