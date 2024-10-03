#!/usr/bin/env python3

import os
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation import ConstantVelocityExecutor, OccupancyGrid, PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.planning.actions import TaskAction, TaskPlan

data_folder = get_data_folder()


class RobotWithBattery(Robot):
    def __init__(self, name, radius, path_executor, grasp_generator, battery_capacity, consumption_rate, node):
        super().__init__(name=name, radius=radius, path_executor=path_executor, grasp_generator=grasp_generator)
        self.battery_capacity = battery_capacity
        self.consumption_rate = consumption_rate
        self.battery_level = battery_capacity
        self.node = node  # Nodo ROS2 per la registrazione e la pubblicazione dei messaggi
        self.battery_publisher = node.create_publisher(Float32, f"{self.name}/battery_level", 10)
        self.action_publisher = node.create_publisher(String, f"{self.name}/action_log", 10)

    def execute_plan(self, plan):
        """Esegue un piano di azioni e logga ogni azione."""
        for action in plan.actions:
            result = self.execute_action(action)  # Funzione per eseguire l'azione
            self.log_action(plan)  # Logga ogni azione
            if result is not None:  # Se ci sono risultati da elaborare
                return result
    def update_battery(self, time_elapsed):
        """Aggiorna il livello della batteria in base al tempo trascorso."""
        self.battery_level -= self.consumption_rate * time_elapsed
        if self.battery_level < 0:
            self.battery_level = 0
        self.publish_battery_status()

    def is_battery_empty(self):
        """Controlla se la batteria è scarica."""
        return self.battery_level <= 0

    def go_standby(self):
        """Metti il robot in standby e logga l'azione."""
        action_msg = String()
        action_msg.data = f"{self.name} is in standby due to low battery."
        self.action_publisher.publish(action_msg)
        print(action_msg.data)

    def publish_battery_status(self):
        """Pubblica lo stato della batteria come messaggio ROS."""
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_publisher.publish(battery_msg)

    def log_action(self, action_description):
        """Logga e pubblica un'azione eseguita dal robot."""
        action_msg = String()
        action_msg.data = f"{self.name} executed action: {action_description}"
        self.action_publisher.publish(action_msg)
        print(action_msg.data)

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
        parent=counter,
        pose=Pose(x=1.0, y=-0.5, z=0.0, q=[0.9238811, 0.0, 0.0, 0.3826797]),
    )
    world.add_object(
        category="apple", parent=table, pose=Pose(x=3.2, y=3.5, z=0.0, yaw=0.0)
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
        node=node,
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
    world.add_robot(robot0, loc="bathroom")

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
    rclpy.init()  # Inizializza ROS2
    node = rclpy.create_node("robot_logging_node")  # Crea un nodo ROS2

    args = parse_args()

    # Crea o carica un mondo
    if args.world_file == "":
        world = create_world(args.multirobot)
    else:
        world = create_world_from_yaml(args.world_file)

    robot = world.get_robot_by_name("robot0")

    # Esempio di piano d'azione
    actions_plan = [
        TaskAction("navigate", source_location="bathroom", target_location="table"),
        TaskAction("pick", object="apple"),
        TaskAction("navigate", source_location="table", target_location="desk"),
        TaskAction("place"),
    ]
    plan = TaskPlan(actions=actions_plan)
    success = robot.execute_plan(plan)
    if success:
        print("Plan executed successfully.")
    
    # Log delle azioni e aggiornamento della batteria
        for action in actions_plan:
            robot.log_action(actions_plan)  # Logga ogni azione
            robot.update_battery(10)  # Aggiorna la batteria dopo ogni azione
    else:
        print("Plan execution failed.")


    start_gui(world)
    rclpy.spin(node)  # Mantiene attivo il nodo ROS
    rclpy.shutdown() # Chiudi ROS2 alla fine del programma
