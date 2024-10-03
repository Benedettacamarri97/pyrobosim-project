import pyrobosim.core as prb
import os
import argparse
import numpy as np

from pyrobosim.core import Robot, World, WorldYamlLoader , Object
from pyrobosim.gui import start_gui 
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties 
from pyrobosim.navigation import ConstantVelocityExecutor, OccupancyGrid, PathPlanner 
from pyrobosim.utils.general import get_data_folder 
from pyrobosim.utils.pose import Pose 
from pyrobosim.planning.actions import TaskAction

data_folder = get_data_folder()


def create_world(multirobot=False):

    world = World()

    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Aggiungi stanze (Cucina, Salotto, Stanza di riposo)
    r1= [(-4,1),(-4,3),(-2,3),(-2,1)]
    world.add_room(
        name="kitchen",
        footprint=r1,
        color=[1, 0, 0]
    )
    r2=[(0,0),(0,2),(2,2),(2,0)]
    world.add_room(
        name="living_room",
        footprint=r2,
        color=[0, 0.6, 0]
    )
    r3=[(4,0),(4,1.50),(6,1.50),(6,0)]
    world.add_room(
        name="rest_area",
        footprint=r3,
        color=[0, 0, 0.6]
    )
    table = world.add_location(
        category="table",
        parent="kitchen",
        pose=Pose(x=-3, y=1.50, z=0.0, yaw=-np.pi / 2.0),
    )
    desk = world.add_location(
        name= "desk1",
        category="desk",
        parent="rest_area",
        pose=Pose(x=5, y=0.50, z=0.0, yaw=-np.pi / 2.0),
    )
    desk = world.add_location(
        category="desk",
        parent="living_room",
        pose=Pose(x=1, y=0.50, z=0.0, yaw=-np.pi / 2.0),
    )

    world.add_hallway(room_start="kitchen", room_end="living_room", width=0.4)
    world.add_hallway(room_start="living_room", room_end="rest_area", width=0.7)
    world.add_hallway(room_start="kitchen", room_end="rest_area", width=0.7)

    world.add_object(
        category="banana",
        parent=table,
        pose=Pose(x=-3.20, y=1.60, z=0.0),
    )

    # Aggiungi un oggetto che il robot dovrà raccogliere
    #world.add_object("kitchen", "apple", [2, 2])
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    robot = Robot(
        name="robot0",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
        ),
        grasp_generator=GraspGenerator(grasp_props),
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
    world.add_robot(robot, loc="kitchen")

    return world
def navigate_to(world, robot, target_location):
    print(f"Il robot sta navigando verso {target_location}")
    # Imposta la posizione direttamente
    target_obj = world.get_location_by_name(target_location)
    navigate_action = TaskAction(
        "navigate", 
        source_location=robot.location.name,
        target_location=target_obj.name if target_obj else target_location,
    ) 
    robot.execute_action(navigate_action)
    print(f"Il robot è arrivato a {target_location}")

def pick_object(world, robot, object_name):
    obj = world.get_object_by_name(object_name)
    pick_action = TaskAction(
        "pick",
    )
    robot.execute_action(pick_action)
    print(f"Il robot ha raccolto {object_name}")

def place_object(robot, destination):
    print(f"Il robot sta consegnando l'oggetto {destination}")
    place_action = TaskAction(
        "place",
    )
    robot.execute_action(place_action)
    print("Il robot ha consegnato l'oggetto.")

def run_simulation(world, robot):
    print("Inizio della simulazione")
    
    # Navigazione verso la mela in cucina
    navigate_to(world, robot, target_location="table0")
    pick_object(world, robot, "banana")
    
    # Navigazione verso il salotto per consegnare la banana
    navigate_to(world, robot, "desk1")
    place_object(robot, "banana")
    
    print("Simulazione completata")
# Simulazione delle operazioni del robot

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
    if args.world_file == "":
        world = create_world()
    else:
        world = create_world_from_yaml(args.world_file)

    robot = world.get_robot_by_name("robot0")
    run_simulation(world, robot)
    start_gui(world)



