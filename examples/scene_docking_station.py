#!/usr/bin/env python3
import os
import argparse
import numpy as np 
"""
Test script showing how to build a world and use it with pyrobosim
"""
from pyrobosim.core import Robot, World, WorldYamlLoader , Object, Location
from pyrobosim.gui import start_gui 
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties 
from pyrobosim.navigation import ConstantVelocityExecutor, OccupancyGrid, PathPlanner 
from pyrobosim.utils.general import get_data_folder 
from pyrobosim.utils.pose import Pose
from pyrobosim.planning.actions import TaskAction, TaskPlan


data_folder = get_data_folder()

# Crea un mondo simulato
def create_world(multirobot=True):

    world = World()
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Aggiungi stanze (magazzino, stazione di ricarica, ecc.)
    r1= [(-4,1),(-4,3),(-2,3),(-2,1)]
    world.add_room(
        name="storage_area",
        footprint=r1,
        color=[1, 0, 0],
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )
    r2=[(0,0),(0,1.75),(2,1.75),(2,0)]
    world.add_room(
        name="charging_station",
        footprint=r2,
        color=[0, 0.6, 0]
    )
    r3=[(4,0),(4,1.50),(6,1.50),(6,0)]
    world.add_room(
        name="delivery_area",
        footprint=r3,
        color=[0, 0, 0.6]
    )
    world.add_hallway(room_start="storage_area", room_end="charging_station", width=0.4)
    world.add_hallway(
       room_start="charging_station",
        room_end="delivery_area",
        width=0.35,
    )

    world.add_hallway(
        room_start="storage_area",
        room_end="delivery_area",
        width=0.35,
        conn_method="points",
        conn_points=[(5, 1.46), (4.70, 2.80), (-2, 2.80)],
    )

    # Aggiungi ostacoli
    table = world.add_location(
        name="docking_station",
        category="table",
        parent="storage_area",
        pose=Pose(x=-3, y=1.50, z=0.0, yaw=-np.pi / 2.0),
    )
    
    desk = world.add_location(
        category="desk",
        parent="storage_area",
        pose=Pose(x=-3.00, y=2.65, z=0.0, yaw=0.0),
    )

    counter1 = world.add_location(
        name="desk1",
        category="desk",
        parent="charging_station",
        pose=Pose(x=0.94, y=0.35, z=0.0, yaw=0.0),
    )

    counter = world.add_location(
        name="macchina_smistamento",
        category="counter",
        parent="delivery_area",
        pose=Pose(x=5, y=0.39, z=0.0),
    )

    world.add_object(
        name="pacchetto",
        category="banana",
        parent=table,
        pose=Pose(x=-3.20, y=1.60, z=0.0),
    )

    world.add_object(
        name="scatolone",
        category="apple",
        parent=desk,
        pose=Pose(x=-2.87, y=2.73, z=0.0),
    )

    world.add_object(name="pacco_grande",category="banana", parent=counter)
    world.add_object(name="pacco_piccolo",category="water", parent=counter)
    world.add_object(name="inventario_oggetti",category="banana", parent=table)
    world.add_object(name="inventario_consegne",category="banana", parent=counter1)



    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )

    robot0 = Robot(
        name="robot0",
        radius=0.1,
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
    world.add_robot(robot0, loc="storage_area")

    
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
    world.add_robot(robot1, loc="charging_station")

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
    world.add_robot(robot2, loc="delivery_area")

    return world

object_destinations = {
    "pacchetto": {
        "categoria": "banana",
        "destinazione": "macchina_smistamento"
    },
    "scatolone": {
        "categoria": "apple",
        "destinazione": "docking_station"
    },
    "pacco_grande": {
        "categoria": "banana",
        "destinazione": "macchina_smistamento"
    },
    "pacco_piccolo": {
        "categoria": "water",
        "destinazione": "desk1"
    },
    "inventario_oggetti": {
        "categoria": "banana",
        "destinazione": "desk"
    },
    "inventario_consegne": {
        "categoria": "banana",
        "destinazione": "docking_station"
    }
}

# Funzione per far prendere un oggetto al robot
def pick_and_place(robot, obj_name, world):
    #cerca l'oggetto nel dizionario
    obj_info = object_destinations.get(obj_name)
    if obj_info:
        obj_category = obj_info["categoria"]
        destination = obj_info["destinazione"]
        # Ottieni l'oggetto e la sua posizione
        obj = world.get_object_by_name(obj_name)
        obj_location = obj.parent  # La location (es. table, desk) dove si trova l'oggetto
        #obj_pose = obj.pose  # La posizione esatta dell'oggetto
        
        actions = [
            TaskAction(
                "navigate",
                source_location=robot.location.name,
                target_location=obj.parent.name,
            ),
            TaskAction("pick", object=obj_name),
            TaskAction(
                "navigate",
                source_location=obj.parent.name,
                target_location=destination,
            ),
            TaskAction("place"),
        ]

        plan = TaskPlan(actions=actions)
        result, num_completed = robot.execute_plan(plan)
        if destination == "docking_station":
            second_actions=[
                TaskAction(
                    "navigate",
                    robot="robot1",
                    source_location=robot1.location.name,
                    target_location="docking_station",
                ),
                TaskAction(
                    "pick",
                    object=obj_name,
                ),
                TaskAction(
                    "navigate",
                    source_location="docking_station",
                    target_location="desk1"
                ),
                TaskAction(
                    "place",
                    object=obj_name,
                ),
            ]
            plan = TaskPlan(actions=second_actions)
            result, num_completed = robot1.execute_plan(plan)

#non usato
def get_room(self, room_name):
    for room in self.rooms:
        if room.name == room_name:
            return room
    return None

#non usato
def get_navigate(self):
    return self.nav_poses if hasattr(self, 'nav_poses') else []

#non usato
def navigate_to_room(robot, room_name, world):
   
    room = world.get_room(room_name)
    if room:
        nav_poses = room.get_navigate() or [room.pose]
        print(f"{robot.name} naviga verso la stanza {room_name}.")
        robot.execute_action("navigate", nav_poses[0])  # Usa execute_action per navigare
    else:
        print(f"Stanza {room_name} non trovata.")

#non usato
def navigate_to(robot, target_position):
    print(f"Il robot sta navigando verso {target_position}")
    # Imposta la posizione direttamente
    robot.pose = target_position  # Aggiorna manualmente la posizione del robot
    print(f"Il robot è arrivato a {target_position}")

#non usato
def navigate_to_robot(robot, target_robot_name, world):
    target_robot = world.get_robot_by_name(target_robot_name)
    if target_robot:
        print(f"{robot.name} naviga verso il robot {target_robot_name}.")
        robot.execute_action("navigate", target_robot.pose)  # Usa execute_action per navigare
    else:
        print(f"Robot {target_robot_name} non trovato.")

#non usato
def go_to(robot, destination, world):
    print(f"{robot.name} si sta muovendo verso {destination}")
    pose = world.get_location(destination)
    if pose:
        robot.execute_action("navigate", pose)  # Usa execute_action per navigare
    else:
        print(f"Destinazione {destination} non trovata.")

#non usato
def get_location(self, destination):
    # Cerca la stanza con il nome corrispondente
    room = self.get_room(destination) #mia
    if room:
        # Se viene trovata la stanza, restituisci la stanza stessa o una posizione navigabile
        return room.get_navigate()[0] if room.get_navigate() else room.pose #mia
    
    # Cerca un'altra location (ad esempio un tavolo, scaffale, ecc.)
    location = self.get_location_by_name(destination) #
    if location:
        return location.pose
    
    print(f"Destinazione {destination} non trovata nel mondo.")
    return None
#non usato
def get_carried_object(self):
    if self.carried_object is not None:
        return self.carried_object
    else:
        print(f"{self.name} non sta trasportando nessun oggetto.")
        return None
#non usato   
def set_parent(self, new_parent):
    self.parent = new_parent

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

    robot0 = world.get_robot_by_name("robot0")
    robot1 = world.get_robot_by_name("robot1")
    robot2 = world.get_robot_by_name("robot2")
    # Simula le operazioni dei robot
    #pick_and_place(robot0, "pacco_piccolo", world)  # Robot 0 consegna un pacchetto
    pick_and_place(robot0, "scatolone",  world)   # Robot 1 consegna uno scatolone
    #pick_and_place(robot2, "scatolone", world)  # Robot 2 consegna un pacco grande
    # Start the program either as ROS node or standalone.
    start_gui(world)


