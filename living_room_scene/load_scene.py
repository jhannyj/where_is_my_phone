import os
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
                         ProcessModelDirectives, LoadModelDirectives)

here = os.path.dirname(os.path.abspath(__file__))
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
parser = Parser(plant)
parser.package_map().Add("living_room", here)
ProcessModelDirectives(
    LoadModelDirectives(os.path.join(here, "living_room_scene.dmd.yaml")), plant, parser)
plant.Finalize()
diagram = builder.Build()
print("bodies:", plant.num_bodies(), "| collision geometries:", plant.num_collision_geometries())
