import xml.etree.ElementTree as ET
import numpy as np
from dataclasses import dataclass

def parse_world(file_path: str) -> ET.ElementTree:
    """
    Parses the world file in file_path and returns it as an xml object
    """
    with open(file_path) as f:
        return ET.parse(f)

def add_model(world: ET.ElementTree, model: ET.Element) -> None:
    """
    Adds a model to the world within the world element
    """
    world.getroot()[0].append(model)

@dataclass
class PostModel:
    tag_id: int
    position: np.ndarray
    yaw: float

def generate_post_model_xml(post: PostModel) -> ET.Element:
    """
    Generates the xml for a given post model assuming the .dae file follows the format 4x4_1000-<id>.dae
    """
    x, y, z = post.position
    yaw = post.yaw
    tag_id = post.tag_id
    xml_str = f"""
    <model name='post_{tag_id}'>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://post/meshes/4x4_1000-{tag_id}.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode />
            </contact>
            <bounce />
            <friction>
              <torsional>
                <ode />
              </torsional>
              <ode />
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://post/meshes/4x4_1000-{tag_id}.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    """
    return ET.fromstring(xml_str)

def write_world(world: ET.ElementTree, file_path: str) -> None:
    """
    Writes the world to the file path
    """
    world.write(file_path)

def add_post(world: ET.ElementTree, post: PostModel) -> None:
    """
    Adds a post to the world
    """
    post_xml = generate_post_model_xml(post)
    add_model(world, post_xml)

def main():
    world = parse_world("config/gazebo/env_description/world_empty.world")
    post1 = PostModel(1, np.array([10.0, -5.5, 0.5]), 4.71239)
    post2 = PostModel(2, np.array([-3.5, 15.5, 0.5]), 4.71239)
    post3 = PostModel(3, np.array([13.5, 5.5, 0.5]), 4.71239)
    post4 = PostModel(4, np.array([-3.5, -15.5, 0.5]), 4.71239)
    post5 = PostModel(5, np.array([-5.5, -15.5, 0.5]), 4.71239)
    posts = [post1, post2, post3, post4, post5]
    for post in posts:
        add_post(world, post)
    write_world(world, "config/gazebo/env_description/world_generated.world")

if __name__ == "__main__":
    main()